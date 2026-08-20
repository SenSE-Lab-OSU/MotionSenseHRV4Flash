/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <hal/nrf_rtc.h>

#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */

#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

#define ESB_PTX_PAYLOAD_LENGTH		8U
#define RTC0_TICKS_PER_SECOND		NRF_RTC_INPUT_FREQ
#define ORANGE_LED_DUTY_CYCLE_PERCENT	5U
#define ORANGE_LED_PERIOD_TICKS	RTC0_TICKS_PER_SECOND
#define ORANGE_LED_ON_TICKS							       \
	((ORANGE_LED_PERIOD_TICKS * ORANGE_LED_DUTY_CYCLE_PERCENT + 50U) / 100U)
#define ORANGE_LED_OFF_TICKS		(ORANGE_LED_PERIOD_TICKS - ORANGE_LED_ON_TICKS)
#define ESB_PTX_TX_INTERVAL_TICKS						       \
	DIV_ROUND_UP((uint64_t)CONFIG_ESB_PTX_TX_INTERVAL_MS * RTC0_TICKS_PER_SECOND, \
		     MSEC_PER_SEC)

BUILD_ASSERT(ORANGE_LED_ON_TICKS > 0U);
BUILD_ASSERT(ORANGE_LED_OFF_TICKS > 0U);
BUILD_ASSERT(ESB_PTX_TX_INTERVAL_TICKS > 0U);
BUILD_ASSERT(ESB_PTX_TX_INTERVAL_TICKS < (NRF_RTC_COUNTER_MAX / 2U));

static atomic_t tx_ready;
static atomic_t packet_counter;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

static uint32_t rtc0_counter_get(void)
{
	return nrf_rtc_counter_get(NRF_RTC0);
}

static uint32_t rtc0_ticks_elapsed(uint32_t start_ticks, uint32_t end_ticks)
{
	return (end_ticks - start_ticks) & NRF_RTC_COUNTER_MAX;
}

static bool rtc0_interval_elapsed(uint32_t start_ticks, uint32_t interval_ticks,
				  uint32_t now_ticks)
{
	return rtc0_ticks_elapsed(start_ticks, now_ticks) >= interval_ticks;
}

static void rtc0_timebase_start(void)
{
	/*
	 * This ESB configuration uses MPSL FEM-only support, so it does not
	 * initialize MPSL or use RTC0. The network-core system clock uses RTC1.
	 */
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_STOP);
	nrf_rtc_prescaler_set(NRF_RTC0, 0);
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_START);
}

static void event_handler(const struct esb_evt *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		atomic_inc(&packet_counter);
		atomic_set(&tx_ready, 1);
		break;

	case ESB_EVENT_TX_FAILED:
		atomic_set(&tx_ready, 1);
		break;

	default:
		break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)

static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("HF clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("HF clock could not start: %d", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

static int clocks_start(void)
{
	int err;
	int res;
	const struct device *radio_clk_dev =
		DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
	struct onoff_client radio_cli;

	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

	sys_notify_init_spinwait(&radio_cli.notify);

	err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);
	do {
		err = sys_notify_fetch_result(&radio_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Radio clock could not start: %d", res);
			return res;
		}
	} while (err == -EAGAIN);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	return 0;
}

#else
BUILD_ASSERT(false, "No clock control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

static int esb_initialize(void)
{
	int err;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4,
				  0xC5, 0xC6, 0xC7, 0xC8};
	struct esb_config config = ESB_DEFAULT_CONFIG;

	/* DPL is required by ESB for per-packet no-ACK transmissions. */
	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.payload_length = ESB_PTX_PAYLOAD_LENGTH;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	return esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
}

static int packet_send(uint32_t rtc0_ticks)
{
	uint32_t count = (uint32_t)atomic_get(&packet_counter);

	/* Payload fields: RTC0 counter followed by packet counter, both little-endian. */
	sys_put_le32(rtc0_ticks, &tx_payload.data[0]);
	sys_put_le32(count, &tx_payload.data[sizeof(rtc0_ticks)]);
	tx_payload.noack = true;

	return esb_write_payload(&tx_payload);
}

int main(void)
{
	bool orange_led_on = true;
	uint32_t last_led_state_change_ticks;
	uint32_t last_tx_ticks;
	int err;

	err = clocks_start();
	if (err) {
		return 0;
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LED initialization failed: %d", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed: %d", err);
		return 0;
	}

	rtc0_timebase_start();

	atomic_set(&tx_ready, 1);
	atomic_set(&packet_counter, 0);
	last_led_state_change_ticks = rtc0_counter_get();
	last_tx_ticks = last_led_state_change_ticks;
	err = dk_set_led(DK_LED3, orange_led_on);
	if (err) {
		LOG_ERR("Orange LED initialization failed: %d", err);
		return 0;
	}

	LOG_INF("RTC0 timebase is %u Hz; ESB interval is %u ms",
		RTC0_TICKS_PER_SECOND, CONFIG_ESB_PTX_TX_INTERVAL_MS);

	while (1) {
		uint32_t now_ticks = rtc0_counter_get();
		uint32_t led_interval_ticks = orange_led_on ? ORANGE_LED_ON_TICKS :
			ORANGE_LED_OFF_TICKS;

		if (rtc0_interval_elapsed(last_led_state_change_ticks, led_interval_ticks,
					 now_ticks)) {
			last_led_state_change_ticks = now_ticks;
			orange_led_on = !orange_led_on;
			err = dk_set_led(DK_LED3, orange_led_on);
			if (err) {
				LOG_ERR("Orange LED update failed: %d", err);
			}
		}

		if (rtc0_interval_elapsed(last_tx_ticks, ESB_PTX_TX_INTERVAL_TICKS,
					 now_ticks)) {
			last_tx_ticks = now_ticks;
			if (atomic_cas(&tx_ready, 1, 0)) {
				err = packet_send(now_ticks);
				if (err) {
					atomic_set(&tx_ready, 1);
					LOG_ERR("ESB payload write failed: %d", err);
				}
			}
		}

		k_sleep(K_MSEC(1));
	}
}
