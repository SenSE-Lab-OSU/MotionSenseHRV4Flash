/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_rtc.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_gpiote.h>

LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

#define ESB_PTX_PAYLOAD_LENGTH		8U
#define RTC0_TICK_RATE_HZ		CONFIG_ESB_PTX_RTC0_TICK_RATE_HZ
#define RTC0_PRESCALER		NRF_RTC_FREQ_TO_PRESCALER(RTC0_TICK_RATE_HZ)
#define ORANGE_LED_DUTY_CYCLE_PERCENT	5U
#define ORANGE_LED_PERIOD_MS		1000U
#define ORANGE_LED_ON_MS		\
	((ORANGE_LED_PERIOD_MS * ORANGE_LED_DUTY_CYCLE_PERCENT) / 100U)
#define ORANGE_LED_OFF_MS		(ORANGE_LED_PERIOD_MS - ORANGE_LED_ON_MS)
#define RTC0_NODE			DT_NODELABEL(rtc0)
#define RTC0_TX_IRQ_PRIORITY		DT_IRQ(RTC0_NODE, priority)
#define PACKET_STROBE_NODE		DT_NODELABEL(packet_strobe)
#define PACKET_STROBE_PORT_NODE		DT_PARENT(PACKET_STROBE_NODE)
#define PACKET_STROBE_GPIOTE_INST	\
	DT_PROP(DT_PHANDLE(PACKET_STROBE_PORT_NODE, gpiote_instance), instance)
#define PACKET_STROBE_PSEL		\
	NRF_GPIO_PIN_MAP(DT_PROP(PACKET_STROBE_PORT_NODE, port), \
			 DT_GPIO_HOG_PIN_BY_IDX(PACKET_STROBE_NODE, 0))
#define TX_INTERRUPT_MARKER_NODE	DT_NODELABEL(future_output)
#define TX_INTERRUPT_MARKER_PORT_NODE	DT_PARENT(TX_INTERRUPT_MARKER_NODE)
#define TX_INTERRUPT_MARKER_PSEL	\
	NRF_GPIO_PIN_MAP(DT_PROP(TX_INTERRUPT_MARKER_PORT_NODE, port), \
			 DT_GPIO_HOG_PIN_BY_IDX(TX_INTERRUPT_MARKER_NODE, 0))

BUILD_ASSERT((NRF_RTC_INPUT_FREQ % RTC0_TICK_RATE_HZ) == 0U);
BUILD_ASSERT(ORANGE_LED_ON_MS > 0U);
BUILD_ASSERT(ORANGE_LED_OFF_MS > 0U);
BUILD_ASSERT(RTC0_TX_IRQ_PRIORITY >= CONFIG_ESB_EVENT_IRQ_PRIORITY,
	     "RTC0 interrupt priority must not preempt ESB events");
BUILD_ASSERT(IS_ENABLED(_CONCAT(CONFIG_, _CONCAT(NRFX_GPIOTE,
						 PACKET_STROBE_GPIOTE_INST))),
	     "The packet strobe GPIOTE instance must be enabled");

static atomic_t tx_ready;
static atomic_t packet_counter;
static const nrfx_gpiote_t packet_strobe_gpiote =
	NRFX_GPIOTE_INSTANCE(PACKET_STROBE_GPIOTE_INST);
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

static uint32_t rtc0_counter_get(void)
{
	return nrf_rtc_counter_get(NRF_RTC0);
}

static void rtc0_timebase_start(void)
{
	/*
	 * This ESB configuration uses MPSL FEM-only support, so it does not
	 * initialize MPSL or use RTC0. The network-core system clock uses RTC1.
	 */
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_STOP);
	nrf_rtc_prescaler_set(NRF_RTC0, RTC0_PRESCALER);
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_CLEAR);
	nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_TICK);
	nrf_rtc_event_enable(NRF_RTC0, NRF_RTC_INT_TICK_MASK);
	nrf_rtc_int_enable(NRF_RTC0, NRF_RTC_INT_TICK_MASK);
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_START);
}

static int packet_strobe_initialize(void)
{
	const nrfx_gpiote_output_config_t output_config =
		NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
	nrfx_gpiote_task_config_t task_config;
	uint8_t gpiote_channel;
	uint8_t ready_dppi_channel;
	uint8_t end_dppi_channel;
	nrfx_err_t nrfx_err;

	if (!nrfx_gpiote_init_check(&packet_strobe_gpiote)) {
		nrfx_err = nrfx_gpiote_init(&packet_strobe_gpiote, 0);
		if (nrfx_err != NRFX_SUCCESS) {
			LOG_ERR("Packet strobe GPIOTE initialization failed: 0x%08x", nrfx_err);
			return -EIO;
		}
	}

	nrfx_err = nrfx_gpiote_channel_alloc(&packet_strobe_gpiote, &gpiote_channel);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Packet strobe GPIOTE channel allocation failed: 0x%08x",
			nrfx_err);
		return -ENOMEM;
	}

	task_config.task_ch = gpiote_channel;
	task_config.polarity = NRF_GPIOTE_POLARITY_TOGGLE;
	task_config.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW;
	nrfx_err = nrfx_gpiote_output_configure(&packet_strobe_gpiote,
						   PACKET_STROBE_PSEL, &output_config, &task_config);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Packet strobe GPIOTE configuration failed: 0x%08x", nrfx_err);
		return -EIO;
	}

	nrfx_err = nrfx_gppi_channel_alloc(&ready_dppi_channel);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Packet strobe READY DPPI channel allocation failed: 0x%08x",
			nrfx_err);
		return -ENOMEM;
	}

	nrfx_err = nrfx_gppi_channel_alloc(&end_dppi_channel);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Packet strobe END DPPI channel allocation failed: 0x%08x",
			nrfx_err);
		return -ENOMEM;
	}

	nrfx_gpiote_out_task_enable(&packet_strobe_gpiote, PACKET_STROBE_PSEL);
	/* ESB's READY_START shortcut starts the packet at the READY event. */
	nrfx_gppi_channel_endpoints_setup(
		ready_dppi_channel,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_READY),
		nrfx_gpiote_set_task_address_get(&packet_strobe_gpiote, PACKET_STROBE_PSEL));
	nrfx_gppi_channel_endpoints_setup(
		end_dppi_channel,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_END),
		nrfx_gpiote_clr_task_address_get(&packet_strobe_gpiote, PACKET_STROBE_PSEL));
	nrfx_gppi_channels_enable(BIT(ready_dppi_channel) | BIT(end_dppi_channel));

	return 0;
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
	int err;

	/* Payload fields: RTC0 counter followed by packet counter, both little-endian. */
	sys_put_le32(rtc0_ticks, &tx_payload.data[0]);
	sys_put_le32(count, &tx_payload.data[sizeof(rtc0_ticks)]);
	tx_payload.noack = true;

	err = esb_write_payload(&tx_payload);
	if (err) {
		return err;
	}

	return 0;
}

ISR_DIRECT_DECLARE(rtc0_tx_irq_handler)
{
	//uint32_t rtc0_ticks;
	int err;

	nrf_gpio_pin_set(TX_INTERRUPT_MARKER_PSEL);

	if (!nrf_rtc_event_check(NRF_RTC0, NRF_RTC_EVENT_TICK)) {
		return 0;
	}

	nrf_rtc_event_clear(NRF_RTC0, NRF_RTC_EVENT_TICK);
	//rtc0_ticks = rtc0_counter_get();

	if (!atomic_cas(&tx_ready, 1, 0)) {
		return 0;
	}

	//nrf_gpio_pin_set(TX_INTERRUPT_MARKER_PSEL);
	//err = packet_send(rtc0_ticks);
	packet_send(88);
	nrf_gpio_pin_clear(TX_INTERRUPT_MARKER_PSEL);
	if (err) {
		atomic_set(&tx_ready, 1);
	}

	return 0;
}

static void rtc0_tx_irq_initialize(void)
{
	IRQ_DIRECT_CONNECT(DT_IRQN(RTC0_NODE), RTC0_TX_IRQ_PRIORITY,
			   rtc0_tx_irq_handler, 0);
	irq_enable(DT_IRQN(RTC0_NODE));
}

int main(void)
{
	bool orange_led_on = true;
	int64_t next_led_state_change_ms;
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

	err = packet_strobe_initialize();
	if (err) {
		return 0;
	}

	atomic_set(&tx_ready, 1);
	atomic_set(&packet_counter, 0);
	err = dk_set_led(DK_LED3, orange_led_on);
	if (err) {
		LOG_ERR("Orange LED initialization failed: %d", err);
		return 0;
	}

	rtc0_tx_irq_initialize();
	rtc0_timebase_start();
	next_led_state_change_ms = k_uptime_get() + ORANGE_LED_ON_MS;

	LOG_INF("RTC0 TICK rate is %u Hz", RTC0_TICK_RATE_HZ);

	while (1) {
		int64_t now_ms = k_uptime_get();

		if (now_ms >= next_led_state_change_ms) {
			orange_led_on = !orange_led_on;
			next_led_state_change_ms += orange_led_on ? ORANGE_LED_ON_MS :
				ORANGE_LED_OFF_MS;
			err = dk_set_led(DK_LED3, orange_led_on);
			if (err) {
				LOG_ERR("Orange LED update failed: %d", err);
			}
		}

		k_msleep(10);
	}
}
