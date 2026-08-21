/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <esb.h>

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

#define ESB_PAYLOAD_LENGTH	8U
#define RX_PULSE_PIN		14U
#define RX_PULSE_DURATION	K_MSEC(1)
#define LED1_TOGGLE_PACKET_COUNT	8U
#define LED1_NODE		DT_ALIAS(led0)

static const struct device *const rx_pulse_gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static struct esb_payload rx_payload;
static uint8_t received_packets_since_led1_toggle;

static void rx_pulse_off_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	err = gpio_pin_set(rx_pulse_gpio, RX_PULSE_PIN, 0);
	if (err) {
		LOG_ERR("P1.14 set low failed: %d", err);
	}
}

static K_WORK_DELAYABLE_DEFINE(rx_pulse_off_work, rx_pulse_off_handler);

static int rx_pulse_initialize(void)
{
	if (!device_is_ready(rx_pulse_gpio)) {
		LOG_ERR("P1.14 GPIO is not ready");
		return -ENODEV;
	}

	return gpio_pin_configure(rx_pulse_gpio, RX_PULSE_PIN, GPIO_OUTPUT_INACTIVE);
}

static int led1_initialize(void)
{
	if (!gpio_is_ready_dt(&led1)) {
		LOG_ERR("LED1 GPIO is not ready");
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
}

static void rx_pulse_start(void)
{
	int err;

	err = gpio_pin_set(rx_pulse_gpio, RX_PULSE_PIN, 1);
	if (err) {
		return;
	}

	(void)k_work_reschedule(&rx_pulse_off_work, RX_PULSE_DURATION);
}

static void led1_toggle_after_eight_packets(void)
{
	int err;

	received_packets_since_led1_toggle++;
	if (received_packets_since_led1_toggle < LED1_TOGGLE_PACKET_COUNT) {
		return;
	}

	received_packets_since_led1_toggle = 0U;
	err = gpio_pin_toggle_dt(&led1);
	if (err) {
		LOG_ERR("LED1 toggle failed: %d", err);
	}
}

static void event_handler(const struct esb_evt *event)
{
	if (event->evt_id != ESB_EVENT_RX_RECEIVED) {
		return;
	}

	while (esb_read_rx_payload(&rx_payload) == 0) {
		rx_pulse_start();
		led1_toggle_after_eight_packets();
	}
}

static int hfclk_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the HF clock manager");
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

	return 0;
}

static int esb_initialize(void)
{
	int err;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4,
				  0xC5, 0xC6, 0xC7, 0xC8};
	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.payload_length = ESB_PAYLOAD_LENGTH;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
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

int main(void)
{
	int err;

	err = rx_pulse_initialize();
	if (err) {
		LOG_ERR("P1.14 initialization failed: %d", err);
		return 0;
	}

	err = led1_initialize();
	if (err) {
		LOG_ERR("LED1 initialization failed: %d", err);
		return 0;
	}

	err = hfclk_start();
	if (err) {
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed: %d", err);
		return 0;
	}

	err = esb_start_rx();
	if (err) {
		LOG_ERR("ESB RX start failed: %d", err);
	}

	return 0;
}
