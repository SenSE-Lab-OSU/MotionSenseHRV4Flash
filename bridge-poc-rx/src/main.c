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
#include <zephyr/logging/log.h>

#include <esb.h>
#include <hal/nrf_dppi.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppib.h>
#include <hal/nrf_radio.h>
#include <nrfx_gpiote.h>

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

#define ESB_PAYLOAD_LENGTH	8U
#define RX_PULSE_PIN		14U
#define LED1_TOGGLE_PACKET_COUNT	8U
#define LED1_NODE		DT_ALIAS(led0)
#define RX_PULSE_PORT_NODE	DT_NODELABEL(gpio1)
#define RX_PULSE_RADIO_DPPI_CHANNEL	7U
#define RX_PULSE_PERI_DPPI_CHANNEL	1U
#define RX_PULSE_PPIB_CHANNEL	1U
#define RX_PULSE_GPIOTE_INST	\
	DT_PROP(DT_PHANDLE(RX_PULSE_PORT_NODE, gpiote_instance), instance)
#define RX_PULSE_PSEL		\
	NRF_GPIO_PIN_MAP(DT_PROP(RX_PULSE_PORT_NODE, port), RX_PULSE_PIN)

BUILD_ASSERT(IS_ENABLED(_CONCAT(CONFIG_, _CONCAT(NRFX_GPIOTE,
						 RX_PULSE_GPIOTE_INST))),
	     "The P1.14 GPIOTE instance must be enabled");

static const struct device *const rx_pulse_gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const nrfx_gpiote_t rx_pulse_gpiote =
	NRFX_GPIOTE_INSTANCE(RX_PULSE_GPIOTE_INST);
static struct esb_payload rx_payload;
static uint8_t received_packets_since_led1_toggle;

static void rx_pulse_end(void)
{
	/* Use GPIOTE's CLR task to release the same task-controlled output. */
	nrfx_gpiote_clr_task_trigger(&rx_pulse_gpiote, RX_PULSE_PSEL);
}

static int rx_pulse_initialize(void)
{
	if (!device_is_ready(rx_pulse_gpio)) {
		LOG_ERR("P1.14 GPIO is not ready");
		return -ENODEV;
	}

	return gpio_pin_configure(rx_pulse_gpio, RX_PULSE_PIN, GPIO_OUTPUT_INACTIVE);
}

static int rx_pulse_hardware_trigger_initialize(void)
{
	const nrfx_gpiote_output_config_t output_config =
	NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
	nrfx_gpiote_task_config_t task_config;
	uint8_t gpiote_channel;
	nrfx_err_t nrfx_err;

	if (!nrfx_gpiote_init_check(&rx_pulse_gpiote)) {
		nrfx_err = nrfx_gpiote_init(&rx_pulse_gpiote, 0);
		if (nrfx_err != NRFX_SUCCESS) {
			LOG_ERR("P1.14 GPIOTE initialization failed: 0x%08x", nrfx_err);
			return -EIO;
		}
	}

	nrfx_err = nrfx_gpiote_channel_alloc(&rx_pulse_gpiote, &gpiote_channel);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("P1.14 GPIOTE channel allocation failed: 0x%08x", nrfx_err);
		return -ENOMEM;
	}

	task_config.task_ch = gpiote_channel;
	task_config.polarity = NRF_GPIOTE_POLARITY_LOTOHI;
	task_config.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW;
	nrfx_err = nrfx_gpiote_output_configure(&rx_pulse_gpiote, RX_PULSE_PSEL,
						   &output_config, &task_config);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("P1.14 GPIOTE configuration failed: 0x%08x", nrfx_err);
		return -EIO;
	}

	nrfx_gpiote_out_task_enable(&rx_pulse_gpiote, RX_PULSE_PSEL);

	/*
	 * RADIO is in the RADIO domain, while GPIOTE20 (for GPIO1) is in the
	 * PERI domain. PPIB11 and PPIB21 form the hardware bridge between them.
	 * These channels avoid ESB's fixed RADIO DPPI channels 0 through 6 and
	 * MPSL's reserved channel 0 in each peripheral.
	 */
	nrf_radio_publish_set(NRF_RADIO, NRF_RADIO_EVENT_CRCOK,
			      RX_PULSE_RADIO_DPPI_CHANNEL);
	nrf_ppib_subscribe_set(NRF_PPIB11,
			       nrf_ppib_send_task_get(RX_PULSE_PPIB_CHANNEL),
			       RX_PULSE_RADIO_DPPI_CHANNEL);
	nrf_ppib_publish_set(NRF_PPIB21,
			     nrf_ppib_receive_event_get(RX_PULSE_PPIB_CHANNEL),
			     RX_PULSE_PERI_DPPI_CHANNEL);
	nrf_gpiote_subscribe_set(
		rx_pulse_gpiote.p_reg,
		nrfx_gpiote_set_task_get(&rx_pulse_gpiote, RX_PULSE_PSEL),
		RX_PULSE_PERI_DPPI_CHANNEL);
	nrf_dppi_channels_enable(NRF_DPPIC10, BIT(RX_PULSE_RADIO_DPPI_CHANNEL));
	nrf_dppi_channels_enable(NRF_DPPIC20, BIT(RX_PULSE_PERI_DPPI_CHANNEL));

	return 0;
}

static int led1_initialize(void)
{
	if (!gpio_is_ready_dt(&led1)) {
		LOG_ERR("LED1 GPIO is not ready");
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
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

	/* CRCOK raised P1.14 in hardware; end it when ESB delivers the packet. */
	rx_pulse_end();

	while (esb_read_rx_payload(&rx_payload) == 0) {
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

	err = rx_pulse_hardware_trigger_initialize();
	if (err) {
		LOG_ERR("P1.14 hardware marker initialization failed: %d", err);
		return 0;
	}

	err = esb_start_rx();
	if (err) {
		LOG_ERR("ESB RX start failed: %d", err);
	}

	return 0;
}
