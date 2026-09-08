#include "imuFsyncTiming.h"

#include "BLEService.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <helpers/nrfx_gppi.h>
#include <nrfx.h>
#include <nrfx_gpiote.h>
#include <nrfx_rtc.h>
#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(imu_fsync_timing, CONFIG_LOG_LEVEL_ICM20948_ACCEL);

#define ICM20948_NODE DT_ALIAS(imu)
#define IMU_FSYNC_RTC_COMPARE_CHANNEL 1U
#define IMU_FSYNC_RTC_MASK 0x00FFFFFFU
#define IMU_FSYNC_EDGE_INTERVAL_TICKS 32U
#define IMU_FSYNC_EDGE_RING_SIZE 32U

static const struct gpio_dt_spec imu_fsync_gpio =
	GPIO_DT_SPEC_GET(ICM20948_NODE, fsync_gpios);
static const nrfx_rtc_t imu_fsync_rtc = NRFX_RTC_INSTANCE(0);
static const nrfx_gpiote_t imu_fsync_gpiote = NRFX_GPIOTE_INSTANCE(0);
static const nrfx_gpiote_pin_t imu_fsync_pin =
	NRF_DT_GPIOS_TO_PSEL(ICM20948_NODE, fsync_gpios);

static struct imu_fsync_edge imu_fsync_edges[IMU_FSYNC_EDGE_RING_SIZE];
static uint8_t imu_fsync_head;
static uint8_t imu_fsync_tail;
static uint8_t imu_fsync_level;
static uint8_t imu_fsync_gpiote_channel;
static uint8_t imu_fsync_gppi_channel;
static uint32_t imu_fsync_next_raw_compare;
static uint32_t imu_fsync_next_extended_tick;
static uint32_t imu_fsync_edge_ordinal;
static int imu_fsync_error;
static bool imu_fsync_initialized;
static bool imu_fsync_active;
static bool imu_fsync_gpiote_allocated;
static bool imu_fsync_gppi_allocated;

K_SEM_DEFINE(imu_fsync_edge_sem, 0, 1);

static uint8_t imu_fsync_ring_next(uint8_t index)
{
	return (uint8_t)((index + 1U) % IMU_FSYNC_EDGE_RING_SIZE);
}

static void imu_fsync_reset_state(void)
{
	unsigned int key = irq_lock();

	imu_fsync_head = 0U;
	imu_fsync_tail = 0U;
	imu_fsync_level = 0U;
	imu_fsync_next_raw_compare = 0U;
	imu_fsync_next_extended_tick = 0U;
	imu_fsync_edge_ordinal = 0U;
	imu_fsync_error = 0;
	k_sem_reset(&imu_fsync_edge_sem);
	irq_unlock(key);
}

static void imu_fsync_release_resources(void)
{
	if (imu_fsync_gppi_allocated) {
		nrfx_gppi_channels_disable(BIT(imu_fsync_gppi_channel));
		nrfx_gppi_channel_endpoints_clear(
			imu_fsync_gppi_channel,
			nrfx_rtc_event_address_get(&imu_fsync_rtc,
				NRF_RTC_EVENT_COMPARE_1),
			nrfx_gpiote_out_task_address_get(&imu_fsync_gpiote,
				imu_fsync_pin));
		(void)nrfx_gppi_channel_free(imu_fsync_gppi_channel);
		imu_fsync_gppi_allocated = false;
	}

	if (imu_fsync_gpiote_allocated) {
		nrfx_gpiote_out_task_disable(&imu_fsync_gpiote, imu_fsync_pin);
		(void)nrfx_gpiote_pin_uninit(&imu_fsync_gpiote, imu_fsync_pin);
		(void)nrfx_gpiote_channel_free(&imu_fsync_gpiote,
					      imu_fsync_gpiote_channel);
		imu_fsync_gpiote_allocated = false;
	}
}

int imu_fsync_timing_init(void)
{
	int ret;

	if (imu_fsync_initialized) {
		return 0;
	}
	if (!device_is_ready(imu_fsync_gpio.port)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&imu_fsync_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	if (!nrfx_gpiote_init_check(&imu_fsync_gpiote)) {
		nrfx_err_t err = nrfx_gpiote_init(&imu_fsync_gpiote, IRQ_PRIO_LOWEST);

		if (err != NRFX_SUCCESS) {
			return -EIO;
		}
	}

	imu_fsync_reset_state();
	imu_fsync_initialized = true;
	return 0;
}

int imu_fsync_timing_start(void)
{
	const nrfx_gpiote_output_config_t output_config = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	nrfx_gpiote_task_config_t task_config;
	nrfx_err_t err;
	uint32_t now_ticks;
	uint32_t raw_ticks;
	unsigned int key;
	int ret;

	if (!imu_fsync_initialized) {
		return -EACCES;
	}
	if (imu_fsync_active) {
		return -EALREADY;
	}

	ret = rtc0_collection_counter_get(&now_ticks);
	if (ret != 0) {
		return ret;
	}

	imu_fsync_reset_state();
	err = nrfx_gpiote_channel_alloc(&imu_fsync_gpiote,
				       &imu_fsync_gpiote_channel);
	if (err != NRFX_SUCCESS) {
		return -ENOMEM;
	}
	imu_fsync_gpiote_allocated = true;
	task_config.task_ch = imu_fsync_gpiote_channel;
	task_config.polarity = NRF_GPIOTE_POLARITY_TOGGLE;
	task_config.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW;
	err = nrfx_gpiote_output_configure(&imu_fsync_gpiote, imu_fsync_pin,
					  &output_config, &task_config);
	if (err != NRFX_SUCCESS) {
		imu_fsync_release_resources();
		return -EIO;
	}
	nrfx_gpiote_out_task_enable(&imu_fsync_gpiote, imu_fsync_pin);

	err = nrfx_gppi_channel_alloc(&imu_fsync_gppi_channel);
	if (err != NRFX_SUCCESS) {
		imu_fsync_release_resources();
		return -ENOMEM;
	}
	imu_fsync_gppi_allocated = true;
	nrfx_gppi_channel_endpoints_setup(
		imu_fsync_gppi_channel,
		nrfx_rtc_event_address_get(&imu_fsync_rtc,
			NRF_RTC_EVENT_COMPARE_1),
		nrfx_gpiote_out_task_address_get(&imu_fsync_gpiote, imu_fsync_pin));
	nrfx_gppi_channels_enable(BIT(imu_fsync_gppi_channel));

	raw_ticks = now_ticks & IMU_FSYNC_RTC_MASK;
	key = irq_lock();
	imu_fsync_next_raw_compare =
		(raw_ticks + IMU_FSYNC_EDGE_INTERVAL_TICKS) & IMU_FSYNC_RTC_MASK;
	imu_fsync_next_extended_tick = now_ticks + IMU_FSYNC_EDGE_INTERVAL_TICKS;
	err = nrfx_rtc_cc_set(&imu_fsync_rtc, IMU_FSYNC_RTC_COMPARE_CHANNEL,
			      imu_fsync_next_raw_compare, true);
	if (err == NRFX_SUCCESS) {
		imu_fsync_active = true;
	}
	irq_unlock(key);
	if (err != NRFX_SUCCESS) {
		imu_fsync_release_resources();
		return -EIO;
	}

	LOG_INF("IMU FSYNC toggling every %u RTC ticks", IMU_FSYNC_EDGE_INTERVAL_TICKS);
	return 0;
}

void imu_fsync_timing_stop(void)
{
	unsigned int key;

	if (!imu_fsync_initialized) {
		return;
	}

	key = irq_lock();
	if (imu_fsync_active) {
		imu_fsync_active = false;
		(void)nrfx_rtc_cc_disable(&imu_fsync_rtc,
					  IMU_FSYNC_RTC_COMPARE_CHANNEL);
	}
	irq_unlock(key);
	imu_fsync_release_resources();
	(void)gpio_pin_configure_dt(&imu_fsync_gpio, GPIO_OUTPUT_INACTIVE);
	imu_fsync_reset_state();
}

int imu_fsync_timing_take_edge(struct imu_fsync_edge *edge)
{
	unsigned int key;
	int ret = 0;

	if (edge == NULL) {
		return -EINVAL;
	}

	key = irq_lock();
	if (imu_fsync_error != 0) {
		ret = imu_fsync_error;
	} else if (imu_fsync_tail == imu_fsync_head) {
		ret = -ENOENT;
	} else {
		*edge = imu_fsync_edges[imu_fsync_tail];
		imu_fsync_tail = imu_fsync_ring_next(imu_fsync_tail);
	}
	irq_unlock(key);
	return ret;
}

uint32_t imu_fsync_timing_edge_count_get(void)
{
	unsigned int key = irq_lock();
	uint32_t ordinal = imu_fsync_edge_ordinal;

	irq_unlock(key);
	return ordinal;
}

int imu_fsync_timing_wait_for_edge_after(uint32_t ordinal, k_timeout_t timeout)
{
	int ret;

	while (imu_fsync_timing_edge_count_get() <= ordinal) {
		ret = k_sem_take(&imu_fsync_edge_sem, timeout);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

void imu_fsync_timing_rtc_compare_isr(void)
{
	uint8_t next_head;
	nrfx_err_t err;

	if (!imu_fsync_active) {
		return;
	}

	imu_fsync_level ^= 1U;
	imu_fsync_edge_ordinal++;
	next_head = imu_fsync_ring_next(imu_fsync_head);
	if (next_head == imu_fsync_tail) {
		imu_fsync_error = -ENOMEM;
	} else {
		imu_fsync_edges[imu_fsync_head] = (struct imu_fsync_edge){
			.rtc_ticks = imu_fsync_next_extended_tick,
			.ordinal = imu_fsync_edge_ordinal,
			.level = imu_fsync_level,
		};
		imu_fsync_head = next_head;
	}

	imu_fsync_next_raw_compare =
		(imu_fsync_next_raw_compare + IMU_FSYNC_EDGE_INTERVAL_TICKS) &
		IMU_FSYNC_RTC_MASK;
	imu_fsync_next_extended_tick += IMU_FSYNC_EDGE_INTERVAL_TICKS;
	err = nrfx_rtc_cc_set(&imu_fsync_rtc, IMU_FSYNC_RTC_COMPARE_CHANNEL,
			      imu_fsync_next_raw_compare, true);
	if (err != NRFX_SUCCESS) {
		imu_fsync_error = -EIO;
	}

	k_sem_give(&imu_fsync_edge_sem);
}
