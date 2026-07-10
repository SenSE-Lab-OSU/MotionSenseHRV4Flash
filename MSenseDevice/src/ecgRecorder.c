#include "ecgRecorder.h"

#include "drivers/ecg/max30001.h"
#include "zephyrfilesystem.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include "BLEService.h"

LOG_MODULE_REGISTER(ecg_recorder, CONFIG_LOG_LEVEL_MAX30001);

#define ECG_RECORD_MAX30001_NODE DT_NODELABEL(max30001)

#define ECG_RECORD_FRAME_SIZE 12
#define ECG_RECORD_SYNC0 0xA5u
#define ECG_RECORD_SYNC1 0xECu
#define ECG_RECORD_TYPE_SAMPLE 0x01u
#define ECG_RECORD_CRC_POLY 0x07u

#define ECG_RECORD_THREAD_STACK_SIZE 4096
#define ECG_RECORD_THREAD_PRIORITY 5

static const struct gpio_dt_spec ecg_intb =
	GPIO_DT_SPEC_GET(ECG_RECORD_MAX30001_NODE, intb_gpios);

static K_SEM_DEFINE(ecg_record_start_sem, 0, 1);
static K_SEM_DEFINE(ecg_fifo_sem, 0, 1);
static K_SEM_DEFINE(ecg_record_stopped_sem, 0, 1);

static struct gpio_callback ecg_intb_callback;
static atomic_t ecg_record_requested;
static atomic_t ecg_record_active;
static bool ecg_intb_callback_added;
static uint32_t ecg_sequence;
uint32_t ecg_total_samples = 0;
uint32_t ecg_last_update_samples = 0;

float random_bar;
uint8_t ecg_packet[ENMO_DATA_LEN] = {0};
#define ECG_COUNTER (sizeof(random_bar))


static void ecg_record_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(ecg_record_thread_id,
		ECG_RECORD_THREAD_STACK_SIZE,
		ecg_record_thread,
		NULL,
		NULL,
		NULL,
		ECG_RECORD_THREAD_PRIORITY,
		0,
		0);

static uint8_t ecg_record_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			if ((crc & 0x80u) != 0u) {
				crc = (uint8_t)((crc << 1) ^ ECG_RECORD_CRC_POLY);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static void ecg_record_intb_handler(const struct device *port,
				    struct gpio_callback *cb,
				    uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_sem_give(&ecg_fifo_sem);
}

static int ecg_record_configure_intb(void)
{
	int ret;

	if (!gpio_is_ready_dt(&ecg_intb)) {
		LOG_ERR("MAX30001 INTB GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&ecg_intb, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	if (!ecg_intb_callback_added) {
		gpio_init_callback(&ecg_intb_callback,
				   ecg_record_intb_handler,
				   BIT(ecg_intb.pin));

		ret = gpio_add_callback(ecg_intb.port, &ecg_intb_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

static void ecg_record_store_sample(const struct max30001_ecg_sample *sample)
{
	uint8_t frame[ECG_RECORD_FRAME_SIZE];

	frame[0] = ECG_RECORD_SYNC0;
	frame[1] = ECG_RECORD_SYNC1;
	frame[2] = ECG_RECORD_TYPE_SAMPLE;
	frame[3] = (uint8_t)((sample->etag & 0x07u) |
			     ((sample->ptag & 0x07u) << 3));
	frame[4] = (uint8_t)(ecg_sequence & 0xFFu);
	frame[5] = (uint8_t)((ecg_sequence >> 8) & 0xFFu);
	frame[6] = (uint8_t)((ecg_sequence >> 16) & 0xFFu);
	frame[7] = (uint8_t)((ecg_sequence >> 24) & 0xFFu);
	frame[8] = (uint8_t)((sample->raw >> 16) & 0xFFu);
	frame[9] = (uint8_t)((sample->raw >> 8) & 0xFFu);
	frame[10] = (uint8_t)(sample->raw & 0xFFu);
	frame[11] = ecg_record_crc8(&frame[2], 9);

	store_data(frame, sizeof(frame), ecg);
}

static void ecg_record_process_samples(const struct max30001_ecg_sample *samples,
				       size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (!samples[i].time_valid) {
			continue;
		}

		ecg_record_store_sample(&samples[i]);
		ecg_sequence++;
	}
}



static void ecg_record_drain_fifo(void)
{
	int ret;

	for (int pass = 0; pass < 4; pass++) {
		struct max30001_ecg_sample samples[MAX30001_ECG_FIFO_MAX_SAMPLES];
		size_t count = 0;

		ret = max30001_ecg_read_fifo(samples, ARRAY_SIZE(samples), &count);
		if (ret != 0) {
			LOG_ERR("MAX30001 ECG FIFO read failed: %d", ret);
			return;
		}

		if (count == 0) {
			return;
		}
		
		
		
		ecg_record_process_samples(samples, count);

		ecg_total_samples += count;
		ecg_last_update_samples += count;
		if (ecg_last_update_samples >= 1024){
			LOG_INF("count: %d", ecg_total_samples);
			
			memcpy(&ecg_packet[4], &ecg_total_samples, sizeof(ecg_total_samples));
			my_motionData.dataPacket = ecg_packet;
      		my_motionData.packetLength = sizeof(ecg_packet);
			ecg_last_update_samples = 0;
      		k_work_submit(&my_motionData.work);
		}

		if (count < ARRAY_SIZE(samples) || samples[count - 1].eof) {
			return;
		}
	}
}

static int ecg_record_run(void)
{
	int ret;

	if (!file_system_ready) {
		LOG_ERR("Filesystem is not ready for ECG recording");
		return -ENODEV;
	}

	ret = ecg_record_configure_intb();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INTB: %d", ret);
		return ret;
	}

	ecg_sequence = 0;
	k_sem_reset(&ecg_fifo_sem);

	ret = max30001_ecg_init_512();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG init failed: %d", ret);
		(void)max30001_ecg_stop();
		return ret;
	}

	ret = max30001_ecg_start();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG start failed: %d", ret);
		(void)max30001_ecg_stop();
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable MAX30001 INTB interrupt: %d", ret);
		(void)max30001_ecg_stop();
		return ret;
	}

	k_sem_give(&ecg_fifo_sem);
	LOG_INF("ECG NAND recording active");

	while (atomic_get(&ecg_record_requested) != 0) {
		ret = k_sem_take(&ecg_fifo_sem, K_SECONDS(1));
		if (ret == 0) {
			ecg_record_drain_fifo();
		}
	}

	(void)gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
	ecg_record_drain_fifo();
	(void)max30001_ecg_stop();

	return 0;
}

static void ecg_record_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&ecg_record_start_sem, K_FOREVER);

		while (atomic_get(&ecg_record_requested) != 0) {
			int ret;

			atomic_set(&ecg_record_active, 1);
			ret = ecg_record_run();
			atomic_clear(&ecg_record_active);
			k_sem_give(&ecg_record_stopped_sem);

			if (atomic_get(&ecg_record_requested) == 0) {
				break;
			}

			LOG_ERR("ECG recording stopped after error: %d", ret);
			k_sleep(K_SECONDS(1));
		}
	}
}

int ecg_recorder_start(void)
{
	if (!file_system_ready) {
		return -ENODEV;
	}

	k_sem_reset(&ecg_record_stopped_sem);

	if (atomic_cas(&ecg_record_requested, 0, 1)) {
		k_sem_give(&ecg_record_start_sem);
	}

	return 0;
}

int ecg_recorder_stop(void)
{
	atomic_clear(&ecg_record_requested);
	k_sem_give(&ecg_fifo_sem);

	if (atomic_get(&ecg_record_active) == 0) {
		return 0;
	}

	return k_sem_take(&ecg_record_stopped_sem, K_SECONDS(3));
}
