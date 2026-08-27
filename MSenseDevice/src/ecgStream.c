#include "ecgStream.h"

#include "drivers/ecg/max30001.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ecg_stream, CONFIG_LOG_LEVEL_MAX30001);

#define ECG_STREAM_UART_NODE DT_NODELABEL(cdc_acm_uart1)
#define ECG_STREAM_MAX30001_NODE DT_NODELABEL(max30001)

#define ECG_STREAM_FRAME_SIZE 12
#define ECG_STREAM_SYNC0 0xA5u
#define ECG_STREAM_SYNC1 0xECu
#define ECG_STREAM_TYPE_SAMPLE 0x01u
#define ECG_STREAM_CRC_POLY 0x07u

#define ECG_STREAM_TX_RING_SIZE 4096
#define ECG_STREAM_UART_TX_CHUNK_SIZE 64
#define ECG_STREAM_THREAD_STACK_SIZE 4096
#define ECG_STREAM_THREAD_PRIORITY 5
#define ECG_STREAM_DTR_POLL_MS 100
#define ECG_STREAM_DROP_LOG_INTERVAL 512u

static const struct device *const ecg_uart =
	DEVICE_DT_GET(ECG_STREAM_UART_NODE);
static const struct gpio_dt_spec ecg_intb =
	GPIO_DT_SPEC_GET(ECG_STREAM_MAX30001_NODE, intb_gpios);

RING_BUF_DECLARE(ecg_tx_ring, ECG_STREAM_TX_RING_SIZE);

static K_SEM_DEFINE(ecg_stream_start_sem, 0, 1);
static K_SEM_DEFINE(ecg_fifo_sem, 0, 1);

static struct gpio_callback ecg_intb_callback;
static atomic_t ecg_stream_requested;
static bool ecg_intb_callback_added;
static uint32_t ecg_sequence;
static uint32_t ecg_dropped_frames;

static void ecg_stream_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(ecg_stream_thread_id,
		ECG_STREAM_THREAD_STACK_SIZE,
		ecg_stream_thread,
		NULL,
		NULL,
		NULL,
		ECG_STREAM_THREAD_PRIORITY,
		0,
		0);

static uint8_t ecg_stream_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			if ((crc & 0x80u) != 0u) {
				crc = (uint8_t)((crc << 1) ^ ECG_STREAM_CRC_POLY);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static void ecg_stream_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_tx_ready(dev)) {
		uint8_t *data;
		uint32_t len;
		int sent;

		len = ring_buf_get_claim(&ecg_tx_ring, &data,
					 ECG_STREAM_UART_TX_CHUNK_SIZE);
		if (len == 0u) {
			uart_irq_tx_disable(dev);
			return;
		}

		sent = uart_fifo_fill(dev, data, (int)len);
		if (sent < 0) {
			(void)ring_buf_get_finish(&ecg_tx_ring, 0);
			uart_irq_tx_disable(dev);
			return;
		}

		(void)ring_buf_get_finish(&ecg_tx_ring, (uint32_t)sent);

		if ((uint32_t)sent < len) {
			return;
		}
	}
}

static int ecg_stream_enqueue(const uint8_t *data, size_t len)
{
	unsigned int key;
	uint32_t size = (uint32_t)len;
	uint32_t written;

	key = irq_lock();
	if (ring_buf_space_get(&ecg_tx_ring) < size) {
		irq_unlock(key);
		return -ENOMEM;
	}

	written = ring_buf_put(&ecg_tx_ring, data, size);
	if (written == size) {
		uart_irq_tx_enable(ecg_uart);
	}
	irq_unlock(key);

	return written == size ? 0 : -ENOMEM;
}

static bool ecg_stream_dtr_is_set(void)
{
	uint32_t dtr = 0;

	return uart_line_ctrl_get(ecg_uart, UART_LINE_CTRL_DTR, &dtr) == 0 &&
	       dtr != 0u;
}

static int ecg_stream_wait_for_dtr(void)
{
	while (atomic_get(&ecg_stream_requested) != 0) {
		if (ecg_stream_dtr_is_set()) {
			(void)uart_line_ctrl_set(ecg_uart, UART_LINE_CTRL_DCD, 1);
			(void)uart_line_ctrl_set(ecg_uart, UART_LINE_CTRL_DSR, 1);
			return 0;
		}

		k_sleep(K_MSEC(ECG_STREAM_DTR_POLL_MS));
	}

	return -ECANCELED;
}

static void ecg_stream_intb_handler(const struct device *port,
				    struct gpio_callback *cb,
				    uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_sem_give(&ecg_fifo_sem);
}

static int ecg_stream_configure_intb(void)
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
				   ecg_stream_intb_handler,
				   BIT(ecg_intb.pin));

		ret = gpio_add_callback(ecg_intb.port, &ecg_intb_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

static int ecg_stream_configure_uart(void)
{
	int ret;

	if (!device_is_ready(ecg_uart)) {
		LOG_ERR("ECG CDC ACM UART is not ready");
		return -ENODEV;
	}

	ret = uart_irq_callback_user_data_set(ecg_uart,
					      ecg_stream_uart_isr,
					      NULL);
	if (ret != 0) {
		LOG_ERR("Failed to set ECG UART callback: %d", ret);
		return ret;
	}

	uart_irq_rx_disable(ecg_uart);
	uart_irq_tx_disable(ecg_uart);

	return 0;
}

static int ecg_stream_send_sample(const struct max30001_ecg_sample *sample)
{
	uint8_t frame[ECG_STREAM_FRAME_SIZE];
	int ret;

	frame[0] = ECG_STREAM_SYNC0;
	frame[1] = ECG_STREAM_SYNC1;
	frame[2] = ECG_STREAM_TYPE_SAMPLE;
	frame[3] = (uint8_t)((sample->etag & 0x07u) |
			     ((sample->ptag & 0x07u) << 3));
	frame[4] = (uint8_t)(ecg_sequence & 0xFFu);
	frame[5] = (uint8_t)((ecg_sequence >> 8) & 0xFFu);
	frame[6] = (uint8_t)((ecg_sequence >> 16) & 0xFFu);
	frame[7] = (uint8_t)((ecg_sequence >> 24) & 0xFFu);
	frame[8] = (uint8_t)((sample->raw >> 16) & 0xFFu);
	frame[9] = (uint8_t)((sample->raw >> 8) & 0xFFu);
	frame[10] = (uint8_t)(sample->raw & 0xFFu);
	frame[11] = ecg_stream_crc8(&frame[2], 9);

	ret = ecg_stream_enqueue(frame, sizeof(frame));
	if (ret != 0) {
		ecg_dropped_frames++;
		if ((ecg_dropped_frames % ECG_STREAM_DROP_LOG_INTERVAL) == 1u) {
			LOG_WRN("Dropped ECG stream frames: %u",
				(unsigned int)ecg_dropped_frames);
		}
	}

	return ret;
}

static void ecg_stream_process_samples(const struct max30001_ecg_sample *samples,
				       size_t count)
{
	bool host_connected = ecg_stream_dtr_is_set();

	for (size_t i = 0; i < count; i++) {
		if (!samples[i].time_valid) {
			continue;
		}

		if (host_connected) {
			(void)ecg_stream_send_sample(&samples[i]);
		} else {
			ecg_dropped_frames++;
		}

		ecg_sequence++;
	}
}

static void ecg_stream_drain_fifo(void)
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

		ecg_stream_process_samples(samples, count);

		if (count < ARRAY_SIZE(samples) || samples[count - 1].eof) {
			return;
		}
	}
}

static int ecg_stream_run(void)
{
	int ret;

	ret = ecg_stream_configure_uart();
	if (ret != 0) {
		return ret;
	}

	ret = ecg_stream_configure_intb();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INTB: %d", ret);
		return ret;
	}

	LOG_INF("Waiting for ECG raw CDC ACM DTR");
	ret = ecg_stream_wait_for_dtr();
	if (ret != 0) {
		return ret;
	}

	ring_buf_reset(&ecg_tx_ring);
	ecg_sequence = 0;
	ecg_dropped_frames = 0;
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
	LOG_INF("ECG raw stream active on cdc_acm_uart1");

	while (atomic_get(&ecg_stream_requested) != 0) {
		ret = k_sem_take(&ecg_fifo_sem, K_SECONDS(1));
		if (ret == 0) {
			ecg_stream_drain_fifo();
		}
	}

	(void)gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
	(void)max30001_ecg_stop();
	uart_irq_tx_disable(ecg_uart);

	return 0;
}

static void ecg_stream_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&ecg_stream_start_sem, K_FOREVER);

		while (atomic_get(&ecg_stream_requested) != 0) {
			int ret = ecg_stream_run();

			if (atomic_get(&ecg_stream_requested) == 0) {
				break;
			}

			LOG_ERR("ECG stream stopped after error: %d", ret);
			k_sleep(K_SECONDS(1));
		}
	}
}

int ecg_stream_start(void)
{
	if (atomic_cas(&ecg_stream_requested, 0, 1)) {
		k_sem_give(&ecg_stream_start_sem);
	}

	return 0;
}

int ecg_stream_stop(void)
{
	atomic_clear(&ecg_stream_requested);
	k_sem_give(&ecg_fifo_sem);

	return 0;
}
