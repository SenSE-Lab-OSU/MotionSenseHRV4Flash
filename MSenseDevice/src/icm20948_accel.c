#include "icm20948_accel.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(icm20948_accel, CONFIG_LOG_LEVEL_ICM20948_ACCEL);

#define ICM20948_NODE DT_NODELABEL(icm20948)

#define ICM20948_REG_WHO_AM_I 0x00U
#define ICM20948_REG_USER_CTRL 0x03U
#define ICM20948_REG_LP_CONFIG 0x05U
#define ICM20948_REG_PWR_MGMT_1 0x06U
#define ICM20948_REG_PWR_MGMT_2 0x07U
#define ICM20948_REG_INT_PIN_CFG 0x0FU
#define ICM20948_REG_INT_ENABLE 0x10U
#define ICM20948_REG_INT_ENABLE_1 0x11U
#define ICM20948_REG_INT_ENABLE_2 0x12U
#define ICM20948_REG_INT_ENABLE_3 0x13U
#define ICM20948_REG_INT_STATUS_1 0x1AU
#define ICM20948_REG_INT_STATUS_2 0x1BU
#define ICM20948_REG_FIFO_EN_1 0x66U
#define ICM20948_REG_FIFO_EN_2 0x67U
#define ICM20948_REG_FIFO_RST 0x68U
#define ICM20948_REG_FIFO_MODE 0x69U
#define ICM20948_REG_FIFO_COUNTH 0x70U
#define ICM20948_REG_FIFO_R_W 0x72U
#define ICM20948_REG_REG_BANK_SEL 0x7FU

#define ICM20948_BANK_0 0x00U
#define ICM20948_BANK_2 0x20U

#define ICM20948_WHO_AM_I_VALUE 0xEAU

#define ICM20948_USER_CTRL_FIFO_EN BIT(6)
#define ICM20948_USER_CTRL_I2C_IF_DIS BIT(4)
#define ICM20948_PWR_MGMT_1_DEVICE_RESET BIT(7)
#define ICM20948_PWR_MGMT_1_SLEEP BIT(6)
#define ICM20948_PWR_MGMT_1_TEMP_DIS BIT(3)
#define ICM20948_PWR_MGMT_1_CLKSEL_AUTO 0x01U
#define ICM20948_PWR_MGMT_2_DISABLE_GYRO 0x07U
#define ICM20948_INT_PIN_CFG_INT1_LATCH_EN BIT(5)
#define ICM20948_INT_ENABLE_1_RAW_DATA_RDY BIT(0)
#define ICM20948_ACCEL_CONFIG_DLPF_ENABLE BIT(0)
#define ICM20948_FSYNC_CONFIG_ACCEL_XOUT_L 0x05U
#define ICM20948_FIFO_EN_2_ACCEL BIT(4)
#define ICM20948_FIFO_RST_ALL 0x1FU

#define ICM20948_ACCEL_DIVIDER 1U
#define ICM20948_ACCEL_LOG_INTERVAL_SAMPLES 563U
#define ICM20948_FIFO_SAMPLE_BYTES 6U
#define ICM20948_FIFO_DRAIN_CHUNK_SAMPLES 128U
#define ICM20948_FIFO_DRAIN_CHUNK_BYTES \
	(ICM20948_FIFO_DRAIN_CHUNK_SAMPLES * ICM20948_FIFO_SAMPLE_BYTES)
#define ICM20948_FIFO_READ_BUFFER_BYTES (ICM20948_FIFO_DRAIN_CHUNK_BYTES + 1U)
#define ICM20948_FIFO_CAPACITY_BYTES 4096U
#define ICM20948_FIFO_BATCH_DELAY_MS 200U
#define ICM20948_ACCEL_THREAD_STACK_SIZE 1536
#define ICM20948_ACCEL_THREAD_PRIORITY 4

#define ICM20948_SPI_OPERATION \
	(SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)

static const struct spi_dt_spec icm20948_spi =
	SPI_DT_SPEC_GET(ICM20948_NODE, ICM20948_SPI_OPERATION, 0U);
static const struct gpio_dt_spec icm20948_int =
	GPIO_DT_SPEC_GET(ICM20948_NODE, int_gpios);
static const struct gpio_dt_spec icm20948_fsync =
	GPIO_DT_SPEC_GET(ICM20948_NODE, fsync_gpios);

static struct gpio_callback icm20948_int_callback;
static struct k_mutex icm20948_io_lock;
static struct k_mutex icm20948_sample_lock;
static struct icm20948_accel_sample icm20948_latest_sample;
static uint8_t icm20948_spi_tx[ICM20948_FIFO_READ_BUFFER_BYTES];
static uint8_t icm20948_spi_rx[ICM20948_FIFO_READ_BUFFER_BYTES];
static uint8_t icm20948_fifo_data[ICM20948_FIFO_DRAIN_CHUNK_BYTES];
static atomic_t icm20948_initialized;
static atomic_t icm20948_streaming;
static atomic_t icm20948_stream_generation;
static atomic_t icm20948_consumer_failed;
static bool icm20948_sample_valid;
static uint32_t icm20948_sample_errors;
static uint32_t icm20948_fifo_overflows;
static icm20948_accel_fifo_consumer_t icm20948_fifo_consumer;
static void *icm20948_fifo_consumer_context;

K_SEM_DEFINE(icm20948_data_ready_sem, 0, 1);

static int icm20948_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t tx_data[] = {reg & 0x7FU, value};
	const struct spi_buf tx_buf = {
		.buf = tx_data,
		.len = sizeof(tx_data),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1U,
	};

	return spi_write_dt(&icm20948_spi, &tx);
}

static int icm20948_read_regs(uint8_t reg, uint8_t *data, size_t data_len)
{
	struct spi_buf tx_buf;
	struct spi_buf rx_buf;
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1U,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1U,
	};
	int ret;

	if ((data_len == 0U) || (data_len > (sizeof(icm20948_spi_tx) - 1U))) {
		return -EINVAL;
	}

	memset(icm20948_spi_tx, 0, data_len + 1U);
	icm20948_spi_tx[0] = reg | BIT(7);
	tx_buf.buf = icm20948_spi_tx;
	tx_buf.len = data_len + 1U;
	rx_buf.buf = icm20948_spi_rx;
	rx_buf.len = data_len + 1U;

	ret = spi_transceive_dt(&icm20948_spi, &tx, &rx);
	if (ret == 0) {
		for (size_t i = 0U; i < data_len; i++) {
			data[i] = icm20948_spi_rx[i + 1U];
		}
	}

	return ret;
}

static int icm20948_read_reg(uint8_t reg, uint8_t *value)
{
	return icm20948_read_regs(reg, value, 1U);
}

static int icm20948_select_bank(uint8_t bank)
{
	return icm20948_write_reg(ICM20948_REG_REG_BANK_SEL, bank);
}

static int icm20948_reset_fifo(void)
{
	int ret;

	ret = icm20948_write_reg(ICM20948_REG_FIFO_RST, ICM20948_FIFO_RST_ALL);
	if (ret != 0) {
		return ret;
	}

	return icm20948_write_reg(ICM20948_REG_FIFO_RST, 0U);
}

static int icm20948_configure_fifo(void)
{
	int ret;

	ret = icm20948_write_reg(ICM20948_REG_FIFO_EN_1, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_FIFO_EN_2, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_FIFO_MODE, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_reset_fifo();
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_USER_CTRL,
				 ICM20948_USER_CTRL_FIFO_EN |
				 ICM20948_USER_CTRL_I2C_IF_DIS);
	if (ret != 0) {
		return ret;
	}

	return icm20948_write_reg(ICM20948_REG_FIFO_EN_2, ICM20948_FIFO_EN_2_ACCEL);
}

static int icm20948_configure_accelerometer(void)
{
	uint8_t who_am_i;
	int ret;

	ret = icm20948_write_reg(ICM20948_REG_PWR_MGMT_1,
				 ICM20948_PWR_MGMT_1_DEVICE_RESET);
	if (ret != 0) {
		return ret;
	}

	k_msleep(100);

	ret = icm20948_select_bank(ICM20948_BANK_0);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_read_reg(ICM20948_REG_WHO_AM_I, &who_am_i);
	if (ret != 0) {
		return ret;
	}

	if (who_am_i != ICM20948_WHO_AM_I_VALUE) {
		LOG_ERR("Unexpected ICM-20948 WHO_AM_I: 0x%02x", who_am_i);
		return -ENODEV;
	}

	ret = icm20948_write_reg(ICM20948_REG_USER_CTRL,
				 ICM20948_USER_CTRL_I2C_IF_DIS);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_PWR_MGMT_1,
				 ICM20948_PWR_MGMT_1_TEMP_DIS |
				 ICM20948_PWR_MGMT_1_CLKSEL_AUTO);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_PWR_MGMT_2,
				 ICM20948_PWR_MGMT_2_DISABLE_GYRO);
	if (ret != 0) {
		return ret;
	}

	k_busy_wait(25);

	ret = icm20948_write_reg(ICM20948_REG_LP_CONFIG, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_INT_PIN_CFG,
				 ICM20948_INT_PIN_CFG_INT1_LATCH_EN);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_INT_ENABLE, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_INT_ENABLE_1, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_INT_ENABLE_2, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(ICM20948_REG_INT_ENABLE_3, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_select_bank(ICM20948_BANK_2);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(0x14U, ICM20948_ACCEL_CONFIG_DLPF_ENABLE);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(0x10U, 0U);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_write_reg(0x11U, ICM20948_ACCEL_DIVIDER);
	if (ret != 0) {
		return ret;
	}

	/* Sample the external hardware-timed FSYNC level into ACCEL_XOUT_L[0]. */
	ret = icm20948_write_reg(0x52U, ICM20948_FSYNC_CONFIG_ACCEL_XOUT_L);
	if (ret != 0) {
		return ret;
	}
	ret = icm20948_read_reg(0x52U, &who_am_i);
	if ((ret != 0) || (who_am_i != ICM20948_FSYNC_CONFIG_ACCEL_XOUT_L)) {
		return (ret != 0) ? ret : -EIO;
	}

	ret = icm20948_select_bank(ICM20948_BANK_0);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_configure_fifo();
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_read_reg(ICM20948_REG_INT_STATUS_1, &who_am_i);
	if (ret != 0) {
		return ret;
	}

	ret = icm20948_read_reg(ICM20948_REG_INT_STATUS_2, &who_am_i);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static void icm20948_store_sample(const uint8_t *data)
{
	struct icm20948_accel_sample sample;

	sample.x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
	sample.y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
	sample.z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
	sample.timestamp_ms = k_uptime_get();

	k_mutex_lock(&icm20948_sample_lock, K_FOREVER);
	sample.sequence = icm20948_latest_sample.sequence + 1U;
	icm20948_latest_sample = sample;
	icm20948_sample_valid = true;
	k_mutex_unlock(&icm20948_sample_lock);

	if ((sample.sequence % ICM20948_ACCEL_LOG_INTERVAL_SAMPLES) == 0U) {
		LOG_INF("ICM-20948 accel sample %u: x=%d y=%d z=%d", sample.sequence,
			sample.x, sample.y, sample.z);
	}
}

static void icm20948_record_first_error(int *result, int error)
{
	if ((*result == 0) && (error != 0)) {
		*result = error;
	}
}

static int icm20948_read_fifo_count(uint16_t *fifo_bytes)
{
	uint8_t count[2];
	int ret;

	ret = icm20948_read_regs(ICM20948_REG_FIFO_COUNTH, count, sizeof(count));
	if (ret == 0) {
		*fifo_bytes = ((uint16_t)(count[0] & 0x1FU) << 8) | count[1];
	}

	return ret;
}

static int icm20948_drain_fifo(void)
{
	uint8_t overflow_status;
	uint16_t fifo_bytes;
	int ret;

	if (atomic_get(&icm20948_consumer_failed) != 0) {
		return -EIO;
	}

	ret = icm20948_read_reg(ICM20948_REG_INT_STATUS_2, &overflow_status);
	if (ret != 0) {
		return ret;
	}

	if (overflow_status != 0U) {
		icm20948_fifo_overflows++;
		LOG_WRN("ICM-20948 FIFO overflow %u; resetting FIFO",
			icm20948_fifo_overflows);
		ret = icm20948_reset_fifo();
		return (ret == 0) ? -EOVERFLOW : ret;
	}

	ret = icm20948_read_fifo_count(&fifo_bytes);
	if (ret != 0) {
		return ret;
	}

	if (fifo_bytes == 0U) {
		return 0;
	}

	if ((fifo_bytes > ICM20948_FIFO_CAPACITY_BYTES) ||
	    ((fifo_bytes % ICM20948_FIFO_SAMPLE_BYTES) != 0U)) {
		LOG_WRN("Invalid ICM-20948 FIFO count: %u", fifo_bytes);
		ret = icm20948_reset_fifo();
		return (ret == 0) ? -EIO : ret;
	}

	while (fifo_bytes != 0U) {
		size_t bytes_to_read = MIN((size_t)fifo_bytes,
					   (size_t)ICM20948_FIFO_DRAIN_CHUNK_BYTES);

		ret = icm20948_read_regs(ICM20948_REG_FIFO_R_W, icm20948_fifo_data,
					 bytes_to_read);
		if (ret != 0) {
			return ret;
		}

		if (icm20948_fifo_consumer != NULL) {
			ret = icm20948_fifo_consumer(icm20948_fifo_data,
						     bytes_to_read,
						     icm20948_fifo_consumer_context);
			if (ret != 0) {
				atomic_set(&icm20948_consumer_failed, 1);
				return ret;
			}
		}

		for (size_t offset = 0U; offset < bytes_to_read;
		     offset += ICM20948_FIFO_SAMPLE_BYTES) {
			icm20948_store_sample(&icm20948_fifo_data[offset]);
		}

		fifo_bytes -= bytes_to_read;
	}

	return 0;
}

static void icm20948_log_read_error(int ret)
{
	icm20948_sample_errors++;
	if ((icm20948_sample_errors % ICM20948_ACCEL_LOG_INTERVAL_SAMPLES) == 1U) {
		LOG_WRN("ICM-20948 accelerometer FIFO read failed: %d", ret);
	}
}

static void icm20948_int_handler(const struct device *port,
				 struct gpio_callback *callback, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);

	k_sem_give(&icm20948_data_ready_sem);
}

static void icm20948_sample_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		atomic_val_t stream_generation;
		uint8_t status;
		int ret;

		k_sem_take(&icm20948_data_ready_sem, K_FOREVER);

		if (atomic_get(&icm20948_streaming) == 0) {
			continue;
		}

		stream_generation = atomic_get(&icm20948_stream_generation);
		k_msleep(ICM20948_FIFO_BATCH_DELAY_MS);

		if ((atomic_get(&icm20948_streaming) == 0) ||
		    (atomic_get(&icm20948_stream_generation) != stream_generation)) {
			continue;
		}

		k_mutex_lock(&icm20948_io_lock, K_FOREVER);
		if ((atomic_get(&icm20948_streaming) != 0) &&
		    (atomic_get(&icm20948_stream_generation) == stream_generation)) {
			ret = icm20948_drain_fifo();
			if ((ret != 0) && (ret != -EOVERFLOW)) {
				icm20948_log_read_error(ret);
			}

			ret = icm20948_read_reg(ICM20948_REG_INT_STATUS_1, &status);
			if (ret != 0) {
				icm20948_log_read_error(ret);
			}
		}
		k_mutex_unlock(&icm20948_io_lock);
	}
}

K_THREAD_DEFINE(icm20948_sample_thread_id, ICM20948_ACCEL_THREAD_STACK_SIZE,
		icm20948_sample_thread, NULL, NULL, NULL,
		ICM20948_ACCEL_THREAD_PRIORITY, 0, 0);

int icm20948_accel_init(void)
{
	int ret;

	if (atomic_get(&icm20948_initialized) != 0) {
		return 0;
	}

	if (!spi_is_ready_dt(&icm20948_spi) || !device_is_ready(icm20948_int.port) ||
	    !device_is_ready(icm20948_fsync.port)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&icm20948_int, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	gpio_init_callback(&icm20948_int_callback, icm20948_int_handler,
			   BIT(icm20948_int.pin));
	ret = gpio_add_callback(icm20948_int.port, &icm20948_int_callback);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&icm20948_int, GPIO_INT_DISABLE);
	if (ret != 0) {
		return ret;
	}

	atomic_set(&icm20948_initialized, 1);
	return 0;
}

int icm20948_accel_set_fifo_consumer(icm20948_accel_fifo_consumer_t consumer,
					     void *context)
{
	if (atomic_get(&icm20948_streaming) != 0) {
		return -EBUSY;
	}

	k_mutex_lock(&icm20948_io_lock, K_FOREVER);
	if (atomic_get(&icm20948_streaming) != 0) {
		k_mutex_unlock(&icm20948_io_lock);
		return -EBUSY;
	}

	icm20948_fifo_consumer = consumer;
	icm20948_fifo_consumer_context = context;
	atomic_clear(&icm20948_consumer_failed);
	k_mutex_unlock(&icm20948_io_lock);
	return 0;
}

int icm20948_accel_start(void)
{
	int ret;

	if (atomic_get(&icm20948_initialized) == 0) {
		return -ENODEV;
	}

	if (atomic_get(&icm20948_streaming) != 0) {
		return 0;
	}

	k_mutex_lock(&icm20948_io_lock, K_FOREVER);
	if (atomic_get(&icm20948_streaming) != 0) {
		k_mutex_unlock(&icm20948_io_lock);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&icm20948_int, GPIO_INT_DISABLE);
	if (ret == 0) {
		k_sem_reset(&icm20948_data_ready_sem);
		k_mutex_lock(&icm20948_sample_lock, K_FOREVER);
		icm20948_sample_valid = false;
		k_mutex_unlock(&icm20948_sample_lock);
		ret = icm20948_configure_accelerometer();
	}
	if (ret == 0) {
		atomic_clear(&icm20948_consumer_failed);
		atomic_inc(&icm20948_stream_generation);
		atomic_set(&icm20948_streaming, 1);
		ret = gpio_pin_interrupt_configure_dt(&icm20948_int,
						      GPIO_INT_EDGE_TO_ACTIVE);
	}
	if (ret == 0) {
		ret = icm20948_write_reg(ICM20948_REG_INT_ENABLE_1,
					 ICM20948_INT_ENABLE_1_RAW_DATA_RDY);
	}
	if (ret != 0) {
		atomic_clear(&icm20948_streaming);
		atomic_inc(&icm20948_stream_generation);
		(void)gpio_pin_interrupt_configure_dt(&icm20948_int, GPIO_INT_DISABLE);
		(void)icm20948_write_reg(ICM20948_REG_INT_ENABLE_1, 0U);
		(void)icm20948_write_reg(ICM20948_REG_FIFO_EN_2, 0U);
		(void)icm20948_reset_fifo();
		(void)icm20948_write_reg(ICM20948_REG_USER_CTRL,
					 ICM20948_USER_CTRL_I2C_IF_DIS);
		(void)icm20948_write_reg(ICM20948_REG_PWR_MGMT_1,
					 ICM20948_PWR_MGMT_1_SLEEP |
					 ICM20948_PWR_MGMT_1_TEMP_DIS |
					 ICM20948_PWR_MGMT_1_CLKSEL_AUTO);
	}
	k_mutex_unlock(&icm20948_io_lock);

	if (ret == 0) {
		LOG_INF("ICM-20948 accel FIFO streaming at 562.5 Hz, +/-2 g");
	}

	return ret;
}

int icm20948_accel_stop(void)
{
	int first_error = 0;
	int ret;

	if (atomic_get(&icm20948_initialized) == 0) {
		return -ENODEV;
	}

	ret = gpio_pin_interrupt_configure_dt(&icm20948_int, GPIO_INT_DISABLE);
	icm20948_record_first_error(&first_error, ret);
	k_sem_reset(&icm20948_data_ready_sem);

	k_mutex_lock(&icm20948_io_lock, K_FOREVER);
	ret = icm20948_write_reg(ICM20948_REG_INT_ENABLE_1, 0U);
	icm20948_record_first_error(&first_error, ret);
	ret = icm20948_write_reg(ICM20948_REG_FIFO_EN_2, 0U);
	icm20948_record_first_error(&first_error, ret);
	if (atomic_get(&icm20948_consumer_failed) == 0) {
		ret = icm20948_drain_fifo();
		icm20948_record_first_error(&first_error, ret);
	}
	atomic_clear(&icm20948_streaming);
	atomic_inc(&icm20948_stream_generation);
	ret = icm20948_reset_fifo();
	icm20948_record_first_error(&first_error, ret);
	ret = icm20948_write_reg(ICM20948_REG_USER_CTRL,
				 ICM20948_USER_CTRL_I2C_IF_DIS);
	icm20948_record_first_error(&first_error, ret);
	ret = icm20948_write_reg(ICM20948_REG_PWR_MGMT_1,
				 ICM20948_PWR_MGMT_1_SLEEP |
				 ICM20948_PWR_MGMT_1_TEMP_DIS |
				 ICM20948_PWR_MGMT_1_CLKSEL_AUTO);
	icm20948_record_first_error(&first_error, ret);
	k_mutex_unlock(&icm20948_io_lock);

	if (first_error == 0) {
		LOG_INF("ICM-20948 accelerometer stopped");
	}

	return first_error;
}

int icm20948_accel_get_latest(struct icm20948_accel_sample *sample)
{
	if (sample == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&icm20948_sample_lock, K_FOREVER);
	if (!icm20948_sample_valid) {
		k_mutex_unlock(&icm20948_sample_lock);
		return -ENODATA;
	}

	*sample = icm20948_latest_sample;
	k_mutex_unlock(&icm20948_sample_lock);

	return 0;
}
