#include "max30001.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(max30001, CONFIG_LOG_LEVEL_MAX30001);

/*
 * PIN LIST
 *
 * AOUT: NC (TEST POINT ONLY)
 * CS:     41 (1.09)
 * MOSI:   38 (1.06)
 * SCLK:   40 (1.08)
 * MISO:    3 (0.03)
 * INTB:   21 (0.21)
 * INTB2:  19 (0.19)
 * 32K_OSC: EXTERNAL PROVIDED FULL SWING
 */

#define MAX30001_NODE DT_NODELABEL(max30001)

#define MAX30001_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define MAX30001_REG_NO_OP     0x00u
#define MAX30001_REG_STATUS    0x01u
#define MAX30001_REG_EN_INT    0x02u
#define MAX30001_REG_MNGR_INT  0x04u
#define MAX30001_REG_SYNCH     0x09u
#define MAX30001_REG_FIFO_RST  0x0Au
#define MAX30001_REG_INFO      0x0Fu
#define MAX30001_REG_CNFG_GEN  0x10u
#define MAX30001_REG_CNFG_EMUX 0x14u
#define MAX30001_REG_CNFG_ECG  0x15u
#define MAX30001_REG_ECG_FIFO  0x21u

#define MAX30001_READ_BIT 0x01u
#define MAX30001_INFO_PATTERN 0x05u
#define MAX30001_INFO_PATTERN_SHIFT 20u
#define MAX30001_INFO_REV_ID_SHIFT 16u

#define MAX30001_STATUS_PLLINT BIT(8)

#define MAX30001_CNFG_EMUX_ECGP_ECGN 0x000000u
#define MAX30001_CNFG_ECG_512HZ      0x006000u
#define MAX30001_MNGR_INT_EFIT_8     0x3B0004u
#define MAX30001_EN_INT_ECG_FIFO     0xC00003u
#define MAX30001_CNFG_GEN_ECG_BIAS   0x080017u

#define MAX30001_PLL_LOCK_TIMEOUT_MS 500
#define MAX30001_PLL_LOCK_POLL_MS    10

#define MAX30001_READ_CMD(reg)  (((reg) << 1) | MAX30001_READ_BIT)
#define MAX30001_WRITE_CMD(reg) ((reg) << 1)

static const struct spi_dt_spec max30001_spi =
	SPI_DT_SPEC_GET(MAX30001_NODE, MAX30001_SPI_OPERATION, 0);

static int max30001_transceive(const uint8_t tx_buffer[4], uint8_t rx_buffer[4])
{
	const struct spi_buf tx_buf = {
		.buf = (void *)tx_buffer,
		.len = 4,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = 4,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	return spi_transceive_dt(&max30001_spi, &tx, &rx);
}

static int max30001_write_reg(uint8_t reg, uint32_t value)
{
	uint8_t rx_buffer[4] = {0};
	uint8_t tx_buffer[4] = {
		MAX30001_WRITE_CMD(reg),
		(uint8_t)((value >> 16) & 0xFFu),
		(uint8_t)((value >> 8) & 0xFFu),
		(uint8_t)(value & 0xFFu),
	};

	return max30001_transceive(tx_buffer, rx_buffer);
}

static int max30001_read_reg(uint8_t reg, uint32_t *value)
{
	int ret;
	uint8_t rx_buffer[4] = {0};
	uint8_t tx_buffer[4] = {
		MAX30001_READ_CMD(reg),
		0x00,
		0x00,
		0x00,
	};

	if (value == NULL) {
		return -EINVAL;
	}

	ret = max30001_transceive(tx_buffer, rx_buffer);
	if (ret != 0) {
		return ret;
	}

	*value = ((uint32_t)rx_buffer[1] << 16) |
		 ((uint32_t)rx_buffer[2] << 8) |
		 (uint32_t)rx_buffer[3];

	return 0;
}

static int max30001_command(uint8_t reg)
{
	return max30001_write_reg(reg, 0x000000u);
}

static int max30001_wait_pll_lock(void)
{
	int ret;
	uint32_t status = 0;

	for (int elapsed_ms = 0; elapsed_ms < MAX30001_PLL_LOCK_TIMEOUT_MS;
	     elapsed_ms += MAX30001_PLL_LOCK_POLL_MS) {
		ret = max30001_read_reg(MAX30001_REG_STATUS, &status);
		if (ret != 0) {
			return ret;
		}

		if ((status & MAX30001_STATUS_PLLINT) == 0u) {
			return 0;
		}

		k_sleep(K_MSEC(MAX30001_PLL_LOCK_POLL_MS));
	}

	LOG_ERR("MAX30001 PLL did not lock, STATUS=0x%06x", (unsigned int)status);
	return -ETIMEDOUT;
}

static void max30001_decode_ecg_sample(uint32_t raw,
				       struct max30001_ecg_sample *sample)
{
	sample->raw = raw & 0xFFFFFFu;
	sample->etag = (uint8_t)((raw >> 3) & 0x07u);
	sample->ptag = (uint8_t)(raw & 0x07u);
	sample->time_valid = sample->etag <= MAX30001_ECG_ETAG_FAST_EOF;
	sample->data_valid = (sample->etag == MAX30001_ECG_ETAG_VALID) ||
			     (sample->etag == MAX30001_ECG_ETAG_VALID_EOF);
	sample->eof = (sample->etag == MAX30001_ECG_ETAG_VALID_EOF) ||
		      (sample->etag == MAX30001_ECG_ETAG_FAST_EOF);
}

int max30001_probe(uint32_t *info)
{
	int ret;
	uint32_t info_value;
	uint8_t info_pattern;
	uint8_t rev_id;

	if (!spi_is_ready_dt(&max30001_spi)) {
		LOG_ERR("MAX30001 SPI bus is not ready");
		return -ENODEV;
	}

	/*
	 * The INFO register should not be the first transaction after power-up
	 * or SW_RST, so start with a harmless NO_OP write transaction.
	 */
	ret = max30001_write_reg(MAX30001_REG_NO_OP, 0x000000u);
	if (ret != 0) {
		LOG_ERR("MAX30001 NO_OP failed: %d", ret);
		return ret;
	}

	ret = max30001_read_reg(MAX30001_REG_INFO, &info_value);
	if (ret != 0) {
		LOG_ERR("MAX30001 INFO read failed: %d", ret);
		return ret;
	}

	if (info != NULL) {
		*info = info_value;
	}

	info_pattern = (uint8_t)((info_value >> MAX30001_INFO_PATTERN_SHIFT) & 0x0Fu);
	rev_id = (uint8_t)((info_value >> MAX30001_INFO_REV_ID_SHIFT) & 0x0Fu);

	LOG_INF("MAX30001 INFO=0x%06x REV_ID=0x%x",
		(unsigned int)info_value, rev_id);

	if (info_pattern != MAX30001_INFO_PATTERN) {
		LOG_ERR("MAX30001 INFO fixed pattern mismatch: 0x%x", info_pattern);
		return -EIO;
	}

	return 0;
}

int max30001_ecg_init_512(void)
{
	int ret;

	ret = max30001_probe(NULL);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_write_reg(MAX30001_REG_EN_INT, 0x000000u);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_command(MAX30001_REG_FIFO_RST);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_write_reg(MAX30001_REG_CNFG_EMUX,
				 MAX30001_CNFG_EMUX_ECGP_ECGN);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_write_reg(MAX30001_REG_CNFG_ECG,
				 MAX30001_CNFG_ECG_512HZ);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_write_reg(MAX30001_REG_MNGR_INT,
				 MAX30001_MNGR_INT_EFIT_8);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_write_reg(MAX30001_REG_CNFG_GEN,
				 MAX30001_CNFG_GEN_ECG_BIAS);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_wait_pll_lock();
	if (ret != 0) {
		return ret;
	}

	LOG_INF("MAX30001 ECG configured for 512 Hz");
	return 0;
}

int max30001_ecg_start(void)
{
	int ret;

	ret = max30001_write_reg(MAX30001_REG_EN_INT,
				 MAX30001_EN_INT_ECG_FIFO);
	if (ret != 0) {
		return ret;
	}

	ret = max30001_command(MAX30001_REG_SYNCH);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("MAX30001 ECG streaming started");
	return 0;
}

int max30001_ecg_stop(void)
{
	int ret;
	int first_error = 0;

	ret = max30001_write_reg(MAX30001_REG_EN_INT, 0x000000u);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}

	ret = max30001_write_reg(MAX30001_REG_CNFG_GEN, 0x000000u);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}

	ret = max30001_command(MAX30001_REG_FIFO_RST);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}

	LOG_INF("MAX30001 ECG streaming stopped");
	return first_error;
}

int max30001_ecg_read_fifo(struct max30001_ecg_sample *samples,
			   size_t max_samples,
			   size_t *sample_count)
{
	int ret;
	size_t count = 0;

	if (samples == NULL || max_samples == 0) {
		return -EINVAL;
	}

	for (size_t i = 0; i < max_samples; i++) {
		uint32_t raw;
		struct max30001_ecg_sample sample;

		ret = max30001_read_reg(MAX30001_REG_ECG_FIFO, &raw);
		if (ret != 0) {
			if (sample_count != NULL) {
				*sample_count = count;
			}
			return ret;
		}

		max30001_decode_ecg_sample(raw, &sample);

		if (sample.etag == MAX30001_ECG_ETAG_EMPTY) {
			break;
		}

		if (sample.etag == MAX30001_ECG_ETAG_OVERFLOW) {
			LOG_ERR("MAX30001 ECG FIFO overflow");
			(void)max30001_command(MAX30001_REG_FIFO_RST);
			if (sample_count != NULL) {
				*sample_count = 0;
			}
			return -EOVERFLOW;
		}

		if (!sample.time_valid) {
			LOG_WRN("MAX30001 ECG FIFO unused ETAG: %u", sample.etag);
			break;
		}

		samples[count++] = sample;

		if (sample.eof) {
			break;
		}
	}

	if (sample_count != NULL) {
		*sample_count = count;
	}

	return 0;
}
