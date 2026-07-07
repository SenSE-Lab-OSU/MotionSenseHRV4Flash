#include "max30001.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

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

#define MAX30001_REG_NO_OP 0x00u
#define MAX30001_REG_INFO  0x0Fu

#define MAX30001_READ_BIT 0x01u
#define MAX30001_INFO_PATTERN 0x05u
#define MAX30001_INFO_PATTERN_SHIFT 20u
#define MAX30001_INFO_REV_ID_SHIFT 16u

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

	ret = max30001_transceive(tx_buffer, rx_buffer);
	if (ret != 0) {
		return ret;
	}

	*value = ((uint32_t)rx_buffer[1] << 16) |
		 ((uint32_t)rx_buffer[2] << 8) |
		 (uint32_t)rx_buffer[3];

	return 0;
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
