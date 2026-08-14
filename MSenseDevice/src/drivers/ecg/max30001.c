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
#define MAX30001_REG_EN_INT2   0x03u
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
#define MAX30001_CNFG_ECG_512HZ      0x002000u
/* EFIT = 16, CLR_SAMP = self-clear, SAMP_IT = every ECG sample. */
#define MAX30001_MNGR_INT_EFIT_16    0x7B0004u
#define MAX30001_EN_INT_ECG_FIFO     0xC00003u
#define MAX30001_EN_INT2_MASKED       0x000003u
#define MAX30001_EN_INT2_ECG_SAMP     0x000203u
#define MAX30001_CNFG_GEN_ECG_BIAS   0x080017u

#define MAX30001_PLL_LOCK_TIMEOUT_MS 500
#define MAX30001_PLL_LOCK_POLL_MS    10
#define MAX30001_SAMP_STALE_CLEAR_US 1000

#define MAX30001_READ_CMD(reg)  (((reg) << 1) | MAX30001_READ_BIT)
#define MAX30001_WRITE_CMD(reg) ((reg) << 1)

static const struct spi_dt_spec max30001_spi =
	SPI_DT_SPEC_GET(MAX30001_NODE, MAX30001_SPI_OPERATION, 0);

/**
 * @brief Perform a single full-duplex 4-byte SPI transaction with the MAX30001.
 *
 * Every MAX30001 register access is exactly 32 bits on the wire: one
 * command/address byte followed by three data bytes. This helper wraps the
 * Zephyr spi_transceive_dt() call, packaging the caller's fixed-size TX and
 * RX arrays into the spi_buf_set structures the SPI driver expects. It is the
 * single low-level transport primitive on which all other register
 * read/write helpers in this driver are built.
 *
 * @param tx_buffer 4-byte buffer to transmit (command byte + 3 data bytes).
 * @param rx_buffer 4-byte buffer that receives the bytes clocked back in.
 *
 * @retval 0 on success, negative errno from the SPI driver on failure.
 */
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

/**
 * @brief Write a 24-bit value to a MAX30001 register.
 *
 * Builds the 4-byte SPI frame for a register write: the first byte is the
 * register address shifted left with the R/W bit cleared (write), followed by
 * the 24-bit register value in big-endian (MSB-first) order. The bytes
 * clocked back during a write are meaningless and are discarded.
 *
 * @param reg   Register address (one of the MAX30001_REG_* constants).
 * @param value 24-bit value to write (upper 8 bits of the u32 are ignored).
 *
 * @retval 0 on success, negative errno from the SPI transaction on failure.
 */
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

/**
 * @brief Read a 24-bit value from a MAX30001 register.
 *
 * Sends the register address with the read bit set, then clocks out three
 * dummy bytes while the device shifts the register contents back. The three
 * received data bytes are reassembled MSB-first into a 24-bit value placed in
 * the low bits of *value.
 *
 * @param reg   Register address (one of the MAX30001_REG_* constants).
 * @param value Output pointer for the 24-bit register value; must be non-NULL.
 *
 * @retval 0 on success.
 * @retval -EINVAL if value is NULL.
 * @retval Other negative errno from the SPI transaction on failure.
 */
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

/**
 * @brief Issue a self-clearing command to the MAX30001.
 *
 * Some MAX30001 registers (SYNCH, FIFO_RST, SW_RST) act as command triggers:
 * writing any value to them executes the action rather than storing data.
 * This helper writes 0x000000 to such a register to fire the command.
 *
 * @param reg Command register address (e.g. MAX30001_REG_SYNCH,
 *            MAX30001_REG_FIFO_RST).
 *
 * @retval 0 on success, negative errno from the SPI transaction on failure.
 */
static int max30001_command(uint8_t reg)
{
	return max30001_write_reg(reg, 0x000000u);
}

/**
 * @brief Block until the MAX30001 internal PLL reports lock.
 *
 * After the analog front end is enabled (CNFG_GEN written), the chip's PLL
 * needs time to lock before ECG samples are meaningful. The STATUS register's
 * PLLINT bit stays set while the PLL is unlocked, so this routine polls
 * STATUS every MAX30001_PLL_LOCK_POLL_MS (10 ms) until the bit clears or
 * MAX30001_PLL_LOCK_TIMEOUT_MS (500 ms) elapses. Called at the end of ECG
 * initialization to guarantee the device is producing valid data before
 * streaming starts.
 *
 * @retval 0 when the PLL locks within the timeout.
 * @retval -ETIMEDOUT if the PLL never locks.
 * @retval Other negative errno if the STATUS register read fails.
 */
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

/**
 * @brief Decode a raw 24-bit ECG FIFO word into a structured sample.
 *
 * Each word read from the ECG FIFO packs an 18-bit voltage sample together
 * with a 3-bit ETAG (data status) and 3-bit PTAG (pace event pointer) in its
 * low bits. This function extracts those fields and derives three
 * convenience flags from the ETAG:
 *  - time_valid: the word occupies a real sample time slot (ETAG 0-3), as
 *    opposed to EMPTY/OVERFLOW markers.
 *  - data_valid: the voltage reading itself is trustworthy (ETAG VALID or
 *    VALID_EOF; FAST tags mean the filters were still settling).
 *  - eof: this is the last sample of the current FIFO burst (either EOF tag).
 *
 * @param raw    Raw 24-bit word read from MAX30001_REG_ECG_FIFO.
 * @param sample Output structure populated with the decoded fields.
 */
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

/**
 * @brief Verify the MAX30001 is present and responding on the SPI bus.
 *
 * Sanity-checks the hardware connection before any configuration is
 * attempted. The sequence is:
 *  1. Confirm the Zephyr SPI bus device is ready.
 *  2. Issue a harmless NO_OP write, because the datasheet advises that the
 *     INFO register should not be the very first transaction after power-up
 *     or a software reset.
 *  3. Read the INFO register and check that its fixed identification pattern
 *     (0x5 in bits [23:20]) matches, which proves real data is coming back
 *     rather than floating-bus noise.
 *
 * The silicon revision ID is logged for diagnostics.
 *
 * @param info Optional output for the raw 24-bit INFO register value; may be
 *             NULL if the caller only wants the presence check.
 *
 * @retval 0 when the device responds with the expected INFO pattern.
 * @retval -ENODEV if the SPI bus is not ready.
 * @retval -EIO if the INFO fixed pattern does not match.
 * @retval Other negative errno from the underlying SPI transactions.
 */
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

/**
 * @brief Configure the MAX30001 for 512 Hz ECG acquisition.
 *
 * Performs the full bring-up sequence for the ECG channel, leaving the
 * device configured and its PLL locked but with interrupts still masked
 * (streaming does not begin until max30001_ecg_start()):
 *  1. Probe the device to confirm it is alive.
 *  2. Mask both interrupt outputs so no stale sources fire during
 *     reconfiguration.
 *  3. Reset the FIFO to discard any old samples.
 *  4. Route the ECG input mux to the ECGP/ECGN electrode pins (CNFG_EMUX).
 *  5. Select the 512 Hz sample rate (CNFG_ECG).
 *  6. Set the FIFO interrupt threshold (EFIT) to 16 samples (MNGR_INT), so
 *     INTB asserts once at least 16 samples are queued.
 *  7. Enable the ECG channel with internal lead bias resistors (CNFG_GEN).
 *  8. Wait for the PLL to lock before returning.
 *
 * @retval 0 on success.
 * @retval Negative errno if any register access fails or the PLL does not
 *         lock within its timeout.
 */
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

	/* Mask SAMP while the channel and PLL are being brought up. */
	ret = max30001_write_reg(MAX30001_REG_EN_INT2,
				 MAX30001_EN_INT2_MASKED);
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
				 MAX30001_MNGR_INT_EFIT_16);
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

/**
 * @brief Begin ECG sample streaming.
 *
 * Arms the ECG FIFO interrupt sources on INTB, then issues SYNCH to reset the
 * FIFO and internal sample clock. Only after SYNCH does it route the
 * self-clearing every-sample SAMP signal to INT2B. A 1 ms settling delay lets
 * any pre-SYNCH SAMP pulse expire before it is routed to the GPIO. This
 * ordering prevents a recorder from using a pulse generated by the prior
 * sample timeline as the first-sample timing anchor. The ECG output pipeline
 * delay is at least 19.836 ms, leaving ample time before the first post-SYNCH
 * sample reaches FIFO.
 *
 * @retval 0 on success, negative errno if a register access fails.
 */
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

	/* CLR_SAMP self-clears after ~0.5 ms at 512 Hz; wait two pulse widths. */
	k_busy_wait(MAX30001_SAMP_STALE_CLEAR_US);

	ret = max30001_write_reg(MAX30001_REG_EN_INT2,
				 MAX30001_EN_INT2_ECG_SAMP);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("MAX30001 ECG streaming started");
	return 0;
}

/**
 * @brief Stop routing the ECG sample synchronization pulse to INT2B.
 *
 * The ECG recorder captures one post-SYNCH SAMP edge to establish its RTC
 * time base, then calls this routine so the dedicated GPIO does not continue
 * interrupting at 512 Hz. INT2B remains configured as a released open-drain
 * output, but no status source is routed to it.
 */
int max30001_ecg_disable_samp_interrupt(void)
{
	return max30001_write_reg(MAX30001_REG_EN_INT2,
				 MAX30001_EN_INT2_MASKED);
}

/**
 * @brief Stop ECG streaming and power down the analog front end.
 *
 * Tears down acquisition in four steps: mask INT2B and INTB interrupt
 * sources, disable the ECG channel and lead bias (CNFG_GEN = 0), and reset
 * the FIFO to discard any samples still queued. Unlike the other
 * driver calls, this is a best-effort cleanup: every step is attempted even
 * if an earlier one fails, and the first error encountered (if any) is
 * returned so callers can safely use it in error paths.
 *
 * @retval 0 if all steps succeeded, otherwise the errno of the first step
 *         that failed.
 */
int max30001_ecg_stop(void)
{
	int ret;
	int first_error = 0;

	ret = max30001_write_reg(MAX30001_REG_EN_INT2,
				 MAX30001_EN_INT2_MASKED);
	if (ret != 0 && first_error == 0) {
		first_error = ret;
	}

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

/**
 * @brief Drain decoded ECG samples from the device FIFO.
 *
 * Repeatedly reads the ECG_FIFO register (up to max_samples times), decoding
 * each 24-bit word with max30001_decode_ecg_sample() and copying usable
 * samples into the caller's array. Reading stops early when:
 *  - an EMPTY tag is returned (FIFO exhausted) — normal completion;
 *  - an EOF tag is seen (last sample of the current burst) — normal
 *    completion, with that sample included;
 *  - an unused/unknown ETAG appears — a warning is logged and the read ends;
 *  - an OVERFLOW tag is returned — the FIFO contents are unrecoverable, so
 *    the FIFO is reset, *sample_count is set to 0, and -EOVERFLOW is
 *    returned.
 *
 * @param samples      Caller-provided array to fill with decoded samples.
 * @param max_samples  Capacity of the samples array; must be > 0.
 * @param sample_count Optional output for the number of samples stored; set
 *                     on both success and failure paths when non-NULL.
 *
 * @retval 0 on success (including when zero samples were available).
 * @retval -EINVAL if samples is NULL or max_samples is 0.
 * @retval -EOVERFLOW if the device FIFO overflowed (FIFO is reset).
 * @retval Other negative errno if an SPI read fails partway through.
 */
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
