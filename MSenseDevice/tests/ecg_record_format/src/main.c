#include <stddef.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "ecgRecordFormat.h"

static uint32_t get_u32_le(const uint8_t *src)
{
	return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static uint8_t crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80u) != 0u ?
				(uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
		}
	}

	return crc;
}

ZTEST(ecg_record_format, test_frame_uses_little_endian_rtc_tick_and_crc)
{
	uint8_t frame[ECG_RECORD_FORMAT_FRAME_BYTES];

	ecg_record_format_build_sample_frame(frame, 2U, 5U, 0x123456U,
					 0x89abcdefU);

	zassert_mem_equal(frame, "\xa5\xec\x01", 3U, "bad frame prefix");
	zassert_equal(frame[3], 0x2aU, "bad ETAG/PTAG flags");
	zassert_equal(get_u32_le(&frame[4]), 0x89abcdefU,
		      "RTC tick is not little-endian");
	zassert_equal(frame[8], 0x12U, "raw sample MSB changed");
	zassert_equal(frame[9], 0x34U, "raw sample middle byte changed");
	zassert_equal(frame[10], 0x56U, "raw sample LSB changed");
	zassert_equal(frame[11], crc8(&frame[2], 9U), "bad frame CRC");
}

ZTEST(ecg_record_format, test_time_valid_samples_advance_and_wrap_rtc_ticks)
{
	uint8_t first_batch[ECG_RECORD_FORMAT_FRAME_BYTES];
	uint8_t second_batch[ECG_RECORD_FORMAT_FRAME_BYTES];
	uint8_t third_batch[ECG_RECORD_FORMAT_FRAME_BYTES];

	/* Separate calls model samples drained in separate FIFO batches. */
	ecg_record_format_build_sample_frame(first_batch, 0U, 0U, 0U,
					 0xfffffffeU);
	ecg_record_format_build_sample_frame(second_batch, 1U, 0U, 0U,
					 0xffffffffU);
	ecg_record_format_build_sample_frame(third_batch, 2U, 0U, 0U, 0U);

	zassert_equal(get_u32_le(&first_batch[4]), 0xfffffffeU,
		      "first anchor tick changed");
	zassert_equal(get_u32_le(&second_batch[4]), 0xffffffffU,
		      "next sample did not advance one tick");
	zassert_equal(get_u32_le(&third_batch[4]), 0U,
		      "RTC tick did not wrap modulo 2^32");
}

ZTEST_SUITE(ecg_record_format, NULL, NULL, NULL, NULL, NULL);
