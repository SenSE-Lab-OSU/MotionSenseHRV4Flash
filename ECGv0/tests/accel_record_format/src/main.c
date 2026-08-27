#include <string.h>

#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include "accelRecordFormat.h"

static uint32_t get_u32_le(const uint8_t *src)
{
	return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static uint32_t crc_with_zeroed_field(uint8_t *data, size_t length,
				      size_t crc_offset)
{
	uint8_t saved[4];
	uint32_t crc;

	memcpy(saved, &data[crc_offset], sizeof(saved));
	memset(&data[crc_offset], 0, sizeof(saved));
	crc = crc32_ieee(data, length);
	memcpy(&data[crc_offset], saved, sizeof(saved));
	return crc;
}

ZTEST(accel_record_format, test_crc32_ieee_known_vector)
{
	static const uint8_t vector[] = "123456789";

	zassert_equal(crc32_ieee(vector, sizeof(vector) - 1U), 0xcbf43926U,
		      "CRC-32/IEEE vector does not match");
}

ZTEST(accel_record_format, test_header_is_byte_exact_and_valid)
{
	uint8_t header[ACCEL_RECORD_FORMAT_BLOCK_BYTES];

	accel_record_format_build_header(header);
	zassert_mem_equal(header, "ACF3", 4U, "bad header magic");
	zassert_equal(header[4], 3U, "bad format version LSB");
	zassert_equal(header[5], 0U, "bad format version MSB");
	zassert_equal(header[6], 2U, "bad sample format LSB");
	zassert_equal(header[7], 0U, "bad sample format MSB");
	zassert_equal(get_u32_le(&header[8]), 1125U, "bad ODR numerator");
	zassert_equal(get_u32_le(&header[12]), 2U, "bad ODR denominator");
	zassert_equal(header[16], 2U, "bad full scale");
	zassert_equal(header[18], 0U, "bad counts/g LSB");
	zassert_equal(header[19], 0x40U, "bad counts/g MSB");
	zassert_equal(get_u32_le(&header[20]),
		      crc_with_zeroed_field(header, sizeof(header), 20U),
		      "bad header CRC");
	zassert_equal(get_u32_le(&header[24]), 512U, "bad anchor clock rate");
	zassert_equal(get_u32_le(&header[28]), 32U, "bad FSYNC edge interval");
	zassert_equal(header[32], 0U, "bad FSYNC axis");
	zassert_equal(header[33], 0U, "bad FSYNC bit");
	zassert_equal(header[34], 1U, "bad timestamp algorithm");
	zassert_equal(header[35], 32U, "bad timestamp window");
	zassert_equal(header[36], 0U, "reserved header bytes must be zero");
	zassert_equal(header[sizeof(header) - 1U], 0U,
		      "reserved header bytes must be zero");
}

ZTEST(accel_record_format, test_fixed_file_layout_and_trailer)
{
	uint8_t trailer[ACCEL_RECORD_FORMAT_TRAILER_BYTES];
	uint32_t valid_data_bytes =
		ACCEL_RECORD_FORMAT_DATA_BYTES - ACCEL_RECORD_FORMAT_BLOCK_BYTES;

	zassert_equal(ACCEL_RECORD_FORMAT_FILE_BYTES, 4U * 1024U * 1024U,
		      "accelerometer chunks must be 4 MiB");
	zassert_equal(ACCEL_RECORD_FORMAT_FULL_BLOCKS_PER_FILE, 1022U,
		      "unexpected number of full blocks per chunk");
	zassert_equal(ACCEL_RECORD_FORMAT_TRAILER_OFFSET +
		      ACCEL_RECORD_FORMAT_TRAILER_BYTES,
		      ACCEL_RECORD_FORMAT_FILE_BYTES, "trailer is not at chunk end");

	accel_record_format_build_trailer(trailer, valid_data_bytes);
	zassert_mem_equal(trailer, "ACT2", 4U, "bad trailer magic");
	zassert_equal(get_u32_le(&trailer[4]), valid_data_bytes,
		      "bad trailer data length");
	zassert_equal(get_u32_le(&trailer[8]),
		      crc_with_zeroed_field(trailer, sizeof(trailer), 8U),
		      "bad trailer CRC");
	zassert_equal(trailer[12], 0U, "trailer reserved bytes must be zero");
}

ZTEST(accel_record_format, test_full_block_little_endian_axes_and_crc)
{
	uint8_t block[ACCEL_RECORD_FORMAT_BLOCK_BYTES] = {0};
	uint8_t fifo_sample[] = {0x12U, 0x34U, 0xfeU, 0xdcU, 0x80U, 0x01U};
	static const uint8_t expected_sample[] = {0x34U, 0x12U, 0xdcU,
						  0xfeU, 0x01U, 0x80U};
	size_t length;

	accel_record_format_store_fifo_sample(block, 0U, fifo_sample);
	for (size_t sample = 1U;
	     sample < ACCEL_RECORD_FORMAT_SAMPLES_PER_BLOCK; sample++) {
		accel_record_format_store_fifo_sample(block, sample, fifo_sample);
	}
	length = accel_record_format_finalize_block(
		block, ACCEL_RECORD_FORMAT_SAMPLES_PER_BLOCK, 680U, 0x12345678U);

	zassert_equal(length, ACCEL_RECORD_FORMAT_BLOCK_BYTES, "bad full length");
	zassert_mem_equal(block, "ACB1", 4U, "bad block magic");
	zassert_equal(get_u32_le(&block[4]), 0x12345678U,
		      "bad reserved timer output");
	zassert_equal(get_u32_le(&block[8]), 680U, "bad first sample sequence");
	zassert_mem_equal(&block[16], expected_sample, sizeof(expected_sample),
			  "bad LE sample");
	zassert_equal(get_u32_le(&block[12]),
		      crc_with_zeroed_field(block, length, 12U), "bad block CRC");
}

ZTEST(accel_record_format, test_short_final_block_uses_actual_length)
{
	uint8_t block[ACCEL_RECORD_FORMAT_BLOCK_BYTES] = {0};
	static const uint8_t fifo_sample[] = {0x00U, 0x01U, 0xffU,
					      0xfeU, 0x7fU, 0xffU};
	size_t length;

	for (size_t sample = 0U; sample < 3U; sample++) {
		accel_record_format_store_fifo_sample(block, sample, fifo_sample);
	}
	length = accel_record_format_finalize_block(block, 3U, 1360U, 0x10203040U);

	zassert_equal(length, 34U, "short-block length includes padding");
	zassert_equal(get_u32_le(&block[4]), 0x10203040U,
		      "bad short-block timer output");
	zassert_equal(get_u32_le(&block[8]), 1360U, "bad short-block sequence");
	zassert_equal(get_u32_le(&block[12]),
		      crc_with_zeroed_field(block, length, 12U), "bad short-block CRC");
}

ZTEST_SUITE(accel_record_format, NULL, NULL, NULL, NULL, NULL);
