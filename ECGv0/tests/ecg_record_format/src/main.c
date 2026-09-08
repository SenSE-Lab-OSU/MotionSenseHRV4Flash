#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "ecgRecordFormat.h"

static uint32_t get_u32_le(const uint8_t *source)
{
	return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
	       ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

static uint64_t get_u64_le(const uint8_t *source)
{
	uint64_t value = 0U;
	uint8_t index;

	for (index = 0U; index < 8U; index++) {
		value |= (uint64_t)source[index] << (index * 8U);
	}

	return value;
}

static void build_full_block(uint8_t *block, uint32_t first_tick, uint32_t first_index)
{
	uint16_t index;

	msense_ecg_block_begin(block, first_tick, first_index);
	for (index = 0U; index < MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK; index++) {
		uint32_t raw24 = ((uint32_t)index & 0x3ffffU) << 6;

		raw24 |= ((uint32_t)index % 4U) << 3;
		raw24 |= (uint32_t)index % 8U;
		zassert_ok(msense_ecg_block_append_sample(block, index, raw24),
			   "sample %u rejected", index);
	}
	zassert_ok(msense_ecg_block_finalize(block,
			MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK), "block finalization failed");
}

ZTEST(ecg_record_format, test_block_is_byte_exact_and_crc_protected)
{
	uint8_t block[MSENSE_ECG_BLOCK_BYTES];
	struct msense_ecg_block_info info;

	build_full_block(block, 0xffffff00U, 0xffffff00U);
	zassert_mem_equal(block, "ECB2", 4U, "bad block magic");
	zassert_equal(get_u32_le(&block[4]), 0xffffff00U, "bad first RTC tick");
	zassert_equal(get_u32_le(&block[8]), 0xffffff00U, "bad first sample index");
	zassert_equal(get_u32_le(&block[12]), 0x3ab7821cU, "bad golden block CRC");
	zassert_mem_equal(&block[16], "\x00\x00\x00\x00\x00\x49", 6U,
			  "raw words are not MSB first");
	zassert_ok(msense_ecg_block_validate(block, &info), "valid block rejected");
	zassert_equal(info.first_rtc_tick, 0xffffff00U, "bad validated first tick");
	zassert_equal(info.first_sample_index, 0xffffff00U, "bad validated first index");
	zassert_true(!memcmp(&block[4090], "\0\0\0\0\0\0", 6U),
		     "reserved tail was not zeroed");

	block[16] ^= 1U;
	zassert_equal(msense_ecg_block_validate(block, NULL), -EBADMSG,
		      "payload corruption passed the CRC");
	block[16] ^= 1U;
	block[4090] = 1U;
	zassert_equal(msense_ecg_block_validate(block, NULL), -EINVAL,
		      "nonzero reserved byte accepted");
}

ZTEST(ecg_record_format, test_block_is_full_and_continuous_across_counter_wrap)
{
	uint8_t previous_block[MSENSE_ECG_BLOCK_BYTES];
	uint8_t current_block[MSENSE_ECG_BLOCK_BYTES];
	struct msense_ecg_block_info previous;
	struct msense_ecg_block_info current;

	msense_ecg_block_begin(previous_block, 0U, 0U);
	zassert_ok(msense_ecg_block_append_sample(previous_block, 0U, 0U),
		   "valid word rejected");
	zassert_equal(msense_ecg_block_finalize(previous_block, 1U), -EINVAL,
		      "short block finalized");

	build_full_block(previous_block, 0xffffff00U, 0xffffff00U);
	build_full_block(current_block, 0x44eU, 0x44eU);
	zassert_ok(msense_ecg_block_validate(previous_block, &previous),
		   "previous block rejected");
	zassert_ok(msense_ecg_block_validate(current_block, &current),
		   "current block rejected");
	zassert_ok(msense_ecg_block_validate_continuity(&previous, &current),
		   "modulo-32-bit continuity rejected");

	current.first_sample_index++;
	zassert_equal(msense_ecg_block_validate_continuity(&previous, &current), -EILSEQ,
		      "continuity gap accepted");
	memcpy(current_block, "ECB1", 4U);
	zassert_equal(msense_ecg_block_validate(current_block, NULL), -EINVAL,
		      "ECB1 accepted by ECB2 decoder");
}

ZTEST(ecg_record_format, test_file_header_is_byte_exact_and_crc_protected)
{
	uint8_t header[MSENSE_ECG_FILE_HEADER_BYTES];

	msense_ecg_file_header_build(header, 0x0123456789abcdefULL, 7U);
	zassert_mem_equal(header, "ECF2", 4U, "bad header magic");
	zassert_equal(get_u32_le(&header[4]), 7U, "bad chunk index");
	zassert_equal(get_u64_le(&header[8]), 0x0123456789abcdefULL,
		      "bad recording ID");
	zassert_equal(get_u32_le(&header[16]), 0x3fdb88b5U, "bad golden header CRC");
	zassert_ok(msense_ecg_file_header_validate(header), "valid header rejected");
	header[20] = 1U;
	zassert_equal(msense_ecg_file_header_validate(header), -EINVAL,
		      "nonzero header reserved byte accepted");
}

ZTEST_SUITE(ecg_record_format, NULL, NULL, NULL, NULL, NULL);
