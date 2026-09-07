#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "ecgRecordFormat.h"

static uint32_t get_u32_le(const uint8_t *source)
{
	return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
	       ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

ZTEST(ecg_record_format, test_block_is_byte_exact_and_crc_protected)
{
	uint8_t block[MSENSE_ECG_BLOCK_BYTES];
	struct msense_ecg_block_info info;

	msense_ecg_block_begin(block, 0x89abcdefU, 0xfffffffeU, 0U);
	zassert_ok(msense_ecg_block_append_sample(block, 0U, 0x123456U),
		   "first MAX30001 word rejected");
	zassert_ok(msense_ecg_block_append_sample(block, 1U, 0x654313U),
		   "EOF MAX30001 word rejected");
	zassert_ok(msense_ecg_block_finalize(block, 2U), "block finalization failed");

	zassert_mem_equal(block, "ECB1", 4U, "bad block magic");
	zassert_equal(get_u32_le(&block[4]), 0x89abcdefU, "bad first RTC tick");
	zassert_equal(get_u32_le(&block[8]), 0xfffffffeU, "bad first sample index");
	zassert_equal(block[12], 2U, "bad sample count low byte");
	zassert_equal(block[14], 1U, "bad sample format");
	zassert_mem_equal(&block[20], "\x12\x34\x56\x65\x43\x13", 6U,
			  "raw words are not MSB first");
	zassert_ok(msense_ecg_block_validate(block, &info), "valid block rejected");
	zassert_equal(info.sample_count, 2U, "bad validated count");
	zassert_equal(block[MSENSE_ECG_BLOCK_BYTES - 1U], 0U, "tail was not zeroed");

	block[20] ^= 1U;
	zassert_equal(msense_ecg_block_validate(block, NULL), -EBADMSG,
		      "payload corruption passed the CRC");
	block[20] ^= 1U;
	block[30] = 1U;
	zassert_equal(msense_ecg_block_validate(block, NULL), -EINVAL,
		      "nonzero unused payload accepted");
}

ZTEST(ecg_record_format, test_block_rejects_invalid_etag_and_checks_wrap_continuity)
{
	uint8_t previous_block[MSENSE_ECG_BLOCK_BYTES];
	uint8_t current_block[MSENSE_ECG_BLOCK_BYTES];
	struct msense_ecg_block_info previous;
	struct msense_ecg_block_info current;

	msense_ecg_block_begin(previous_block, 0xffffffffU, 0xffffffffU, 0U);
	zassert_ok(msense_ecg_block_append_sample(previous_block, 0U, 0x000000U),
		   "valid word rejected");
	zassert_ok(msense_ecg_block_finalize(previous_block, 1U), "first block failed");
	zassert_ok(msense_ecg_block_validate(previous_block, &previous), "first block invalid");

	msense_ecg_block_begin(current_block, 0U, 0U, 0U);
	zassert_ok(msense_ecg_block_append_sample(current_block, 0U, 0x000003U),
		   "valid EOF word rejected");
	zassert_ok(msense_ecg_block_finalize(current_block, 1U), "second block failed");
	zassert_ok(msense_ecg_block_validate(current_block, &current), "second block invalid");
	zassert_ok(msense_ecg_block_validate_continuity(&previous, &current),
		   "modulo-32-bit continuity rejected");

	msense_ecg_block_begin(current_block, 1U, 1U, 0U);
	zassert_equal(msense_ecg_block_append_sample(current_block, 0U, 0x000030U), -EINVAL,
		      "reserved MAX30001 ETAG accepted");
}

ZTEST(ecg_record_format, test_file_header_and_clean_trailer_validate)
{
	uint8_t header[MSENSE_ECG_FILE_HEADER_BYTES];
	uint8_t trailer[MSENSE_ECG_FILE_TRAILER_BYTES];

	msense_ecg_file_header_build(header, 0x1122334455667788ULL, 3U);
	zassert_mem_equal(header, "ECF1", 4U, "bad header magic");
	zassert_ok(msense_ecg_file_header_validate(header), "valid header rejected");

	msense_ecg_file_trailer_build(trailer, 2U, 3U, 17U, 20U, true);
	zassert_mem_equal(trailer, "ECT1", 4U, "bad trailer magic");
	zassert_ok(msense_ecg_file_trailer_validate(trailer), "valid trailer rejected");
	trailer[40] = 1U;
	zassert_equal(msense_ecg_file_trailer_validate(trailer), -EINVAL,
		      "nonzero trailer reserved byte accepted");
}

ZTEST_SUITE(ecg_record_format, NULL, NULL, NULL, NULL, NULL);
