/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "msense_ecg_block_format.h"

#include <errno.h>
#include <string.h>

#include <zephyr/sys/crc.h>

#define ECG_BLOCK_MAGIC_OFFSET 0U
#define ECG_BLOCK_FIRST_TICK_OFFSET 4U
#define ECG_BLOCK_FIRST_INDEX_OFFSET 8U
#define ECG_BLOCK_SAMPLE_COUNT_OFFSET 12U
#define ECG_BLOCK_SAMPLE_FORMAT_OFFSET 14U
#define ECG_BLOCK_FLAGS_OFFSET 15U
#define ECG_BLOCK_CRC_OFFSET 16U
#define ECG_BLOCK_PAYLOAD_OFFSET 20U

#define ECG_FILE_CRC_OFFSET 40U
#define ECG_TRAILER_CRC_OFFSET 36U

static uint16_t ecg_get_u16_le(const uint8_t *source)
{
	return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static uint32_t ecg_get_u32_le(const uint8_t *source)
{
	return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
	       ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

static uint64_t ecg_get_u64_le(const uint8_t *source)
{
	uint64_t value = 0U;
	uint8_t index;

	for (index = 0U; index < 8U; index++) {
		value |= (uint64_t)source[index] << (index * 8U);
	}

	return value;
}

static void ecg_put_u16_le(uint8_t *destination, uint16_t value)
{
	destination[0] = (uint8_t)(value & 0xffU);
	destination[1] = (uint8_t)(value >> 8);
}

static void ecg_put_u32_le(uint8_t *destination, uint32_t value)
{
	destination[0] = (uint8_t)(value & 0xffU);
	destination[1] = (uint8_t)((value >> 8) & 0xffU);
	destination[2] = (uint8_t)((value >> 16) & 0xffU);
	destination[3] = (uint8_t)((value >> 24) & 0xffU);
}

static void ecg_put_u64_le(uint8_t *destination, uint64_t value)
{
	uint8_t index;

	for (index = 0U; index < 8U; index++) {
		destination[index] = (uint8_t)(value >> (index * 8U));
	}
}

static bool ecg_all_zero(const uint8_t *source, size_t length)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		if (source[index] != 0U) {
			return false;
		}
	}

	return true;
}

static int ecg_validate_page_crc(uint8_t *page, size_t crc_offset)
{
	uint32_t encoded_crc;
	uint32_t calculated_crc;

	encoded_crc = ecg_get_u32_le(&page[crc_offset]);
	memset(&page[crc_offset], 0, sizeof(encoded_crc));
	calculated_crc = crc32_ieee(page, MSENSE_ECG_BLOCK_BYTES);
	ecg_put_u32_le(&page[crc_offset], encoded_crc);

	return calculated_crc == encoded_crc ? 0 : -EBADMSG;
}

void msense_ecg_block_begin(uint8_t *block, uint32_t first_rtc_tick,
			    uint32_t first_sample_index, uint8_t flags)
{
	if (block == NULL) {
		return;
	}

	memset(block, 0, MSENSE_ECG_BLOCK_BYTES);
	memcpy(&block[ECG_BLOCK_MAGIC_OFFSET], "ECB1", 4U);
	ecg_put_u32_le(&block[ECG_BLOCK_FIRST_TICK_OFFSET], first_rtc_tick);
	ecg_put_u32_le(&block[ECG_BLOCK_FIRST_INDEX_OFFSET], first_sample_index);
	block[ECG_BLOCK_SAMPLE_FORMAT_OFFSET] = MSENSE_ECG_BLOCK_FORMAT_VERSION;
	block[ECG_BLOCK_FLAGS_OFFSET] = flags;
}

int msense_ecg_block_append_sample(uint8_t *block, uint16_t sample_count,
				   uint32_t raw24)
{
	size_t payload_offset;
	uint8_t etag;

	if (block == NULL || sample_count >= MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK ||
	    (raw24 & 0xff000000U) != 0U) {
		return -EINVAL;
	}

	etag = (uint8_t)((raw24 >> 3) & 0x07U);
	if (etag > 3U) {
		return -EINVAL;
	}

	payload_offset = ECG_BLOCK_PAYLOAD_OFFSET +
		(sample_count * MSENSE_ECG_BLOCK_SAMPLE_BYTES);
	block[payload_offset] = (uint8_t)(raw24 >> 16);
	block[payload_offset + 1U] = (uint8_t)(raw24 >> 8);
	block[payload_offset + 2U] = (uint8_t)raw24;
	return 0;
}

int msense_ecg_block_finalize(uint8_t *block, uint16_t sample_count)
{
	uint32_t crc;
	size_t payload_end;

	if (block == NULL || sample_count == 0U ||
	    sample_count > MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK) {
		return -EINVAL;
	}
	if (memcmp(&block[ECG_BLOCK_MAGIC_OFFSET], "ECB1", 4U) != 0 ||
	    block[ECG_BLOCK_SAMPLE_FORMAT_OFFSET] != MSENSE_ECG_BLOCK_FORMAT_VERSION ||
	    (block[ECG_BLOCK_FLAGS_OFFSET] & 0xfeU) != 0U) {
		return -EINVAL;
	}

	ecg_put_u16_le(&block[ECG_BLOCK_SAMPLE_COUNT_OFFSET], sample_count);
	payload_end = ECG_BLOCK_PAYLOAD_OFFSET +
		(sample_count * MSENSE_ECG_BLOCK_SAMPLE_BYTES);
	memset(&block[payload_end], 0, MSENSE_ECG_BLOCK_BYTES - payload_end);
	memset(&block[ECG_BLOCK_CRC_OFFSET], 0, sizeof(uint32_t));
	crc = crc32_ieee(block, MSENSE_ECG_BLOCK_BYTES);
	ecg_put_u32_le(&block[ECG_BLOCK_CRC_OFFSET], crc);
	return 0;
}

int msense_ecg_block_validate(uint8_t *block, struct msense_ecg_block_info *info)
{
	uint16_t sample_count;
	size_t payload_end;
	size_t index;
	int ret;

	if (block == NULL || memcmp(&block[ECG_BLOCK_MAGIC_OFFSET], "ECB1", 4U) != 0) {
		return -EINVAL;
	}

	sample_count = ecg_get_u16_le(&block[ECG_BLOCK_SAMPLE_COUNT_OFFSET]);
	if (sample_count == 0U || sample_count > MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK ||
	    block[ECG_BLOCK_SAMPLE_FORMAT_OFFSET] != MSENSE_ECG_BLOCK_FORMAT_VERSION ||
	    (block[ECG_BLOCK_FLAGS_OFFSET] & 0xfeU) != 0U) {
		return -EINVAL;
	}

	payload_end = ECG_BLOCK_PAYLOAD_OFFSET +
		(sample_count * MSENSE_ECG_BLOCK_SAMPLE_BYTES);
	if (!ecg_all_zero(&block[payload_end], MSENSE_ECG_BLOCK_BYTES - payload_end)) {
		return -EINVAL;
	}

	ret = ecg_validate_page_crc(block, ECG_BLOCK_CRC_OFFSET);
	if (ret != 0) {
		return ret;
	}

	for (index = 0U; index < sample_count; index++) {
		size_t sample_offset = ECG_BLOCK_PAYLOAD_OFFSET +
			(index * MSENSE_ECG_BLOCK_SAMPLE_BYTES);
		uint8_t etag = (block[sample_offset + 2U] >> 3) & 0x07U;

		if (etag > 3U) {
			return -EINVAL;
		}
	}

	if (info != NULL) {
		info->first_rtc_tick = ecg_get_u32_le(&block[ECG_BLOCK_FIRST_TICK_OFFSET]);
		info->first_sample_index = ecg_get_u32_le(&block[ECG_BLOCK_FIRST_INDEX_OFFSET]);
		info->sample_count = sample_count;
		info->flags = block[ECG_BLOCK_FLAGS_OFFSET];
	}

	return 0;
}

int msense_ecg_block_validate_continuity(const struct msense_ecg_block_info *previous,
					 const struct msense_ecg_block_info *current)
{
	if (previous == NULL || current == NULL) {
		return -EINVAL;
	}
	if ((current->flags & MSENSE_ECG_BLOCK_FLAG_DISCONTINUITY_BEFORE) != 0U) {
		return 0;
	}
	if (current->first_sample_index !=
	    previous->first_sample_index + previous->sample_count ||
	    current->first_rtc_tick != previous->first_rtc_tick + previous->sample_count) {
		return -EILSEQ;
	}

	return 0;
}

void msense_ecg_file_header_build(uint8_t *header, uint64_t session_id,
				  uint32_t chunk_index)
{
	uint32_t crc;

	if (header == NULL) {
		return;
	}

	memset(header, 0, MSENSE_ECG_FILE_HEADER_BYTES);
	memcpy(header, "ECF1", 4U);
	ecg_put_u16_le(&header[4], MSENSE_ECG_BLOCK_FORMAT_VERSION);
	ecg_put_u16_le(&header[6], 48U);
	ecg_put_u32_le(&header[8], 512U);
	ecg_put_u32_le(&header[12], 1U);
	ecg_put_u32_le(&header[16], 512U);
	ecg_put_u16_le(&header[20], MSENSE_ECG_BLOCK_BYTES);
	ecg_put_u16_le(&header[22], MSENSE_ECG_BLOCK_HEADER_BYTES);
	ecg_put_u16_le(&header[24], MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK);
	header[26] = MSENSE_ECG_BLOCK_SAMPLE_BYTES;
	header[27] = MSENSE_ECG_BLOCK_FORMAT_VERSION;
	ecg_put_u32_le(&header[28], MSENSE_ECG_FILE_FLAG_SHARED_CLOCK);
	ecg_put_u64_le(&header[32], session_id);
	ecg_put_u32_le(&header[44], chunk_index);
	crc = crc32_ieee(header, MSENSE_ECG_FILE_HEADER_BYTES);
	ecg_put_u32_le(&header[ECG_FILE_CRC_OFFSET], crc);
}

int msense_ecg_file_header_validate(uint8_t *header)
{
	if (header == NULL || memcmp(header, "ECF1", 4U) != 0 ||
	    ecg_get_u16_le(&header[4]) != MSENSE_ECG_BLOCK_FORMAT_VERSION ||
	    ecg_get_u16_le(&header[6]) != 48U || ecg_get_u32_le(&header[8]) != 512U ||
	    ecg_get_u32_le(&header[12]) != 1U || ecg_get_u32_le(&header[16]) != 512U ||
	    ecg_get_u16_le(&header[20]) != MSENSE_ECG_BLOCK_BYTES ||
	    ecg_get_u16_le(&header[22]) != MSENSE_ECG_BLOCK_HEADER_BYTES ||
	    ecg_get_u16_le(&header[24]) != MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK ||
	    header[26] != MSENSE_ECG_BLOCK_SAMPLE_BYTES ||
	    header[27] != MSENSE_ECG_BLOCK_FORMAT_VERSION ||
	    ecg_get_u32_le(&header[28]) != MSENSE_ECG_FILE_FLAG_SHARED_CLOCK ||
	    !ecg_all_zero(&header[48], MSENSE_ECG_FILE_HEADER_BYTES - 48U)) {
		return -EINVAL;
	}

	return ecg_validate_page_crc(header, ECG_FILE_CRC_OFFSET);
}

void msense_ecg_file_trailer_build(uint8_t *trailer, uint32_t valid_block_count,
				   uint64_t valid_sample_count,
				   uint32_t first_sample_index,
				   uint32_t next_sample_index, bool clean_close)
{
	uint32_t crc;

	if (trailer == NULL) {
		return;
	}

	memset(trailer, 0, MSENSE_ECG_FILE_TRAILER_BYTES);
	memcpy(trailer, "ECT1", 4U);
	ecg_put_u16_le(&trailer[4], MSENSE_ECG_BLOCK_FORMAT_VERSION);
	ecg_put_u16_le(&trailer[6], 40U);
	ecg_put_u32_le(&trailer[8], valid_block_count);
	ecg_put_u64_le(&trailer[12], valid_sample_count);
	ecg_put_u32_le(&trailer[20], valid_block_count * MSENSE_ECG_BLOCK_BYTES);
	if (valid_block_count != 0U) {
		ecg_put_u32_le(&trailer[24], first_sample_index);
		ecg_put_u32_le(&trailer[28], next_sample_index);
	}
	ecg_put_u32_le(&trailer[32], clean_close ?
			MSENSE_ECG_TRAILER_FLAG_CLEAN_CLOSE : 0U);
	crc = crc32_ieee(trailer, MSENSE_ECG_FILE_TRAILER_BYTES);
	ecg_put_u32_le(&trailer[ECG_TRAILER_CRC_OFFSET], crc);
}

int msense_ecg_file_trailer_validate(uint8_t *trailer)
{
	uint32_t valid_block_count;
	uint64_t valid_sample_count;
	uint32_t first_sample_index;
	uint32_t next_sample_index;
	uint32_t flags;

	if (trailer == NULL || memcmp(trailer, "ECT1", 4U) != 0 ||
	    ecg_get_u16_le(&trailer[4]) != MSENSE_ECG_BLOCK_FORMAT_VERSION ||
	    ecg_get_u16_le(&trailer[6]) != 40U ||
	    !ecg_all_zero(&trailer[40], MSENSE_ECG_FILE_TRAILER_BYTES - 40U)) {
		return -EINVAL;
	}

	valid_block_count = ecg_get_u32_le(&trailer[8]);
	valid_sample_count = ecg_get_u64_le(&trailer[12]);
	first_sample_index = ecg_get_u32_le(&trailer[24]);
	next_sample_index = ecg_get_u32_le(&trailer[28]);
	flags = ecg_get_u32_le(&trailer[32]);
	if (valid_block_count > MSENSE_ECG_FILE_DATA_BLOCKS ||
	    ecg_get_u32_le(&trailer[20]) != valid_block_count * MSENSE_ECG_BLOCK_BYTES ||
	    (flags & ~MSENSE_ECG_TRAILER_FLAG_CLEAN_CLOSE) != 0U ||
	    valid_sample_count > (uint64_t)MSENSE_ECG_FILE_DATA_BLOCKS *
					MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK) {
		return -EINVAL;
	}
	if (valid_block_count == 0U) {
		if (valid_sample_count != 0U || first_sample_index != 0U || next_sample_index != 0U) {
			return -EINVAL;
		}
	} else if (next_sample_index != first_sample_index + (uint32_t)valid_sample_count) {
		return -EINVAL;
	}

	return ecg_validate_page_crc(trailer, ECG_TRAILER_CRC_OFFSET);
}
