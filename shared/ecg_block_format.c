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
#define ECG_BLOCK_CRC_OFFSET 12U
#define ECG_BLOCK_PAYLOAD_OFFSET 16U
#define ECG_BLOCK_RESERVED_OFFSET 4090U
#define ECG_BLOCK_RESERVED_BYTES 6U

#define ECG_FILE_MAGIC_OFFSET 0U
#define ECG_FILE_CHUNK_INDEX_OFFSET 4U
#define ECG_FILE_RECORDING_ID_OFFSET 8U
#define ECG_FILE_CRC_OFFSET 16U
#define ECG_FILE_RESERVED_OFFSET 20U

static uint32_t ecg_get_u32_le(const uint8_t *source)
{
	return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
	       ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
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

/* The CRC field is logically zero while its page CRC is calculated. */
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
			    uint32_t first_sample_index)
{
	if (block == NULL) {
		return;
	}

	memset(block, 0, MSENSE_ECG_BLOCK_BYTES);
	memcpy(&block[ECG_BLOCK_MAGIC_OFFSET], "ECB2", 4U);
	ecg_put_u32_le(&block[ECG_BLOCK_FIRST_TICK_OFFSET], first_rtc_tick);
	ecg_put_u32_le(&block[ECG_BLOCK_FIRST_INDEX_OFFSET], first_sample_index);
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

	if (block == NULL || sample_count != MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK ||
	    memcmp(&block[ECG_BLOCK_MAGIC_OFFSET], "ECB2", 4U) != 0 ||
	    !ecg_all_zero(&block[ECG_BLOCK_RESERVED_OFFSET], ECG_BLOCK_RESERVED_BYTES)) {
		return -EINVAL;
	}

	memset(&block[ECG_BLOCK_CRC_OFFSET], 0, sizeof(uint32_t));
	crc = crc32_ieee(block, MSENSE_ECG_BLOCK_BYTES);
	ecg_put_u32_le(&block[ECG_BLOCK_CRC_OFFSET], crc);
	return 0;
}

int msense_ecg_block_validate(uint8_t *block, struct msense_ecg_block_info *info)
{
	size_t index;
	int ret;

	if (block == NULL || memcmp(&block[ECG_BLOCK_MAGIC_OFFSET], "ECB2", 4U) != 0 ||
	    !ecg_all_zero(&block[ECG_BLOCK_RESERVED_OFFSET], ECG_BLOCK_RESERVED_BYTES)) {
		return -EINVAL;
	}

	ret = ecg_validate_page_crc(block, ECG_BLOCK_CRC_OFFSET);
	if (ret != 0) {
		return ret;
	}

	for (index = 0U; index < MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK; index++) {
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
	}

	return 0;
}

int msense_ecg_block_validate_continuity(const struct msense_ecg_block_info *previous,
					 const struct msense_ecg_block_info *current)
{
	if (previous == NULL || current == NULL) {
		return -EINVAL;
	}
	if (current->first_sample_index != previous->first_sample_index +
					MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK ||
	    current->first_rtc_tick != previous->first_rtc_tick +
					MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK) {
		return -EILSEQ;
	}

	return 0;
}

void msense_ecg_file_header_build(uint8_t *header, uint64_t recording_id,
				  uint32_t chunk_index)
{
	uint32_t crc;

	if (header == NULL) {
		return;
	}

	memset(header, 0, MSENSE_ECG_FILE_HEADER_BYTES);
	memcpy(&header[ECG_FILE_MAGIC_OFFSET], "ECF2", 4U);
	ecg_put_u32_le(&header[ECG_FILE_CHUNK_INDEX_OFFSET], chunk_index);
	ecg_put_u64_le(&header[ECG_FILE_RECORDING_ID_OFFSET], recording_id);
	crc = crc32_ieee(header, MSENSE_ECG_FILE_HEADER_BYTES);
	ecg_put_u32_le(&header[ECG_FILE_CRC_OFFSET], crc);
}

int msense_ecg_file_header_validate(uint8_t *header)
{
	if (header == NULL || memcmp(&header[ECG_FILE_MAGIC_OFFSET], "ECF2", 4U) != 0 ||
	    !ecg_all_zero(&header[ECG_FILE_RESERVED_OFFSET],
			  MSENSE_ECG_FILE_HEADER_BYTES - ECG_FILE_RESERVED_OFFSET)) {
		return -EINVAL;
	}

	return ecg_validate_page_crc(header, ECG_FILE_CRC_OFFSET);
}
