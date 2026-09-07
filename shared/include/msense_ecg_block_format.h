/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_ECG_BLOCK_FORMAT_H_
#define MSENSE_ECG_BLOCK_FORMAT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MSENSE_ECG_BLOCK_FORMAT_VERSION 1U
#define MSENSE_ECG_BLOCK_BYTES 4096U
#define MSENSE_ECG_BLOCK_HEADER_BYTES 20U
#define MSENSE_ECG_BLOCK_SAMPLE_BYTES 3U
#define MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK 1358U
#define MSENSE_ECG_BLOCK_PAYLOAD_BYTES 4074U

#define MSENSE_ECG_FILE_BYTES (4U * 1024U * 1024U)
#define MSENSE_ECG_FILE_HEADER_BYTES MSENSE_ECG_BLOCK_BYTES
#define MSENSE_ECG_FILE_TRAILER_BYTES MSENSE_ECG_BLOCK_BYTES
#define MSENSE_ECG_FILE_TRAILER_OFFSET \
	(MSENSE_ECG_FILE_BYTES - MSENSE_ECG_FILE_TRAILER_BYTES)
#define MSENSE_ECG_FILE_DATA_BLOCKS 1022U

#define MSENSE_ECG_BLOCK_FLAG_DISCONTINUITY_BEFORE 0x01U
#define MSENSE_ECG_FILE_FLAG_SHARED_CLOCK 0x00000001U
#define MSENSE_ECG_TRAILER_FLAG_CLEAN_CLOSE 0x00000001U

struct msense_ecg_block_info {
	uint32_t first_rtc_tick;
	uint32_t first_sample_index;
	uint16_t sample_count;
	uint8_t flags;
};

void msense_ecg_block_begin(uint8_t *block, uint32_t first_rtc_tick,
			    uint32_t first_sample_index, uint8_t flags);
int msense_ecg_block_append_sample(uint8_t *block, uint16_t sample_count,
				   uint32_t raw24);
int msense_ecg_block_finalize(uint8_t *block, uint16_t sample_count);
int msense_ecg_block_validate(uint8_t *block, struct msense_ecg_block_info *info);
int msense_ecg_block_validate_continuity(const struct msense_ecg_block_info *previous,
					 const struct msense_ecg_block_info *current);

void msense_ecg_file_header_build(uint8_t *header, uint64_t session_id,
				  uint32_t chunk_index);
int msense_ecg_file_header_validate(uint8_t *header);
void msense_ecg_file_trailer_build(uint8_t *trailer, uint32_t valid_block_count,
				   uint64_t valid_sample_count,
				   uint32_t first_sample_index,
				   uint32_t next_sample_index, bool clean_close);
int msense_ecg_file_trailer_validate(uint8_t *trailer);

#endif /* MSENSE_ECG_BLOCK_FORMAT_H_ */
