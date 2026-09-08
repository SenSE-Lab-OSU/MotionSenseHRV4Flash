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

#define MSENSE_ECG_BLOCK_FORMAT_VERSION 2U
#define MSENSE_ECG_BLOCK_BYTES 4096U
#define MSENSE_ECG_BLOCK_HEADER_BYTES 16U
#define MSENSE_ECG_BLOCK_SAMPLE_BYTES 3U
#define MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK 1358U
#define MSENSE_ECG_BLOCK_PAYLOAD_BYTES 4074U

#define MSENSE_ECG_FILE_BYTES (4U * 1024U * 1024U)
#define MSENSE_ECG_FILE_HEADER_BYTES MSENSE_ECG_BLOCK_BYTES
#define MSENSE_ECG_FILE_DATA_BLOCKS 1023U

struct msense_ecg_block_info {
	uint32_t first_rtc_tick;
	uint32_t first_sample_index;
};

void msense_ecg_block_begin(uint8_t *block, uint32_t first_rtc_tick,
			    uint32_t first_sample_index);
int msense_ecg_block_append_sample(uint8_t *block, uint16_t sample_count,
				   uint32_t raw24);
int msense_ecg_block_finalize(uint8_t *block, uint16_t sample_count);
int msense_ecg_block_validate(uint8_t *block, struct msense_ecg_block_info *info);
int msense_ecg_block_validate_continuity(const struct msense_ecg_block_info *previous,
					 const struct msense_ecg_block_info *current);

void msense_ecg_file_header_build(uint8_t *header, uint64_t recording_id,
				  uint32_t chunk_index);
int msense_ecg_file_header_validate(uint8_t *header);

#endif /* MSENSE_ECG_BLOCK_FORMAT_H_ */
