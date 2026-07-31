#ifndef ACCEL_RECORD_FORMAT_H_
#define ACCEL_RECORD_FORMAT_H_

#include <stddef.h>
#include <stdint.h>

#define ACCEL_RECORD_FORMAT_BLOCK_BYTES 4096U
#define ACCEL_RECORD_FORMAT_BLOCK_HEADER_BYTES 16U
#define ACCEL_RECORD_FORMAT_SAMPLE_BYTES 6U
#define ACCEL_RECORD_FORMAT_SAMPLES_PER_BLOCK 680U
#define ACCEL_RECORD_FORMAT_FILE_BYTES (4U * 1024U * 1024U)
#define ACCEL_RECORD_FORMAT_TRAILER_BYTES ACCEL_RECORD_FORMAT_BLOCK_BYTES
#define ACCEL_RECORD_FORMAT_TRAILER_OFFSET \
	(ACCEL_RECORD_FORMAT_FILE_BYTES - ACCEL_RECORD_FORMAT_TRAILER_BYTES)
#define ACCEL_RECORD_FORMAT_DATA_BYTES \
	(ACCEL_RECORD_FORMAT_TRAILER_OFFSET - ACCEL_RECORD_FORMAT_BLOCK_BYTES)
#define ACCEL_RECORD_FORMAT_FULL_BLOCKS_PER_FILE \
	(ACCEL_RECORD_FORMAT_DATA_BYTES / ACCEL_RECORD_FORMAT_BLOCK_BYTES)

void accel_record_format_build_header(uint8_t *header);
void accel_record_format_build_trailer(uint8_t *trailer,
					       uint32_t valid_data_bytes);
void accel_record_format_store_fifo_sample(uint8_t *block, size_t sample_index,
					   const uint8_t *fifo_sample);
size_t accel_record_format_finalize_block(uint8_t *block, size_t sample_count,
					  uint32_t first_sample_sequence,
					  uint32_t reserved_timer_output);

#endif /* ACCEL_RECORD_FORMAT_H_ */
