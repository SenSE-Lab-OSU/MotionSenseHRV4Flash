#include "accelRecordFormat.h"

#include <string.h>

#include <zephyr/sys/crc.h>

static void accel_record_format_put_u16_le(uint8_t *dst, uint16_t value)
{
	dst[0] = (uint8_t)(value & 0xffU);
	dst[1] = (uint8_t)(value >> 8);
}

static void accel_record_format_put_u32_le(uint8_t *dst, uint32_t value)
{
	dst[0] = (uint8_t)(value & 0xffU);
	dst[1] = (uint8_t)((value >> 8) & 0xffU);
	dst[2] = (uint8_t)((value >> 16) & 0xffU);
	dst[3] = (uint8_t)((value >> 24) & 0xffU);
}

void accel_record_format_build_header(uint8_t *header)
{
	uint32_t crc;

	memset(header, 0, ACCEL_RECORD_FORMAT_BLOCK_BYTES);
	header[0] = 'A';
	header[1] = 'C';
	header[2] = 'F';
	header[3] = '3';
	accel_record_format_put_u16_le(&header[4], ACCEL_RECORD_FORMAT_VERSION);
	accel_record_format_put_u16_le(&header[6],
				      ACCEL_RECORD_FORMAT_SAMPLE_FORMAT_FSYNC_X_LSB);
	accel_record_format_put_u32_le(&header[8], 1125U);
	accel_record_format_put_u32_le(&header[12], 2U);
	accel_record_format_put_u16_le(&header[16], 2U);
	accel_record_format_put_u16_le(&header[18], 16384U);
	accel_record_format_put_u32_le(&header[24],
				      ACCEL_RECORD_FORMAT_RTC_TICKS_PER_SECOND);
	accel_record_format_put_u32_le(&header[28],
				      ACCEL_RECORD_FORMAT_FSYNC_EDGE_INTERVAL_TICKS);
	header[32] = ACCEL_RECORD_FORMAT_FSYNC_AXIS_X;
	header[33] = ACCEL_RECORD_FORMAT_FSYNC_BIT;
	header[34] = ACCEL_RECORD_FORMAT_TIMESTAMP_ALGORITHM;
	header[35] = ACCEL_RECORD_FORMAT_TIMESTAMP_WINDOW;

	crc = crc32_ieee(header, ACCEL_RECORD_FORMAT_BLOCK_BYTES);
	accel_record_format_put_u32_le(&header[20], crc);
}

void accel_record_format_build_trailer(uint8_t *trailer,
					       uint32_t valid_data_bytes)
{
	uint32_t crc;

	memset(trailer, 0, ACCEL_RECORD_FORMAT_TRAILER_BYTES);
	trailer[0] = 'A';
	trailer[1] = 'C';
	trailer[2] = 'T';
	trailer[3] = '2';
	accel_record_format_put_u32_le(&trailer[4], valid_data_bytes);
	crc = crc32_ieee(trailer, ACCEL_RECORD_FORMAT_TRAILER_BYTES);
	accel_record_format_put_u32_le(&trailer[8], crc);
}

void accel_record_format_store_fifo_sample(uint8_t *block, size_t sample_index,
					   const uint8_t *fifo_sample)
{
	size_t payload_offset = ACCEL_RECORD_FORMAT_BLOCK_HEADER_BYTES +
		(sample_index * ACCEL_RECORD_FORMAT_SAMPLE_BYTES);

	block[payload_offset] = fifo_sample[1];
	block[payload_offset + 1U] = fifo_sample[0];
	block[payload_offset + 2U] = fifo_sample[3];
	block[payload_offset + 3U] = fifo_sample[2];
	block[payload_offset + 4U] = fifo_sample[5];
	block[payload_offset + 5U] = fifo_sample[4];
}

size_t accel_record_format_finalize_block(uint8_t *block, size_t sample_count,
					  uint32_t first_sample_sequence,
					  uint32_t reserved_timer_output)
{
	size_t write_length = ACCEL_RECORD_FORMAT_BLOCK_HEADER_BYTES +
		(sample_count * ACCEL_RECORD_FORMAT_SAMPLE_BYTES);
	uint32_t crc;

	block[0] = 'A';
	block[1] = 'C';
	block[2] = 'B';
	block[3] = '1';
	accel_record_format_put_u32_le(&block[4], reserved_timer_output);
	accel_record_format_put_u32_le(&block[8], first_sample_sequence);
	memset(&block[12], 0, 4U);
	crc = crc32_ieee(block, write_length);
	accel_record_format_put_u32_le(&block[12], crc);

	return write_length;
}
