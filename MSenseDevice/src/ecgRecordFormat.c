#include "ecgRecordFormat.h"

#include <stddef.h>

#define ECG_RECORD_FORMAT_CRC_POLY 0x07u

static uint8_t ecg_record_format_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			if ((crc & 0x80u) != 0u) {
				crc = (uint8_t)((crc << 1) ^ ECG_RECORD_FORMAT_CRC_POLY);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

void ecg_record_format_build_sample_frame(
	uint8_t frame[ECG_RECORD_FORMAT_FRAME_BYTES], uint8_t etag, uint8_t ptag,
	uint32_t raw, uint32_t rtc_tick)
{
	frame[0] = ECG_RECORD_FORMAT_SYNC0;
	frame[1] = ECG_RECORD_FORMAT_SYNC1;
	frame[2] = ECG_RECORD_FORMAT_TYPE_SAMPLE;
	frame[3] = (uint8_t)((etag & 0x07u) | ((ptag & 0x07u) << 3));
	frame[4] = (uint8_t)(rtc_tick & 0xFFu);
	frame[5] = (uint8_t)((rtc_tick >> 8) & 0xFFu);
	frame[6] = (uint8_t)((rtc_tick >> 16) & 0xFFu);
	frame[7] = (uint8_t)((rtc_tick >> 24) & 0xFFu);
	frame[8] = (uint8_t)((raw >> 16) & 0xFFu);
	frame[9] = (uint8_t)((raw >> 8) & 0xFFu);
	frame[10] = (uint8_t)(raw & 0xFFu);
	frame[11] = ecg_record_format_crc8(&frame[2], 9U);
}
