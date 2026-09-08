#ifndef ECG_RECORD_FORMAT_H
#define ECG_RECORD_FORMAT_H

#include <stdint.h>

#define ECG_RECORD_FORMAT_FRAME_BYTES 12U
#define ECG_RECORD_FORMAT_SYNC0 0xA5u
#define ECG_RECORD_FORMAT_SYNC1 0xECu
#define ECG_RECORD_FORMAT_TYPE_SAMPLE 0x01u

/**
 * @brief Build one on-flash MAX30001 ECG sample frame.
 *
 * Bytes 4 through 7 carry the collection-local 512 Hz RTC0 tick associated
 * with the ECG FIFO-output sample. The frame stays compatible in size and
 * record type with the earlier sequence-number layout; callers must use
 * firmware/session provenance to distinguish the two semantics.
 */
void ecg_record_format_build_sample_frame(
	uint8_t frame[ECG_RECORD_FORMAT_FRAME_BYTES], uint8_t etag, uint8_t ptag,
	uint32_t raw, uint32_t rtc_tick);

#endif /* ECG_RECORD_FORMAT_H */
