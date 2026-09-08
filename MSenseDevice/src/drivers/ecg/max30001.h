#ifndef MAX30001_H
#define MAX30001_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAX30001_ECG_FIFO_MAX_SAMPLES 32

enum max30001_ecg_etag {
	MAX30001_ECG_ETAG_VALID = 0,
	MAX30001_ECG_ETAG_FAST = 1,
	MAX30001_ECG_ETAG_VALID_EOF = 2,
	MAX30001_ECG_ETAG_FAST_EOF = 3,
	MAX30001_ECG_ETAG_EMPTY = 6,
	MAX30001_ECG_ETAG_OVERFLOW = 7,
};

struct max30001_ecg_sample {
	uint32_t raw;
	uint8_t etag;
	uint8_t ptag;
	bool data_valid;
	bool time_valid;
	bool eof;
};

/*
 * Probes the MAX30001 SPI interface by issuing a NO_OP transaction followed by
 * an INFO register read. Returns 0 when the INFO fixed pattern is present.
 *
 * If info is non-NULL, the raw 24-bit INFO register value is stored there.
 */
int max30001_probe(uint32_t *info);

int max30001_ecg_init_512(void);
int max30001_ecg_start(void);
/* Masks the self-clearing 512 Hz SAMP pulse on INT2B after initial anchoring. */
int max30001_ecg_disable_samp_interrupt(void);
int max30001_ecg_stop(void);
int max30001_ecg_read_fifo(struct max30001_ecg_sample *samples,
			   size_t max_samples,
			   size_t *sample_count);

#endif /* MAX30001_H */
