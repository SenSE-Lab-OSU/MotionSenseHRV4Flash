#ifndef ICM20948_FIFO_COUNT_H_
#define ICM20948_FIFO_COUNT_H_

#include <stdint.h>

#define ICM20948_FIFO_COUNT_SAMPLE_BYTES 6U
#define ICM20948_FIFO_COUNT_CAPACITY_BYTES 4096U

typedef int (*icm20948_fifo_count_read_t)(void *context, uint16_t *count_bytes);
typedef void (*icm20948_fifo_count_wait_t)(void *context, uint32_t wait_us);

struct icm20948_fifo_count_observation {
	uint16_t initial_count_bytes;
	uint16_t last_count_bytes;
	uint8_t read_count;
	int read_error;
};

int icm20948_fifo_count_read_stable(icm20948_fifo_count_read_t read_count,
				    icm20948_fifo_count_wait_t wait,
				    void *context, uint16_t *count_bytes,
				    struct icm20948_fifo_count_observation *observation);

#endif /* ICM20948_FIFO_COUNT_H_ */
