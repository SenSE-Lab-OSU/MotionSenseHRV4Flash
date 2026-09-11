#ifndef ICM20948_ACCEL_H_
#define ICM20948_ACCEL_H_

#include <stddef.h>
#include <stdint.h>

struct icm20948_accel_sample {
	int16_t x;
	int16_t y;
	int16_t z;
	int64_t timestamp_ms;
	uint32_t sequence;
};

typedef int (*icm20948_accel_fifo_consumer_t)(const uint8_t *fifo_data,
					       size_t fifo_bytes,
					       void *context);
typedef void (*icm20948_accel_fifo_fault_handler_t)(void *context);

enum icm20948_accel_fifo_fault_reason {
	ICM20948_ACCEL_FIFO_FAULT_NONE = 0,
	ICM20948_ACCEL_FIFO_FAULT_COUNT,
	ICM20948_ACCEL_FIFO_FAULT_OVERFLOW,
	ICM20948_ACCEL_FIFO_FAULT_TRANSPORT,
};

struct icm20948_accel_fifo_diagnostics {
	uint32_t recovery_count;
	int32_t first_fault_error;
	uint16_t first_count_bytes;
	uint16_t last_count_bytes;
	uint8_t first_fault_reason;
	uint8_t count_read_count;
	uint8_t first_fault_write_complete;
};

extern struct icm20948_accel_fifo_diagnostics icm20948_accel_fifo_diagnostics;

int icm20948_accel_init(void);
int icm20948_accel_set_fifo_consumer(icm20948_accel_fifo_consumer_t consumer,
					     void *context);
int icm20948_accel_set_fifo_fault_handler(
	icm20948_accel_fifo_fault_handler_t handler, void *context);
int icm20948_accel_start(void);
int icm20948_accel_stop(void);
int icm20948_accel_get_latest(struct icm20948_accel_sample *sample);

#endif /* ICM20948_ACCEL_H_ */
