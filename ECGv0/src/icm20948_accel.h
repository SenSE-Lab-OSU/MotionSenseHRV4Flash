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

int icm20948_accel_init(void);
int icm20948_accel_set_fifo_consumer(icm20948_accel_fifo_consumer_t consumer,
					     void *context);
int icm20948_accel_start(void);
int icm20948_accel_stop(void);
int icm20948_accel_get_latest(struct icm20948_accel_sample *sample);

#endif /* ICM20948_ACCEL_H_ */
