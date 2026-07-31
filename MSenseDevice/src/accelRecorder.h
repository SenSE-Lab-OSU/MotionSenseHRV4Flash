#ifndef ACCEL_RECORDER_H_
#define ACCEL_RECORDER_H_

#include <stddef.h>
#include <stdint.h>

typedef void (*accel_recorder_fault_handler_t)(void *context);

int accel_recorder_start(uint64_t session_id);
int accel_recorder_consume_fifo(const uint8_t *fifo_data, size_t fifo_bytes,
				void *context);
int accel_recorder_stop(void);
int accel_recorder_abort(void);
void accel_recorder_set_fault_handler(accel_recorder_fault_handler_t handler,
					  void *context);

#endif /* ACCEL_RECORDER_H_ */
