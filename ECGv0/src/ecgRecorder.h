#ifndef ECG_RECORDER_H
#define ECG_RECORDER_H

#include <stdbool.h>

typedef void (*ecg_recorder_fault_handler_t)(void *context);

int ecg_recorder_start(void);
int ecg_recorder_stop(void);
bool ecg_recorder_shutdown_confirmed(void);
void ecg_recorder_set_fault_handler(ecg_recorder_fault_handler_t handler,
					     void *context);

#endif /* ECG_RECORDER_H */
