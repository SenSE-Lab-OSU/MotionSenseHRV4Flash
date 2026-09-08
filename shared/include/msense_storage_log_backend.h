#ifndef MSENSE_STORAGE_LOG_BACKEND_H_
#define MSENSE_STORAGE_LOG_BACKEND_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Application policy hooks. The common backend owns Zephyr logging mechanics;
 * products own collection state and their record-stream choice. */
bool msense_storage_log_write_enabled(void);
int msense_storage_log_append(const uint8_t *data, size_t length);
void msense_storage_log_panic(void);

#endif /* MSENSE_STORAGE_LOG_BACKEND_H_ */
