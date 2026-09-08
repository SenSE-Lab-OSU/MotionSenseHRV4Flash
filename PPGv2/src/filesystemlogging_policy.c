#include "BLEService.h"
#include "msense_storage_log_backend.h"
#include "zephyrfilesystem.h"

#include <zephyr/sys/atomic.h>

K_MUTEX_DEFINE(ppg_filesystem_log_callback_lock);
static atomic_t ppg_filesystem_log_writes_enabled = ATOMIC_INIT(0);

void ppg_filesystem_log_enable(void)
{
	atomic_set(&ppg_filesystem_log_writes_enabled, 1);
}

void ppg_filesystem_log_disable_and_wait(void)
{
	/* Close the gate before joining a callback that passed its first check. */
	atomic_clear(&ppg_filesystem_log_writes_enabled);
	k_mutex_lock(&ppg_filesystem_log_callback_lock, K_FOREVER);
	k_mutex_unlock(&ppg_filesystem_log_callback_lock);
}

bool msense_storage_log_write_enabled(void)
{
	return (atomic_get(&ppg_filesystem_log_writes_enabled) != 0) &&
	       file_system_ready && !battery_low && !reset_lock && collecting_data;
}

int msense_storage_log_append(const uint8_t *data, size_t length)
{
	int ret;

	k_mutex_lock(&ppg_filesystem_log_callback_lock, K_FOREVER);
	if (!msense_storage_log_write_enabled()) {
		k_mutex_unlock(&ppg_filesystem_log_callback_lock);
		return (int)length;
	}

	ret = store_data(data, length, customlog);
	k_mutex_unlock(&ppg_filesystem_log_callback_lock);
	if (ret != 0) {
		/* store_data() latched the fault; consume this record to avoid retries. */
		return (int)length;
	}
	return (int)length;
}

void msense_storage_log_panic(void)
{
	panic_single_thread = true;
	/* A log callback cannot safely close FatFS; defer to the fault owner. */
	ppg_collection_latch_storage_fault();
}
