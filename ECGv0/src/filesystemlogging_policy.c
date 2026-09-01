#include "BLEService.h"
#include "msense_storage_log_backend.h"
#include "zephyrfilesystem.h"

#include <zephyr/sys/atomic.h>

K_MUTEX_DEFINE(ecg_filesystem_log_callback_lock);
static atomic_t ecg_filesystem_log_writes_enabled = ATOMIC_INIT(0);

void ecg_filesystem_log_enable(void)
{
	atomic_set(&ecg_filesystem_log_writes_enabled, 1);
}

void ecg_filesystem_log_disable(void)
{
	atomic_clear(&ecg_filesystem_log_writes_enabled);
}

void ecg_filesystem_log_disable_and_wait(void)
{
	/* Close the gate before joining a callback that passed its first check. */
	ecg_filesystem_log_disable();
	k_mutex_lock(&ecg_filesystem_log_callback_lock, K_FOREVER);
	k_mutex_unlock(&ecg_filesystem_log_callback_lock);
}

bool msense_storage_log_write_enabled(void)
{
	return (atomic_get(&ecg_filesystem_log_writes_enabled) != 0) &&
	       file_system_ready && collecting_data && !battery_low && !reset_lock;
}

int msense_storage_log_append(const uint8_t *data, size_t length)
{
	int ret;

	k_mutex_lock(&ecg_filesystem_log_callback_lock, K_FOREVER);
	if (!msense_storage_log_write_enabled()) {
		k_mutex_unlock(&ecg_filesystem_log_callback_lock);
		return (int)length;
	}

	ret = store_data(data, length, customlog);
	k_mutex_unlock(&ecg_filesystem_log_callback_lock);
	if (ret != 0) {
		/* store_data() latched the fault; consume this record to avoid retries. */
		return (int)length;
	}
	return (int)length;
}

void msense_storage_log_panic(void)
{
	panic_single_thread = true;
	/* A log callback must not close FatFS or wait on its own callback lock. */
	ecg_filesystem_log_disable();
	request_ecg_storage_fault();
}
