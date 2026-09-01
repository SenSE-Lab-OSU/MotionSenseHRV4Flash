#include "BLEService.h"
#include "msense_storage_log_backend.h"
#include "zephyrfilesystem.h"

bool msense_storage_log_write_enabled(void)
{
	return file_system_ready && !battery_low && !reset_lock && collecting_data;
}

int msense_storage_log_append(const uint8_t *data, size_t length)
{
	store_data(data, length, customlog);
	return (int)length;
}

void msense_storage_log_panic(void)
{
	panic_single_thread = true;
	(void)close_all_files();
}
