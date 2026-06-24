
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_backend_std.h>
#include "zephyrfilesystem.h"
#include "BLEService.h"

#ifdef CONFIG_LOG_BACKEND_FS_BUFFER


#define MAX_PATH_LEN 256
#define MAX_FLASH_WRITE_SIZE 512
#define LOG_PREFIX_LEN (sizeof(CONFIG_LOG_BACKEND_FS_FILE_PREFIX) - 1)
#define MAX_FILE_NUMERAL 9999
#define FILE_NUMERAL_LEN 4

enum backend_fs_state {
	BACKEND_FS_NOT_INITIALIZED = 0,
	BACKEND_FS_CORRUPTED,
	BACKEND_FS_OK
};

int debug_messages = 0;

static uint32_t log_format_current = 0;




int write_log_to_file(uint8_t *data, size_t length, void *ctx)
{
	debug_messages++;
	if (file_system_ready && !battery_low && !reset_lock) {
		store_data(data, length, customlog);
		
	}
	return length;
}



BUILD_ASSERT(!IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE),
	     "Immediate logging is not supported by LOG FS backend.");

#ifndef CONFIG_LOG_BACKEND_FS_TESTSUITE

static uint8_t __aligned(4) buf[MAX_FLASH_WRITE_SIZE];
LOG_OUTPUT_DEFINE(log_output, write_log_to_file, buf, MAX_FLASH_WRITE_SIZE);

static void log_backend_fs_init(const struct log_backend *const backend)
{

}

static void panic(struct log_backend const *const backend)
{
	panic_single_thread = true;
	// In case of panic, flush any remaining log data to the file.
	log_backend_std_panic(&log_output);
	//flush_data_buffer(customlog);
	// after the messages have logged, close files.
	close_all_files();
	log_backend_deactivate(backend);
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FS_OUTPUT_DICTIONARY)) {
		log_dict_output_dropped_process(&log_output, cnt);
	} else {
		log_backend_std_dropped(&log_output, cnt);
	}
}

static void process(const struct log_backend *const backend,
		union log_msg_generic *msg)
{
	uint32_t flags = log_backend_std_get_flags();

	log_format_func_t log_output_func = log_format_func_t_get(log_format_current);

	log_output_func(&log_output, &msg->log, flags);
}

static int format_set(const struct log_backend *const backend, uint32_t log_type)
{
	log_format_current = log_type;
	return 0;
}

static const struct log_backend_api log_backend_fs_buff_api = {
	.process = process,
	.panic = panic,
	.init = log_backend_fs_init,
	.dropped = dropped,
	.format_set = format_set,
};

LOG_BACKEND_DEFINE(log_backend_fs_buff, log_backend_fs_buff_api,
		   IS_ENABLED(CONFIG_LOG_BACKEND_FS_BUFFER));
#endif

#endif
