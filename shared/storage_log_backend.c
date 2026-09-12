
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_output_dict.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/sys/atomic.h>
#include "msense_storage_log_backend.h"


#ifdef CONFIG_LOG_BACKEND_FS_BUFFER

LOG_MODULE_REGISTER(storage_log_backend, LOG_LEVEL_INF);

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

static uint32_t log_format_current = 0;
static atomic_t drain_requested = ATOMIC_INIT(0);
K_SEM_DEFINE(drain_complete, 0, 1);

int write_log_to_file(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);
	if (!msense_storage_log_write_enabled()) {
		/* Discarded bytes must be reported as consumed so log_output_write()
		 * does not retry this callback indefinitely. */
		return (int)length;
	}
	return msense_storage_log_append(data, length);
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
	// In case of panic, flush any remaining log data to the file.
	log_backend_std_panic(&log_output);
	msense_storage_log_panic();
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

static void notify(const struct log_backend *const backend,
		   enum log_backend_evt event, union log_backend_evt_arg *arg)
{
	ARG_UNUSED(backend);
	ARG_UNUSED(arg);

	if (event == LOG_BACKEND_EVT_PROCESS_THREAD_DONE &&
	    atomic_get(&drain_requested) != 0 && log_buffered_cnt() == 0U) {
		atomic_clear(&drain_requested);
		k_sem_give(&drain_complete);
	}
}

int msense_storage_log_drain(void)
{
	int ret;

	k_sem_reset(&drain_complete);
	/* Keep the logger thread from processing between arming the barrier and
	 * enqueuing its marker. ISR log producers may still enqueue safely. */
	k_sched_lock();
	atomic_set(&drain_requested, 1);
	LOG_INF("Closing storage log");
	/* The SDK emits PROCESS_THREAD_DONE only after a processing pass that
	 * observed another pending message. Two markers guarantee that pass. */
	LOG_INF("Storage log end");
	log_thread_trigger();
	k_sched_unlock();

	/* Covers the configured 8-second logger thread startup delay. */
	ret = k_sem_take(&drain_complete, K_SECONDS(10));
	if (ret != 0) {
		atomic_clear(&drain_requested);
	}
	return ret;
}

static int format_set(const struct log_backend *const backend, uint32_t log_type)
{
	log_format_current = log_type;
	return 0;
}

static const struct log_backend_api log_backend_fs_buff_api = {
	.process = process,
	.notify = notify,
	.panic = panic,
	.init = log_backend_fs_init,
	.dropped = dropped,
	.format_set = format_set,
};

LOG_BACKEND_DEFINE(log_backend_fs_buff, log_backend_fs_buff_api,
		   IS_ENABLED(CONFIG_LOG_BACKEND_FS_BUFFER));
#endif

#endif
