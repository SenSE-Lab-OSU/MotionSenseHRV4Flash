
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <nrfx_qspi.h>
#include <zephyr/logging/log.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "BLEService.h"
#include "accelRecorder.h"
#include "msense_git_metadata.h"
#include "msense_uuid_file.h"
#include "zephyrfilesystem.h"


LOG_MODULE_REGISTER(zephyrfilesystem, 3);

#if CONFIG_DISK_DRIVER_FLASH
#include <zephyr/storage/flash_map.h>
#endif

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "spi_nand.h"
#include "nand_disk.h"
#endif

#if CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#if FF_FS_TINY != 0
#error "Raw NAND recording requires one FatFs sector buffer per open FIL"
#endif
#if FF_MAX_SS != 4096
#error "Raw NAND recording requires 4096-byte FatFs sectors"
#endif
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(PM_LITTLEFS_STORAGE_NAME)
#endif

#if CONFIG_FILE_SYSTEM_LITTLEFS
#include <zephyr/fs/littlefs.h>
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);

#define STORAGE_PARTITION		storage_partition
#define STORAGE_PARTITION_ID		FIXED_PARTITION_ID(STORAGE_PARTITION)
#endif

bool file_system_ready;

bool security_lock;

bool panic_single_thread;

#define MAX_BUFFER_SIZE 9000
#define UUID_CONTENTS_MAX_SIZE 640U
#define STORAGE_SLOW_WRITE_LOG_THRESHOLD_MS 50
#define TEST_FILE_WRITE_BYTES (4096U * 2U)
#define TEST_FILE_PREALLOCATED_BYTES RECORDING_FILE_BYTES
#define TEST_FILE_WRITES_PER_PREALLOCATION \
	(TEST_FILE_PREALLOCATED_BYTES / TEST_FILE_WRITE_BYTES)

//#undef GET_FATTIME
//#define GET_FATTIME() (DWORD)get_current_unix_time()



struct k_work_q my_work_q;




// external globals
uint8_t storage_percent_full;

int upload_timeout_errors;

bool reset_lock;

uint64_t last_time_update_sent;

uint64_t set_date_time = 0;

int patient_num = 0;

typedef struct data_upload_buffer {
	char data_upload_buffer[MAX_BUFFER_SIZE];
	size_t current_size;
} data_upload_buffer;

// internally linked globals
static struct fs_mount_t fs_mnt;
static bool filesystem_mounted;
static uint8_t filesystem_scratch[FILESYSTEM_SCRATCH_BYTES] __aligned(4);
static int64_t logger_write_timer;
static struct k_work logger_control_work;
static struct k_work logger_prepare_work;
static struct k_sem logger_control_done;
static bool logger_control_initialized;
static bool logger_started;
static bool logger_next_prepared;
static bool logger_prepare_requested;
static int logger_control_result;
static char logger_next_path[96];
static uint32_t logger_dropped_buffers;
static size_t logger_logical_bytes;
static bool logger_file_open;
static bool logger_file_retired;
static bool logger_retired_handle_live;
static bool logger_write_work_initialized;
static void logger_write_work_handler(struct k_work *item);

enum logger_control_operation {
	LOGGER_CONTROL_START,
	LOGGER_CONTROL_STOP,
};

static enum logger_control_operation logger_control_operation;

static void filesystem_latch_fault(void)
{
	request_ecg_storage_fault();
}



typedef struct LoggerFile {
	size_t write_size;
	int current_writes;
	char file_name[96];
	struct fs_file_t self_file;
	bool switch_buffer;
	data_upload_buffer buffer1;
	data_upload_buffer buffer2;
} LoggerFile;

typedef struct logger_write_work {
	const void *address;
	size_t size;
	int packet_num;
	bool in_use;
	struct k_work work;
} logger_write_work;

static LoggerFile log_file = {
	.write_size = 8192,
};
static logger_write_work logger_work_item;
static int logger_packet_number;
static int logger_last_packet_number_processed;

#define ECG_FILE_FORMAT "ECB2 4096-byte MAX30001 ECG blocks with CRC-32"

int filesystem_make_recording_path(char *path, size_t path_size,
				   const char *stream_prefix,
				   uint64_t collection_id)
{
	return filesystem_make_recording_chunk_path(path, path_size, stream_prefix,
						    collection_id, 0U);
}

int filesystem_make_recording_chunk_path(char *path, size_t path_size,
					 const char *stream_prefix,
					 uint64_t collection_id,
					 uint32_t chunk_index)
{
	int written;

	if ((path == NULL) || (path_size == 0U) || (stream_prefix == NULL) ||
	    !file_system_ready) {
		return -EINVAL;
	}

	if (patient_num != 0) {
		written = snprintf(path, path_size, "%s/%d%s%llu_%04lu.bin",
				   fs_mnt.mnt_point, patient_num, stream_prefix,
				   (unsigned long long)collection_id,
				   (unsigned long)chunk_index);
	} else {
		written = snprintf(path, path_size, "%s/%s%llu_%04lu.bin",
				   fs_mnt.mnt_point, stream_prefix,
				   (unsigned long long)collection_id,
				   (unsigned long)chunk_index);
	}

	if ((written < 0) || ((size_t)written >= path_size)) {
		return -ENAMETOOLONG;
	}

	return 0;
}


int total_test_files = 0;
int total_log_files = 0;







// Test files
char test_file_arr[TEST_FILE_WRITE_BYTES] = "hello world, this is a story about a man who liked to run. "
    "every day for miles. he wandered and wandered for miles. "
    "as the seasons changed, he kept moving, tracing the edges of towns "
    "and forests, learning the quiet language of the wind. "
    "people sometimes asked him why he ran so far, but he only smiled, "
    "because the answer was something he felt rather than spoke. "
    "the rhythm of his footsteps steadied his thoughts, and the long roads "
    "gave him room to remember who he was and who he hoped to become. "
    "on certain mornings, when the fog clung low to the fields, he felt "
    "as though he were the only person awake in the world. "
    "he liked those mornings best. "
    "they reminded him that solitude was not the same as loneliness; "
    "it was a kind of quiet companionship with the earth itself. "
    "and so he kept running, mile after mile, year after year, "
    "carrying stories in his breath and dreams in his stride.";

	
int create_test_file(int writes)
{
	char destination[50] = "";
	char id_string[12];
	struct fs_file_t test_file;
	struct fs_mount_t *mp = &fs_mnt;
	FRESULT expand_ret;
	int close_ret;
	int ret;
	int sync_ret;
	ssize_t write_ret;

	if (!file_system_ready || !filesystem_mounted) {
		LOG_ERR("Filesystem is unavailable for test-file creation");
		return -EACCES;
	}
	if (writes < 0) {
		return -EINVAL;
	}

	printk("write file...\n");
	fs_file_t_init(&test_file);
	total_test_files++;
	ret = snprintf(id_string, sizeof(id_string), "%d", total_test_files);
	if (ret < 0 || ret >= (int)sizeof(id_string)) {
		return -ENAMETOOLONG;
	}

	strcat(destination, mp->mnt_point);
	strcat(destination, "/");
	strcat(destination, id_string);
	strcat(destination, "testing.txt");
	ret = fs_open(&test_file, destination, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		return ret;
	}

	expand_ret = f_expand((FIL *)test_file.filep, TEST_FILE_PREALLOCATED_BYTES, 1);
	if (expand_ret != FR_OK) {
		LOG_WRN("Failed to expand test file: %d", expand_ret);
		ret = -EIO;
		goto close_file;
	}

	ret = 0;
	for (int i = 0; i < writes; i++) {
		write_ret = fs_write(&test_file, test_file_arr, sizeof(test_file_arr));
		if (write_ret < 0) {
			ret = (int)write_ret;
			break;
		}
		if (write_ret != (ssize_t)sizeof(test_file_arr)) {
			ret = -EIO;
			break;
		}
	}

close_file:
	sync_ret = fs_sync(&test_file);
	close_ret = fs_close(&test_file);
	if (sync_ret != 0) {
		LOG_ERR("Test-file sync failed: %d", sync_ret);
	}
	if (close_ret != 0) {
		LOG_ERR("Test-file close failed: %d", close_ret);
	}
	if (ret == 0 && sync_ret != 0) {
		ret = sync_ret;
	}
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}
	if (ret == 0) {
		printk("done write\n");
	}

	return ret;
}

int create_test_files(int number_of_files)
{
	int ret;

	if (number_of_files < 0) {
		return -EINVAL;
	}

	LOG_INF("creating test files...");
	for (int x = 0; x < number_of_files; x++) {
		LOG_INF("file %d of %d", x, number_of_files);
		ret = create_test_file(TEST_FILE_WRITES_PER_PREALLOCATION);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

bool is_file_open(struct fs_file_t *zfp) {
    // If filep is not NULL, the file wrapper structure is currently active/open
    return (zfp != NULL && zfp->filep != NULL);
}

bool file_exists(const char *path)
{
    struct fs_dirent entry;
    
    // fs_stat returns 0 on success (file exists)
    int ret = fs_stat(path, &entry);
    
    if (ret == 0) {
        // Optional: Ensure it's a file and not a directory
        return entry.type == FS_DIR_ENTRY_FILE;
    }
    
    // Returns -ENOENT if the file does not exist
    return false;
}

bool filesystem_is_mounted(void)
{
	return filesystem_mounted;
}

int filesystem_drain_pending_work(void)
{
	int ret;

	ret = k_work_queue_drain(&my_work_q, false);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int filesystem_gate_and_drain(void)
{
	int ret;

	ret = k_work_queue_drain(&my_work_q, true);
	file_system_ready = false;
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int logger_sync_and_close_file(struct fs_file_t *file_to_close)
{
	int sync_ret;
	int close_ret;

	if (file_to_close->mp == NULL) {
		return 0;
	}

	sync_ret = fs_sync(file_to_close);
	close_ret = fs_close(file_to_close);
	if (sync_ret != 0) {
		return sync_ret;
	}

	return close_ret;
}

static int logger_reset_file(void)
{
	int ret;

	if (logger_file_retired) {
		log_file.buffer1.current_size = 0U;
		log_file.buffer2.current_size = 0U;
		log_file.switch_buffer = false;
		return -EIO;
	}
	ret = logger_sync_and_close_file(&log_file.self_file);
	if (ret != 0) {
		return ret;
	}

	fs_file_t_init(&log_file.self_file);
	log_file.buffer1.current_size = 0U;
	log_file.buffer2.current_size = 0U;
	log_file.switch_buffer = false;
	log_file.current_writes = 0;
	return 0;
}

static void logger_release_retired_file(void)
{
	if (!logger_file_retired) {
		return;
	}
	if (logger_retired_handle_live) {
		/* FatFs is unmounted, so this releases its FIL slab without flushing data. */
		(void)fs_close(&log_file.self_file);
	}
	fs_file_t_init(&log_file.self_file);
	logger_file_retired = false;
	logger_retired_handle_live = false;
	logger_file_open = false;
	logger_logical_bytes = 0U;
	logger_next_prepared = false;
	logger_prepare_requested = false;
	logger_started = false;
}




int shutdown_filesystem(void)
{
	int close_ret;
	int unmount_ret;

	file_system_ready = false;
	if (!filesystem_mounted) {
		return 0;
	}

	close_ret = logger_reset_file();
	unmount_ret = fs_unmount(&fs_mnt);
	if (unmount_ret == 0) {
		filesystem_mounted = false;
		logger_release_retired_file();
		accel_recorder_filesystem_unmounted();
	} else {
		LOG_ERR("Failed to unmount filesystem: %d", unmount_ret);
	}

	if (close_ret != 0) {
		return close_ret;
	}

	return unmount_ret;
}

static int logger_write_failure(const char *operation, int error)
{
	int ret = error < 0 ? error : -EIO;

	file_system_malfunction = true;
	status_reg_ble_notification();
	LOG_WRN("Logger %s failed: %d", operation, error);
	return ret;
}

uint8_t *filesystem_scratch_buffer(void)
{
	/* Filesystem work is serialized on my_work_q, so metadata may share a page. */
	return filesystem_scratch;
}

int filesystem_preallocate_file(struct fs_file_t *file, const char *path,
				uint32_t file_bytes, const void *header,
				size_t header_bytes, bool leave_open)
{
	struct fs_dirent entry;
	FRESULT expand_ret;
	ssize_t written;
	bool write_attempted = false;
	bool close_attempted = false;
	int close_ret = 0;
	int ret;

	if (file == NULL || path == NULL || (header == NULL && header_bytes != 0U)) {
		return -EINVAL;
	}
	ret = fs_stat(path, &entry);
	if (ret == 0) {
		return -EEXIST;
	}
	if (ret != -ENOENT) {
		return ret;
	}
	fs_file_t_init(file);
	ret = fs_open(file, path, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		return ret;
	}
	expand_ret = f_expand((FIL *)file->filep, file_bytes, 1);
	if (expand_ret != FR_OK) {
		ret = -EIO;
		goto fail;
	}
	if (header_bytes != 0U) {
		write_attempted = true;
		written = fs_write(file, header, header_bytes);
		if (written != (ssize_t)header_bytes) {
			ret = written < 0 ? (int)written : -EIO;
			goto fail;
		}
	}
	ret = fs_sync(file);
	if (ret == 0 && !leave_open) {
		close_attempted = true;
		close_ret = fs_close(file);
		fs_file_t_init(file);
		ret = close_ret;
	}
	if (ret == 0) {
		return 0;
	}

fail:
	if (!close_attempted) {
		close_attempted = true;
		close_ret = fs_close(file);
		fs_file_t_init(file);
	}
	/* A data-write attempt may already have programmed its NAND page. */
	if (!write_attempted) {
		(void)fs_unlink(path);
	}
	return ret;
}

int filesystem_open_preallocated_file(struct fs_file_t *file, const char *path,
				      uint32_t offset)
{
	int ret;

	fs_file_t_init(file);
	ret = fs_open(file, path, FS_O_WRITE);
	if (ret == 0) {
		ret = fs_seek(file, (off_t)offset, FS_SEEK_SET);
	}
	if (ret != 0) {
		(void)fs_close(file);
	}
	return ret;
}

static int logger_make_path(char *path, size_t path_size, uint32_t index)
{
	int written;

	if (patient_num != 0) {
		written = snprintf(path, path_size, "%s/%dlog%lu.txt",
				   fs_mnt.mnt_point, patient_num,
				   (unsigned long)index);
	} else {
		written = snprintf(path, path_size, "%s/log%lu.txt",
				   fs_mnt.mnt_point, (unsigned long)index);
	}
	return (written < 0 || (size_t)written >= path_size) ? -ENAMETOOLONG : 0;
}

static int logger_expand_file(struct fs_file_t *file, const char *path,
			      bool leave_open)
{
	return filesystem_preallocate_file(file, path, RECORDING_FILE_BYTES,
					   NULL, 0U, leave_open);
}

static void logger_retire_file(void)
{
	logger_file_retired = true;
	logger_file_open = false;
	logger_retired_handle_live = true;
}

static int logger_close_current_file(void)
{
	int ret;

	if (logger_file_retired) {
		return -EIO;
	}
	if (!logger_file_open) {
		return 0;
	}
	ret = fs_truncate(&log_file.self_file, (off_t)logger_logical_bytes);
	if (ret != 0) {
		logger_retire_file();
		return ret;
	}
	ret = fs_close(&log_file.self_file);
	if (ret != 0) {
		logger_file_retired = true;
		logger_file_open = false;
		logger_retired_handle_live = false;
		fs_file_t_init(&log_file.self_file);
		return ret;
	}
	fs_file_t_init(&log_file.self_file);
	logger_file_open = false;
	return 0;
}

static int logger_prepare_next_file(void)
{
	struct fs_file_t file;
	uint32_t index = (uint32_t)total_log_files + 1U;
	int ret;

	ret = logger_make_path(logger_next_path, sizeof(logger_next_path), index);
	if (ret != 0) {
		return ret;
	}
	ret = logger_expand_file(&file, logger_next_path, false);
	if (ret == 0) {
		total_log_files++;
		logger_next_prepared = true;
	}
	return ret;
}

static int logger_start_worker(void)
{
	uint32_t index = (uint32_t)total_log_files + 1U;
	int ret;

	if (logger_file_retired) {
		return -EIO;
	}
	ret = logger_reset_file();
	if (ret != 0) {
		return ret;
	}
	logger_dropped_buffers = 0U;
	ret = logger_make_path(log_file.file_name, sizeof(log_file.file_name), index);
	if (ret != 0) {
		return ret;
	}
	ret = logger_expand_file(&log_file.self_file, log_file.file_name, true);
	if (ret != 0) {
		return ret;
	}
	logger_logical_bytes = 0U;
	logger_file_open = true;
	total_log_files++;
	ret = logger_prepare_next_file();
	if (ret != 0) {
		(void)fs_close(&log_file.self_file);
		logger_file_open = false;
		(void)fs_unlink(log_file.file_name);
		return ret;
	}
	logger_started = true;
	return 0;
}

static int logger_activate_next_file(void)
{
	int ret;

	if (logger_file_retired) {
		return -EIO;
	}
	if (!logger_next_prepared) {
		return -ENOSPC;
	}
	/* Opening a preallocated file starts at offset zero; avoid a recoverable
	 * seek path after a partial-capable stream has failed. */
	fs_file_t_init(&log_file.self_file);
	ret = fs_open(&log_file.self_file, logger_next_path, FS_O_WRITE);
	if (ret != 0) {
		return ret;
	}
	strcpy(log_file.file_name, logger_next_path);
	log_file.current_writes = 0;
	logger_logical_bytes = 0U;
	logger_file_open = true;
	logger_next_prepared = false;
	logger_prepare_requested = true;
	return 0;
}

static void logger_prepare_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);
	if (!logger_started || logger_next_prepared || logger_file_retired) {
		return;
	}
	ret = logger_prepare_next_file();
	if (ret != 0) {
		filesystem_latch_fault();
	}
}

static void logger_control_work_handler(struct k_work *work)
{
	enum logger_control_operation operation = logger_control_operation;
	int ret = 0;

	ARG_UNUSED(work);
	if (operation == LOGGER_CONTROL_START) {
		ret = logger_start_worker();
	} else if (logger_started) {
		ret = logger_close_current_file();
		logger_started = false;
		if (logger_next_prepared) {
			int unlink_ret = fs_unlink(logger_next_path);

			if (ret == 0) {
				ret = unlink_ret;
			}
			logger_next_prepared = false;
		}
		if (logger_dropped_buffers != 0U) {
			LOG_WRN("Logger dropped %u buffers while storage was busy",
				(unsigned int)logger_dropped_buffers);
		}
	}
	logger_control_result = ret;
	k_sem_give(&logger_control_done);
}

static int logger_submit_control(enum logger_control_operation operation)
{
	int ret;

	if (!logger_control_initialized) {
		k_work_init(&logger_control_work, logger_control_work_handler);
		k_work_init(&logger_prepare_work, logger_prepare_work_handler);
		k_sem_init(&logger_control_done, 0, 1);
		logger_control_initialized = true;
	}
	k_sem_reset(&logger_control_done);
	logger_control_operation = operation;
	ret = k_work_submit_to_queue(&my_work_q, &logger_control_work);
	if (ret <= 0) {
		return ret < 0 ? ret : -EALREADY;
	}
	ret = k_sem_take(&logger_control_done, K_FOREVER);
	return ret == 0 ? logger_control_result : ret;
}

static void logger_initialize_write_work(void)
{
	if (!logger_write_work_initialized) {
		k_work_init(&logger_work_item.work, logger_write_work_handler);
		logger_write_work_initialized = true;
	}
}

int filesystem_logger_start(void)
{
	logger_initialize_write_work();
	return logger_submit_control(LOGGER_CONTROL_START);
}

int filesystem_logger_stop(void)
{
	return logger_started ? logger_submit_control(LOGGER_CONTROL_STOP) : 0;
}

static int logger_rollover_file(void)
{
	int ret;

	ret = logger_close_current_file();
	if (ret != 0) {
		return ret;
	}
	ret = get_storage_percent_full();
	if (ret < 0) {
		return ret;
	}
	if (storage_percent_full >= 99U) {
		return -ENOSPC;
	}
	return logger_activate_next_file();
}

static int logger_write_bytes(const uint8_t *data, size_t size)
{
	size_t offset = 0U;

	if (!logger_started || logger_file_retired || !logger_file_open) {
		return -EIO;
	}
	while (offset < size) {
		size_t remaining = RECORDING_FILE_BYTES - logger_logical_bytes;
		size_t write_bytes = MIN(size - offset, remaining);
		ssize_t written;
		int ret;

		if (write_bytes == 0U) {
			ret = logger_rollover_file();
			if (ret != 0) {
				return ret;
			}
			continue;
		}
		written = fs_write(&log_file.self_file, &data[offset], write_bytes);
		if (written != (ssize_t)write_bytes) {
			logger_retire_file();
			return written < 0 ? (int)written : -EIO;
		}
		logger_logical_bytes += write_bytes;
		offset += write_bytes;
		log_file.current_writes++;
		if (logger_logical_bytes == RECORDING_FILE_BYTES) {
			ret = logger_rollover_file();
			if (ret != 0) {
				return ret;
			}
		}
	}
	return 0;
}

static int logger_write_to_file(const void *data, size_t size)
{
	int ret;

	if (!file_system_ready || !filesystem_mounted) {
		return logger_write_failure("filesystem unavailable", -EACCES);
	}
	if (storage_percent_full >= 99U) {
		return logger_write_failure("storage full", -ENOSPC);
	}
	if (IS_ENABLED(CONFIG_DISK_DRIVER_RAW_NAND) && get_read_only()) {
		return logger_write_failure("raw disk is read-only", -EROFS);
	}
	if (data == NULL) {
		return logger_write_failure("invalid write", -EINVAL);
	}

	ret = logger_write_bytes(data, size);
	return ret == 0 ? 0 : logger_write_failure("file write", ret);
}

static void logger_write_work_handler(struct k_work *item)
{
	logger_write_work *work_item =
		CONTAINER_OF(item, logger_write_work, work);
	int write_ret;
	int64_t time_value;
	bool first_write = log_file.current_writes == 0;

	start_timer(&logger_write_timer);
	work_item->in_use = true;
	write_ret = logger_write_to_file(work_item->address, work_item->size);
	time_value = stop_timer(&logger_write_timer);
	if (write_ret == 0) {
		if (first_write) {
			LOG_INF("storage: log file started; first write %zu bytes in %lli ms",
				work_item->size, time_value);
		} else if (time_value > STORAGE_SLOW_WRITE_LOG_THRESHOLD_MS) {
			LOG_WRN("storage: slow log write; packet %d, %zu bytes in %lli ms",
				work_item->packet_num, work_item->size, time_value);
		}
	}
	if (work_item->packet_num <= logger_last_packet_number_processed) {
		LOG_ERR("Logger FIFO in k_work not met");
	}
	logger_last_packet_number_processed = work_item->packet_num;
	work_item->in_use = false;
	if (write_ret != 0) {
		LOG_ERR("Logger filesystem write failed: %d", write_ret);
		filesystem_latch_fault();
	}
	if (logger_prepare_requested) {
		logger_prepare_requested = false;
		if (k_work_submit_to_queue(&my_work_q, &logger_prepare_work) < 0) {
			filesystem_latch_fault();
		}
	}
}

static int logger_submit_buffer(const void *data, size_t size)
{
	int ret;
	int work_status;

	if (!file_system_ready || !filesystem_mounted) {
		return -EACCES;
	}
	work_status = k_work_busy_get(&logger_work_item.work);
	if (work_status != 0 || logger_work_item.in_use) {
		return -EBUSY;
	}

	logger_work_item.address = data;
	logger_work_item.size = size;
	logger_work_item.packet_num = ++logger_packet_number;
	ret = k_work_submit_to_queue(&my_work_q, &logger_work_item.work);
	if (ret != 1) {
		upload_timeout_errors++;
		LOG_ERR("Logger work submit returned %d, total errors: %d", ret,
			upload_timeout_errors);
		return ret < 0 ? ret : -EALREADY;
	}
	return 0;
}


int filesystem_logger_append(const void *data, size_t size)
{
	data_upload_buffer *current_buffer;
	int ret;

	if (data == NULL && size != 0U) {
		return -EINVAL;
	}
	current_buffer = log_file.switch_buffer ? &log_file.buffer2 :
		&log_file.buffer1;
	if (current_buffer->current_size >= log_file.write_size) {
		logger_dropped_buffers++;
		current_buffer->current_size = 0U;
	}
	if (size > sizeof(current_buffer->data_upload_buffer) -
		    current_buffer->current_size) {
		LOG_ERR("Logger buffer capacity exceeded");
		filesystem_latch_fault();
		return -ENOSPC;
	}

	if (size != 0U) {
		memcpy(&current_buffer->data_upload_buffer[current_buffer->current_size],
		       data, size);
	}
	current_buffer->current_size += size;
	if (current_buffer->current_size < log_file.write_size) {
		return 0;
	}
	if (current_buffer->current_size != log_file.write_size) {
		LOG_WRN("Logger buffer is %zu bytes over the target",
			current_buffer->current_size - log_file.write_size);
	}
	if (panic_single_thread) {
		LOG_ERR("Cannot queue a logger buffer during panic mode");
		filesystem_latch_fault();
		return -ENOTSUP;
	}
	ret = logger_submit_buffer(current_buffer->data_upload_buffer,
				  current_buffer->current_size);
	if (ret != 0) {
		logger_dropped_buffers++;
		current_buffer->current_size = 0U;
		return 0;
	}
	/* The worker owns this buffer before the producer changes buffers. */
	current_buffer->current_size = 0U;
	log_file.switch_buffer = !log_file.switch_buffer;
	return 0;
}

int filesystem_logger_flush(void)
{
	struct k_work_sync log_sync;
	data_upload_buffer *current_buffer;
	int ret;

	if (!logger_write_work_initialized) {
		return 0;
	}
	(void)k_work_flush(&logger_work_item.work, &log_sync);
	current_buffer = log_file.switch_buffer ? &log_file.buffer2 :
		&log_file.buffer1;
	if (current_buffer->current_size == 0U) {
		return 0;
	}
	if (panic_single_thread) {
		LOG_ERR("Cannot flush a logger buffer during panic mode");
		return -ENOTSUP;
	}
	ret = logger_submit_buffer(current_buffer->data_upload_buffer,
				  current_buffer->current_size);
	if (ret != 0) {
		logger_dropped_buffers++;
		current_buffer->current_size = 0U;
		return 0;
	}
	current_buffer->current_size = 0U;
	log_file.switch_buffer = !log_file.switch_buffer;
	return 0;
}


int write_device_info_file(const char *device_name, const char *device_id_hex,
			   const char *dis_model)
{
	struct fs_mount_t *mp = &fs_mnt;
	char uuid_name[32];
	char uuid_contents[UUID_CONTENTS_MAX_SIZE];
	int written;

	if (device_name == NULL || device_id_hex == NULL || dis_model == NULL) {
		return -EINVAL;
	}
	if (!file_system_ready || !filesystem_mounted) {
		return -EACCES;
	}

	written = snprintf(uuid_name, sizeof(uuid_name), "%s/uuid.txt", mp->mnt_point);
	if (written < 0 || written >= sizeof(uuid_name)) {
		return -ENAMETOOLONG;
	}

	written = snprintf(uuid_contents, sizeof(uuid_contents),
			   "Name: %s\nDevice ID: %s\nVersion: %s"
			   "\nBuild Date (UTC): %s\nGit Commit: %s\nGit Tree: %s"
			   "\naccel format: ICM-20948 accel binary format v2"
			   "\necg format: %s"
			   "\nFor a more complete description of how this device works, please visit "
			   "https://github.com/SenSE-Lab-OSU/MotionSenseHRV4Flash for more info.\n",
			   device_name, device_id_hex, dis_model,
			   MSENSE_BUILD_DATE_UTC, MSENSE_GIT_COMMIT, MSENSE_GIT_TREE_STATE,
			   ECG_FILE_FORMAT);
	if (written < 0 || written >= sizeof(uuid_contents)) {
		return -ENOSPC;
	}

	return msense_uuid_file_ensure(uuid_name, uuid_contents, (size_t)written);
}

static int setup_flash(struct fs_mount_t *mnt)
{
	int rc = 0;
#if CONFIG_DISK_DRIVER_FLASH
	unsigned int id;
	const struct flash_area *pfa;

	mnt->storage_dev = (void *)STORAGE_PARTITION_ID;
	id = STORAGE_PARTITION_ID;

	rc = flash_area_open(id, &pfa);
	LOG_INF("Area %u at 0x%x on %s for %u bytes",
	       id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
	       (unsigned int)pfa->fa_size);

	if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		printk("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		printk("%d\n", rc);
	}

	if (rc < 0) {
		flash_area_close(pfa);
	}
#endif
	return rc;
}

static int mount_app_fs(struct fs_mount_t *mnt)
{
	int rc;

#if CONFIG_FAT_FILESYSTEM_ELM
	static FATFS fat_fs;
	//FS_MOUNT_FLAG_USE_DISK_ACCESS
	//mnt->flags = FS_MOUNT_FLAG_READ_ONLY;
	mnt->type = FS_FATFS;
	mnt->fs_data = &fat_fs;
	if (IS_ENABLED(CONFIG_DISK_DRIVER_RAM)) {
		mnt->mnt_point = "/RAM:";
	} else if (IS_ENABLED(CONFIG_DISK_DRIVER_SDMMC) | IS_ENABLED(CONFIG_DISK_DRIVER_RAW_NAND) | 
	IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
		mnt->mnt_point = "/SD:";
	}

#elif CONFIG_FILE_SYSTEM_LITTLEFS
	mnt->type = FS_LITTLEFS;
	mnt->mnt_point = "/lfs";
	mnt->fs_data = &storage;
#endif
	rc = fs_mount(mnt);

	return rc;
}

int setup_disk(void)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_dir_t dir;
	struct fs_statvfs sbuf;
	bool dir_open = false;
	int close_ret;
	int unmount_ret;
	int rc;

	if (filesystem_mounted) {
		return file_system_ready ? 0 : -EBUSY;
	}

	file_system_ready = false;
	total_test_files = 0;
	total_log_files = 0;
	fs_dir_t_init(&dir);

	if (IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
		rc = setup_flash(mp);
		if (rc < 0) {
			LOG_ERR("Failed to setup flash area: %d", rc);
			return rc;
		}
	}

	if (!IS_ENABLED(CONFIG_FILE_SYSTEM_LITTLEFS) &&
	    !IS_ENABLED(CONFIG_FAT_FILESYSTEM_ELM)) {
		LOG_ERR("No file system selected");
		return -ENOTSUP;
	}

	rc = mount_app_fs(mp);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem: %d", rc);
		return rc;
	}
	filesystem_mounted = true;
	/* Allow log messages to flush to avoid interleaved output. */
	k_sleep(K_MSEC(50));

	LOG_INF("Mount %s: %d", fs_mnt.mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		LOG_ERR("statvfs failed: %d", rc);
		goto unmount;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu",
	       mp->mnt_point,
	       sbuf.f_bsize, sbuf.f_frsize,
	       sbuf.f_blocks, sbuf.f_bfree);

	rc = fs_opendir(&dir, mp->mnt_point);
	LOG_INF("%s opendir: %d", mp->mnt_point, rc);
	if (rc < 0) {
		LOG_ERR("Failed to open directory");
		goto unmount;
	}
	dir_open = true;

	for (;;) {
		struct fs_dirent ent = { 0 };

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			LOG_ERR("Failed to read directory entries");
			break;
		}
		if (ent.name[0] == 0) {
			LOG_INF("End of files");
			break;
		}
		LOG_INF("  %c %u %s",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		       ent.size,
		       ent.name);

		if (strstr(ent.name, "test") != NULL) {
			total_test_files++;
		}
		if (strstr(ent.name, "log") != NULL) {
			total_log_files++;
		}
	}

	close_ret = fs_closedir(&dir);
	dir_open = false;
	if (close_ret != 0 && rc == 0) {
		rc = close_ret;
	}
	if (rc != 0) {
		goto unmount;
	}

	file_system_ready = true;
	return 0;

unmount:
	if (dir_open) {
		close_ret = fs_closedir(&dir);
		if (rc == 0 && close_ret != 0) {
			rc = close_ret;
		}
	}

	file_system_ready = false;
	unmount_ret = fs_unmount(mp);
	if (unmount_ret == 0) {
		filesystem_mounted = false;
	} else {
		LOG_ERR("Failed to unmount filesystem after setup error: %d", unmount_ret);
		if (rc == 0) {
			rc = unmount_ret;
		}
	}

	return rc;
}


int get_storage_percent_full(){
	struct fs_statvfs info;
	struct fs_mount_t* mp = &fs_mnt;
	int rc;
	if (!file_system_ready || !filesystem_mounted) {
		return -EACCES;
	}
	rc = fs_statvfs(mp->mnt_point, &info);
	if (rc != 0) {
		printk("FAIL: statvfs: %d\n", rc);
		return rc < 0 ? rc : -EIO;
	}

	printk("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu\n",
	       mp->mnt_point,
	       info.f_bsize, info.f_frsize,
	       info.f_blocks, info.f_bfree);

	float storage_percent = (info.f_blocks - info.f_bfree);
	storage_percent /= info.f_blocks;
	storage_percent *= 100;
	storage_percent_full = (uint8_t)storage_percent;
	if (storage_percent_full >= 99){
		file_system_full = true;
	}
	storage_ble_notification(&storage_percent_full, sizeof(storage_percent_full));
	LOG_INF("storage: %f and %i and total_errors %i", (double)storage_percent, storage_percent_full, upload_timeout_errors);
	return (int)storage_percent;

}




#include "nand_disk.h"
// The following shows how to use the nand disk driver outside of the driver file directly.


#define DT_DRV_COMPAT senselab_nanddisk
int test_desk_driver(){
	uint8_t write_buf[4096] = {1};
	uint8_t read_buf[4096];
	// you can do:
	//const struct device* filesystem_device = DEVICE_DT_INST_GET(0);
	// OR
	const struct device* filesystem_device2 = sdmmc_disk.dev;
	spi_nand_page_write(filesystem_device2, 63, write_buf, sizeof(read_buf));
	spi_nand_page_write(filesystem_device2, 64, write_buf, sizeof(read_buf));
	spi_nand_page_read(filesystem_device2, 63, read_buf);
	print_page_hex(read_buf, sizeof(read_buf), true);
	spi_nand_block_erase(filesystem_device2, 0);
	spi_nand_page_read(filesystem_device2, 63, read_buf);
	print_page_hex(read_buf, sizeof(read_buf), true);
	spi_nand_page_read(filesystem_device2, 64, read_buf);
	print_page_hex(read_buf, sizeof(read_buf), true);
	return 0;
}



int read_storage_percent_full(){
	return storage_percent_full;
}

#define FAT_UNIX_TIME_MIN_SECONDS 315532800ULL
#define FAT_UNIX_TIME_MAX_SECONDS 4354819199ULL

void set_date_time_bt(uint64_t value){
	if (value < FAT_UNIX_TIME_MIN_SECONDS ||
	    value > FAT_UNIX_TIME_MAX_SECONDS) {
		LOG_WRN("rejected invalid Unix time in seconds: %llu", value);
		return;
	}

	set_date_time = value;
	last_time_update_sent = k_uptime_get() / 1000;
	LOG_INF("new datetime sent, value is %llu, seconds uptime is %llu", set_date_time, last_time_update_sent);
}

uint64_t get_current_unix_time(){
	
	uint64_t current_upime = k_uptime_get();
	current_upime /= 1000;
	LOG_DBG("current uptime in seconds: %llu", current_upime);
	uint64_t current_time = (current_upime - last_time_update_sent) + set_date_time;
	LOG_DBG("current timestamp: %llu", current_time);
	return current_time;
}

// for now we will use Mountain Time (UTC -7)
#define TIMEZONE_SHIFT -8
// To make this work with the file system, you will need to set FF_FS_NORTC to 0 in  zephyr\modules\fatfs (line 82).
DWORD get_fattime(void)
{
	time_t t;
	struct tm *stm;

	t = get_current_unix_time();
	// stm = localtime(&t);
	if (set_date_time != 0)
	{
		stm = gmtime(&t);
		return (DWORD)(stm->tm_year - 80) << 25 |
			   (DWORD)(stm->tm_mon + 1) << 21 |
			   (DWORD)stm->tm_mday << 16 |
			   (DWORD)(stm->tm_hour + TIMEZONE_SHIFT) << 11 |
			   (DWORD)stm->tm_min << 5 |
			   (DWORD)stm->tm_sec >> 1;
	}
	LOG_WRN_ONCE("FAT date/time is unset; recording timestamps will be invalid");
	return 0;
}

int64_t start_time;

void start_timer(int64_t *start_time_ref){
	if (start_time_ref != NULL){
		if (*start_time_ref != 0){
			LOG_WRN("timer was executed again before it could finish!");
		}
		*start_time_ref = k_uptime_get();
	}
	else{
		start_time = k_uptime_get();
	}
}


int64_t stop_timer(int64_t *start_time_ref){
	int64_t length;
	if (start_time_ref != NULL){
		length = k_uptime_get() - *start_time_ref;
		*start_time_ref = 0;
	}
	else{
		length = k_uptime_get() - start_time;
		start_time = 0;
	}
	return length;
}
