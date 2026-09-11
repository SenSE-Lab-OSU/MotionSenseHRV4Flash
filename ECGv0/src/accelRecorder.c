#include "accelRecorder.h"
#include "accelRecordFormat.h"
#include "accelTimingEstimator.h"
#include "imuFsyncTiming.h"
#include "zephyrfilesystem.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(accel_recorder, CONFIG_LOG_LEVEL_ICM20948_ACCEL);

#define ACCEL_RECORD_BUFFER_COUNT 4U
#define ACCEL_RECORD_SYNC_INTERVAL_BLOCKS 8U
#define ACCEL_RECORD_PATH_MAX 96U

BUILD_ASSERT(ACCEL_RECORD_FORMAT_FILE_BYTES == RECORDING_FILE_BYTES,
	     "Accelerometer and ECG chunk sizes must match");
BUILD_ASSERT(ACCEL_RECORD_FORMAT_BLOCK_BYTES <= FILESYSTEM_SCRATCH_BYTES,
	     "Filesystem scratch page must hold accelerometer metadata");

enum accel_record_block_state {
	ACCEL_RECORD_BLOCK_FREE,
	ACCEL_RECORD_BLOCK_FILLING,
	ACCEL_RECORD_BLOCK_PENDING,
	ACCEL_RECORD_BLOCK_QUEUED,
	ACCEL_RECORD_BLOCK_WRITING,
};

enum accel_record_control_operation {
	ACCEL_RECORD_CONTROL_NONE,
	ACCEL_RECORD_CONTROL_OPEN,
	ACCEL_RECORD_CONTROL_ROTATE,
	ACCEL_RECORD_CONTROL_CLOSE,
	ACCEL_RECORD_CONTROL_ABORT,
};

struct accel_record_block {
	struct k_work work;
	uint8_t data[ACCEL_RECORD_FORMAT_BLOCK_BYTES];
	atomic_t state;
	size_t sample_count;
	size_t write_length;
	uint64_t first_sample_sequence;
	uint32_t reserved_timer_output;
	uint32_t pending_order;
	bool sync_after_write;
};

static struct accel_record_block accel_record_blocks[ACCEL_RECORD_BUFFER_COUNT] __aligned(4);
static struct fs_file_t accel_record_file;
static struct k_work accel_record_control_work;
static struct k_work accel_record_prepare_work;
static struct k_sem accel_record_control_done;
static struct k_spinlock accel_record_state_lock;
static atomic_t accel_record_active;
static atomic_t accel_record_failed;
static atomic_t accel_record_rotating;
static atomic_t accel_record_rotation_completion_pending;
static bool accel_record_initialized;
static bool accel_record_file_open;
static bool accel_record_next_prepared;
static bool accel_record_chunk_full;
static uint32_t accel_record_session_full_block_count;
static uint32_t accel_record_chunk_full_block_count;
static uint32_t accel_record_chunk_data_bytes;
static uint32_t accel_record_chunk_index;
static uint64_t accel_record_session_id;
static uint64_t accel_record_next_sample_sequence;
static uint32_t accel_record_dropped_samples;
static uint32_t accel_record_pending_order;
static struct accel_record_block *accel_record_filling_block;
static enum accel_record_control_operation accel_record_control_operation;
static int accel_record_control_result;
static char accel_record_path[ACCEL_RECORD_PATH_MAX];
static char accel_record_next_path[ACCEL_RECORD_PATH_MAX];
static accel_recorder_fault_handler_t accel_record_fault_handler;
static void *accel_record_fault_context;
static struct accel_timing_estimator accel_record_timing_estimator;
static bool accel_record_marker_valid;
static uint8_t accel_record_previous_marker;

static void accel_record_report_fault(int error)
{
	if (atomic_cas(&accel_record_failed, 0, 1)) {
		atomic_clear(&accel_record_active);
		LOG_ERR("Accelerometer recording failed: %d", error);
		if (accel_record_fault_handler != NULL) {
			accel_record_fault_handler(accel_record_fault_context);
		}
	}
}

static int accel_record_finalize_block(struct accel_record_block *block)
{
	uint32_t estimated_ticks;
	int ret;

	ret = accel_timing_estimator_estimate(&accel_record_timing_estimator,
					      block->first_sample_sequence,
					      &estimated_ticks, NULL);
	if (ret != 0) {
		return ret;
	}

	block->reserved_timer_output = estimated_ticks;

	block->write_length = accel_record_format_finalize_block(
		block->data, block->sample_count,
		(uint32_t)block->first_sample_sequence,
		block->reserved_timer_output);
	if (block->sample_count == ACCEL_RECORD_FORMAT_SAMPLES_PER_BLOCK) {
		accel_record_session_full_block_count++;
	}

	return 0;
}

static struct accel_record_block *accel_record_take_free_block(void)
{
	for (size_t i = 0U; i < ARRAY_SIZE(accel_record_blocks); i++) {
		if (atomic_cas(&accel_record_blocks[i].state,
			       ACCEL_RECORD_BLOCK_FREE,
			       ACCEL_RECORD_BLOCK_FILLING)) {
			accel_record_blocks[i].sample_count = 0U;
			accel_record_blocks[i].write_length = 0U;
			accel_record_blocks[i].first_sample_sequence = 0U;
			accel_record_blocks[i].reserved_timer_output = 0U;
			accel_record_blocks[i].pending_order = 0U;
			accel_record_blocks[i].sync_after_write = false;
			return &accel_record_blocks[i];
		}
	}

	return NULL;
}

static void accel_record_release_block(struct accel_record_block *block)
{
	if (block != NULL) {
		atomic_set(&block->state, ACCEL_RECORD_BLOCK_FREE);
	}
}

static int accel_record_queue_block(struct accel_record_block *block)
{
	int ret;

	atomic_set(&block->state, ACCEL_RECORD_BLOCK_QUEUED);
	ret = k_work_submit_to_queue(&my_work_q, &block->work);
	if (ret != 1) {
		accel_record_release_block(block);
		return (ret < 0) ? ret : -EALREADY;
	}

	return 0;
}

static int accel_record_consume_fsync_marker(uint8_t marker,
					       uint64_t sample_sequence)
{
	struct imu_fsync_edge edge;
	int ret;

	if (!accel_record_marker_valid) {
		/* The timing generator holds FSYNC low until after FIFO streaming starts. */
		if (marker != 0U) {
			return -EIO;
		}
		accel_record_previous_marker = marker;
		accel_record_marker_valid = true;
		return 0;
	}

	if (marker == accel_record_previous_marker) {
		return 0;
	}

	ret = imu_fsync_timing_take_edge(&edge);
	if (ret != 0) {
		return ret;
	}
	if (edge.level != marker) {
		return -EIO;
	}

	ret = accel_timing_estimator_observe(&accel_record_timing_estimator,
		&(struct accel_timing_observation){
			.sample_sequence = sample_sequence,
			.rtc_ticks = edge.rtc_ticks,
		});
	if (ret != 0) {
		return ret;
	}

	accel_record_previous_marker = marker;
	return 0;
}

static int accel_record_queue_current_chunk_block(struct accel_record_block *block)
{
	int ret;

	if ((accel_record_chunk_data_bytes + block->write_length) >
	    ACCEL_RECORD_FORMAT_DATA_BYTES) {
		return -ENOSPC;
	}

	accel_record_chunk_data_bytes += (uint32_t)block->write_length;
	if (block->write_length == ACCEL_RECORD_FORMAT_BLOCK_BYTES) {
		accel_record_chunk_full_block_count++;
	}

	ret = accel_record_queue_block(block);
	if (ret != 0) {
		return ret;
	}

	if (accel_record_chunk_full_block_count ==
	    ACCEL_RECORD_FORMAT_FULL_BLOCKS_PER_FILE) {
		accel_record_chunk_full = true;
	}

	return 0;
}

static int accel_record_write_trailer(uint8_t *metadata)
{
	ssize_t written;
	int ret;

	accel_record_format_build_trailer(metadata,
					  accel_record_chunk_data_bytes);
	ret = fs_seek(&accel_record_file, ACCEL_RECORD_FORMAT_TRAILER_OFFSET,
		      FS_SEEK_SET);
	if (ret != 0) {
		return ret;
	}

	written = fs_write(&accel_record_file, metadata,
			   ACCEL_RECORD_FORMAT_TRAILER_BYTES);
	if (written != (ssize_t)ACCEL_RECORD_FORMAT_TRAILER_BYTES) {
		return (written < 0) ? (int)written : -EIO;
	}

	return fs_sync(&accel_record_file);
}

static int accel_record_close_current_chunk(void)
{
	int ret = 0;
	int close_ret;

	if (!accel_record_file_open) {
		return 0;
	}

	ret = accel_record_write_trailer(filesystem_scratch_buffer());
	close_ret = fs_close(&accel_record_file);
	accel_record_file_open = false;
	if ((ret == 0) && (close_ret != 0)) {
		ret = close_ret;
	}

	return ret;
}

static int accel_record_open_current_chunk(void)
{
	uint8_t *metadata = filesystem_scratch_buffer();
	int ret;

	ret = filesystem_make_recording_chunk_path(
		accel_record_path, sizeof(accel_record_path), "ac",
		accel_record_session_id, accel_record_chunk_index);
	if (ret != 0) {
		return ret;
	}

	accel_record_format_build_header(metadata);
	ret = filesystem_preallocate_file(
		&accel_record_file, accel_record_path,
		ACCEL_RECORD_FORMAT_FILE_BYTES, metadata,
		ACCEL_RECORD_FORMAT_BLOCK_BYTES, true);
	if (ret != 0) {
		return ret;
	}
	accel_record_file_open = true;
	LOG_INF("Accelerometer recording to %s", accel_record_path);
	return 0;
}

static int accel_record_prepare_next_chunk(void)
{
	struct fs_file_t file;
	uint32_t next_index = accel_record_chunk_index + 1U;
	int ret;

	ret = filesystem_make_recording_chunk_path(
		accel_record_next_path, sizeof(accel_record_next_path), "ac",
		accel_record_session_id, next_index);
	if (ret != 0) {
		return ret;
	}
	ret = filesystem_preallocate_file(
		&file, accel_record_next_path, ACCEL_RECORD_FORMAT_FILE_BYTES,
		NULL, 0U, false);
	if (ret == 0) {
		accel_record_next_prepared = true;
	}
	return ret;
}

static int accel_record_activate_next_chunk(void)
{
	uint8_t *metadata = filesystem_scratch_buffer();
	ssize_t written;
	int ret;

	if (!accel_record_next_prepared) {
		return -ENOSPC;
	}
	ret = accel_record_close_current_chunk();
	if (ret != 0) {
		return ret;
	}
	accel_record_chunk_index++;
	accel_record_chunk_full_block_count = 0U;
	accel_record_chunk_data_bytes = 0U;
	accel_record_chunk_full = false;
	ret = filesystem_open_preallocated_file(
		&accel_record_file, accel_record_next_path, 0U);
	if (ret != 0) {
		return ret;
	}
	accel_record_file_open = true;
	strcpy(accel_record_path, accel_record_next_path);
	accel_record_next_prepared = false;
	accel_record_format_build_header(metadata);
	written = fs_write(&accel_record_file, metadata,
			   ACCEL_RECORD_FORMAT_BLOCK_BYTES);
	if (written != (ssize_t)ACCEL_RECORD_FORMAT_BLOCK_BYTES) {
		return written < 0 ? (int)written : -EIO;
	}
	return fs_sync(&accel_record_file);
}

static void accel_record_prepare_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);
	if (!accel_record_file_open || accel_record_next_prepared ||
	    atomic_get(&accel_record_failed) != 0) {
		return;
	}
	ret = accel_record_prepare_next_chunk();
	if (ret != 0) {
		accel_record_report_fault(ret);
	}
}

static void accel_record_block_work_handler(struct k_work *work)
{
	struct accel_record_block *block =
		CONTAINER_OF(work, struct accel_record_block, work);
	ssize_t written;
	int ret = 0;

	atomic_set(&block->state, ACCEL_RECORD_BLOCK_WRITING);
	written = fs_write(&accel_record_file, block->data, block->write_length);
	if (written != (ssize_t)block->write_length) {
		ret = (written < 0) ? (int)written : -EIO;
	} else if (block->sync_after_write) {
		ret = fs_sync(&accel_record_file);
	}

	if (ret != 0) {
		accel_record_report_fault(ret);
	}

	accel_record_release_block(block);
}

static int accel_record_dispatch_pending_blocks(void)
{
	int ret = 0;

	for (;;) {
		struct accel_record_block *next = NULL;

		for (size_t i = 0U; i < ARRAY_SIZE(accel_record_blocks); i++) {
			struct accel_record_block *block = &accel_record_blocks[i];

			if (atomic_get(&block->state) !=
			    ACCEL_RECORD_BLOCK_PENDING) {
				continue;
			}
			if ((next == NULL) ||
			    (block->pending_order < next->pending_order)) {
				next = block;
			}
		}

		if (next == NULL) {
			return 0;
		}

		ret = accel_record_queue_current_chunk_block(next);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static void accel_record_control_work_handler(struct k_work *work)
{
	enum accel_record_control_operation operation =
		accel_record_control_operation;
	int ret = 0;

	ARG_UNUSED(work);

	switch (operation) {
	case ACCEL_RECORD_CONTROL_OPEN:
		ret = accel_record_open_current_chunk();
		if (ret == 0) {
			ret = accel_record_prepare_next_chunk();
		}
		if (ret != 0 && accel_record_file_open) {
			(void)fs_close(&accel_record_file);
			accel_record_file_open = false;
		}
		break;

	case ACCEL_RECORD_CONTROL_ROTATE:
		ret = accel_record_activate_next_chunk();
		{
			k_spinlock_key_t key =
				k_spin_lock(&accel_record_state_lock);

			if (ret == 0) {
				ret = accel_record_dispatch_pending_blocks();
			}
			atomic_clear(&accel_record_rotating);
			k_spin_unlock(&accel_record_state_lock, key);
		}

		if (ret == 0) {
			ret = k_work_submit_to_queue(&my_work_q,
						  &accel_record_prepare_work);
			if (ret >= 0) {
				ret = 0;
			}
		}
		break;

	case ACCEL_RECORD_CONTROL_CLOSE:
		ret = accel_record_close_current_chunk();
		break;

	case ACCEL_RECORD_CONTROL_ABORT:
		if (accel_record_file_open) {
			ret = fs_close(&accel_record_file);
			accel_record_file_open = false;
		}
		break;

	case ACCEL_RECORD_CONTROL_NONE:
	default:
		ret = -EINVAL;
		break;
	}
	if ((operation == ACCEL_RECORD_CONTROL_CLOSE ||
	     operation == ACCEL_RECORD_CONTROL_ABORT) &&
	    accel_record_next_prepared) {
		int unlink_ret = fs_unlink(accel_record_next_path);

		if (unlink_ret != 0 && ret == 0) {
			ret = unlink_ret;
		}
		accel_record_next_prepared = false;
	}

	if (ret != 0) {
		accel_record_report_fault(ret);
	}
	accel_record_control_result = ret;
	k_sem_give(&accel_record_control_done);
}

static void accel_record_initialize(void)
{
	if (accel_record_initialized) {
		return;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(accel_record_blocks); i++) {
		k_work_init(&accel_record_blocks[i].work,
			    accel_record_block_work_handler);
		atomic_set(&accel_record_blocks[i].state,
			   ACCEL_RECORD_BLOCK_FREE);
	}
	k_work_init(&accel_record_control_work, accel_record_control_work_handler);
	k_work_init(&accel_record_prepare_work, accel_record_prepare_work_handler);
	k_sem_init(&accel_record_control_done, 0, 1);
	accel_record_initialized = true;
}

static int accel_record_submit_control(
	enum accel_record_control_operation operation)
{
	int ret;

	k_sem_reset(&accel_record_control_done);
	accel_record_control_operation = operation;
	ret = k_work_submit_to_queue(&my_work_q, &accel_record_control_work);
	if (ret <= 0) {
		return (ret < 0) ? ret : -EALREADY;
	}

	ret = k_sem_take(&accel_record_control_done, K_FOREVER);
	if (ret != 0) {
		accel_record_report_fault(ret);
		return ret;
	}

	return accel_record_control_result;
}

static int accel_record_start_rotation(void)
{
	int ret;

	atomic_set(&accel_record_rotating, 1);
	atomic_set(&accel_record_rotation_completion_pending, 1);
	k_sem_reset(&accel_record_control_done);
	accel_record_control_operation = ACCEL_RECORD_CONTROL_ROTATE;
	ret = k_work_submit_to_queue(&my_work_q, &accel_record_control_work);
	if (ret <= 0) {
		atomic_clear(&accel_record_rotating);
		atomic_clear(&accel_record_rotation_completion_pending);
		return (ret < 0) ? ret : -EALREADY;
	}

	return 0;
}

static int accel_record_wait_for_rotation(void)
{
	int ret;

	if (atomic_get(&accel_record_rotation_completion_pending) == 0) {
		return 0;
	}

	ret = k_sem_take(&accel_record_control_done, K_FOREVER);
	if (ret != 0) {
		accel_record_report_fault(ret);
		return ret;
	}

	atomic_clear(&accel_record_rotation_completion_pending);
	return accel_record_control_result;
}

static void accel_record_release_pending_blocks(void)
{
	for (size_t i = 0U; i < ARRAY_SIZE(accel_record_blocks); i++) {
		if (atomic_get(&accel_record_blocks[i].state) ==
		    ACCEL_RECORD_BLOCK_PENDING) {
			accel_record_release_block(&accel_record_blocks[i]);
		}
	}
}

void accel_recorder_set_fault_handler(accel_recorder_fault_handler_t handler,
				      void *context)
{
	accel_record_fault_handler = handler;
	accel_record_fault_context = context;
}

int accel_recorder_start(uint64_t session_id)
{
	int ret;

	accel_record_initialize();
	if (!file_system_ready) {
		return -ENODEV;
	}
	if (atomic_get(&accel_record_active) != 0 ||
	    atomic_get(&accel_record_rotating) != 0 || accel_record_file_open ||
	    accel_record_next_prepared) {
		return -EALREADY;
	}

	atomic_clear(&accel_record_failed);
	atomic_clear(&accel_record_rotating);
	atomic_clear(&accel_record_rotation_completion_pending);
	accel_record_session_id = session_id;
	accel_record_chunk_index = 0U;
	accel_record_session_full_block_count = 0U;
	accel_record_chunk_full_block_count = 0U;
	accel_record_chunk_data_bytes = 0U;
	accel_record_chunk_full = false;
	accel_record_next_prepared = false;
	accel_record_pending_order = 0U;
	accel_record_next_sample_sequence = 0U;
	accel_record_dropped_samples = 0U;
	accel_record_filling_block = NULL;
	accel_timing_estimator_reset(&accel_record_timing_estimator);
	accel_record_marker_valid = false;
	accel_record_previous_marker = 0U;
	ret = accel_record_submit_control(ACCEL_RECORD_CONTROL_OPEN);
	if (ret != 0) {
		return ret;
	}

	accel_record_filling_block = accel_record_take_free_block();
	if (accel_record_filling_block == NULL) {
		(void)accel_recorder_abort();
		return -ENOMEM;
	}

	atomic_set(&accel_record_active, 1);
	return 0;
}

static int accel_record_take_next_filling_block(void)
{
	int ret;

	if (accel_record_filling_block != NULL) {
		return 0;
	}

	if (accel_record_chunk_full &&
	    (atomic_get(&accel_record_rotating) == 0)) {
		ret = accel_record_start_rotation();
		if (ret != 0) {
			return ret;
		}
	}

	accel_record_filling_block = accel_record_take_free_block();
	if (accel_record_filling_block == NULL) {
		return -ENOMEM;
	}

	return 0;
}

int accel_recorder_consume_fifo(const uint8_t *fifo_data, size_t fifo_bytes,
				void *context)
{
	ARG_UNUSED(context);

	if ((fifo_data == NULL) || (fifo_bytes == 0U) ||
	    ((fifo_bytes % ACCEL_RECORD_FORMAT_SAMPLE_BYTES) != 0U)) {
		accel_record_report_fault(-EINVAL);
		return -EINVAL;
	}
	if (atomic_get(&accel_record_active) == 0 ||
	    atomic_get(&accel_record_failed) != 0) {
		return -EIO;
	}

	for (size_t offset = 0U; offset < fifo_bytes;
	     offset += ACCEL_RECORD_FORMAT_SAMPLE_BYTES) {
		struct accel_record_block *block;
		uint8_t marker = fifo_data[offset + 1U] & 1U;
		int ret;

		ret = accel_record_consume_fsync_marker(
			marker, accel_record_next_sample_sequence);
		if (ret != 0) {
			accel_record_report_fault(ret);
			return ret;
		}
		ret = accel_record_take_next_filling_block();
		if (ret == -ENOMEM) {
			accel_record_next_sample_sequence++;
			accel_record_dropped_samples++;
			continue;
		}
		if (ret != 0) {
			accel_record_report_fault(ret);
			return ret;
		}
		block = accel_record_filling_block;

		if (block->sample_count == 0U) {
			block->first_sample_sequence = accel_record_next_sample_sequence;
		}

		accel_record_format_store_fifo_sample(block->data,
					      block->sample_count,
					      &fifo_data[offset]);
		block->sample_count++;
		accel_record_next_sample_sequence++;

		if (block->sample_count != ACCEL_RECORD_FORMAT_SAMPLES_PER_BLOCK) {
			continue;
		}

		ret = accel_record_finalize_block(block);
		if (ret != 0) {
			accel_record_report_fault(ret);
			return ret;
		}
		block->sync_after_write =
			((accel_record_session_full_block_count %
			  ACCEL_RECORD_SYNC_INTERVAL_BLOCKS) == 0U);
		accel_record_filling_block = NULL;

		if (atomic_get(&accel_record_rotating) != 0) {
			k_spinlock_key_t key =
				k_spin_lock(&accel_record_state_lock);

			if (atomic_get(&accel_record_rotating) != 0) {
				block->pending_order = accel_record_pending_order++;
				atomic_set(&block->state,
					   ACCEL_RECORD_BLOCK_PENDING);
				k_spin_unlock(&accel_record_state_lock, key);
				continue;
			}
			k_spin_unlock(&accel_record_state_lock, key);
		}

		ret = accel_record_queue_current_chunk_block(block);
		if (ret != 0) {
			accel_record_report_fault(ret);
			return ret;
		}
	}

	return 0;
}

int accel_recorder_stop(void)
{
	struct accel_record_block *block;
	int ret;

	if (!accel_record_initialized) {
		return 0;
	}

	atomic_clear(&accel_record_active);
	ret = accel_record_wait_for_rotation();
	if (ret != 0) {
		block = accel_record_filling_block;
		accel_record_filling_block = NULL;
		accel_record_release_block(block);
		accel_record_release_pending_blocks();
		if (accel_record_file_open || accel_record_next_prepared) {
			(void)accel_record_submit_control(ACCEL_RECORD_CONTROL_ABORT);
		}
		return ret;
	}
	if (!accel_record_file_open && !accel_record_next_prepared) {
		return 0;
	}
	if (!accel_record_file_open) {
		return accel_record_submit_control(ACCEL_RECORD_CONTROL_CLOSE);
	}

	block = accel_record_filling_block;
	accel_record_filling_block = NULL;
	if (block != NULL && block->sample_count != 0U) {
		ret = accel_record_finalize_block(block);
		if (ret != 0) {
			accel_record_release_block(block);
			accel_record_report_fault(ret);
			return ret;
		}
		block->sync_after_write = true;
		ret = accel_record_queue_current_chunk_block(block);
		if (ret != 0) {
			accel_record_report_fault(ret);
			return ret;
		}
	} else {
		accel_record_release_block(block);
	}

	ret = accel_record_submit_control(ACCEL_RECORD_CONTROL_CLOSE);
	if (ret == 0) {
		LOG_INF("Accelerometer recording stopped");
		if (accel_record_dropped_samples != 0U) {
			LOG_WRN("Accelerometer recording dropped %u samples from RAM pressure",
				(unsigned int)accel_record_dropped_samples);
		}
	}

	return ret;
}

int accel_recorder_abort(void)
{
	struct accel_record_block *block;
	int cleanup_ret;
	int ret;

	if (!accel_record_initialized) {
		return 0;
	}

	atomic_clear(&accel_record_active);
	ret = accel_record_wait_for_rotation();
	if (!accel_record_file_open && !accel_record_next_prepared) {
		return 0;
	}

	block = accel_record_filling_block;
	accel_record_filling_block = NULL;
	accel_record_release_block(block);
	accel_record_release_pending_blocks();
	cleanup_ret = accel_record_submit_control(ACCEL_RECORD_CONTROL_ABORT);
	return ret != 0 ? ret : cleanup_ret;
}
