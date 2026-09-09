#include "ecgRecorder.h"

#include "BLEService.h"
#include "drivers/ecg/max30001.h"
#include "ecgRecordFormat.h"
#include "zephyrfilesystem.h"
#include "msense_sensor_stream.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ecg_recorder, CONFIG_LOG_LEVEL_MAX30001);

#define ECG_RECORD_MAX30001_NODE DT_ALIAS(ecg)

#define ECG_RECORD_THREAD_STACK_SIZE 4096
#define ECG_RECORD_THREAD_PRIORITY 5
#define ECG_RECORD_ANCHOR_TIMEOUT_MS 100
#define ECG_RECORD_BUFFER_COUNT 4U
#define ECG_RECORD_SYNC_INTERVAL_BLOCKS 8U
#define ECG_RECORD_PATH_MAX 96U
#define ECG_RECORD_STOP_CAPTURE_TIMEOUT_MS 3500U
#define ECG_RECORD_STORAGE_TIMEOUT_MS 30000U
#define ECG_RECORD_STOP_WAIT_TIMEOUT_MS \
	(ECG_RECORD_STOP_CAPTURE_TIMEOUT_MS + ECG_RECORD_STORAGE_TIMEOUT_MS)

BUILD_ASSERT(MSENSE_ECG_FILE_HEADER_BYTES <= FILESYSTEM_SCRATCH_BYTES,
	     "Filesystem scratch page must hold ECG metadata");

static const struct gpio_dt_spec ecg_intb =
	GPIO_DT_SPEC_GET(ECG_RECORD_MAX30001_NODE, intb_gpios);
static const struct gpio_dt_spec ecg_intb2 =
	GPIO_DT_SPEC_GET(ECG_RECORD_MAX30001_NODE, intb2_gpios);

static K_SEM_DEFINE(ecg_record_start_sem, 0, 1);
static K_SEM_DEFINE(ecg_record_started_sem, 0, 1);
static K_SEM_DEFINE(ecg_fifo_sem, 0, 1);
static K_SEM_DEFINE(ecg_record_stopped_sem, 0, 1);
static K_SEM_DEFINE(ecg_anchor_sem, 0, 1);

static struct gpio_callback ecg_intb_callback;
static struct gpio_callback ecg_intb2_callback;
static atomic_t ecg_record_requested;
static atomic_t ecg_record_stop_requested;
static atomic_t ecg_record_active;
static atomic_t ecg_record_last_error;
static atomic_t ecg_record_start_result;
/*
 * Set for each submitted start request and cleared only after the caller has
 * consumed that request's terminal stopped acknowledgement.  This is not a
 * second recorder state machine: it closes the cancellation-before-start
 * race in the existing start/stopped semaphore protocol.
 */
static atomic_t ecg_record_stop_confirmation_pending;
static bool ecg_intb_callback_added;
static bool ecg_intb2_callback_added;
static atomic_t ecg_anchor_state;
static atomic_t ecg_anchor_error;
static uint32_t ecg_anchor_rtc_tick;
static uint32_t ecg_next_rtc_tick;
static uint32_t ecg_next_sample_index;
static uint64_t ecg_record_session_id;
static ecg_recorder_fault_handler_t ecg_record_fault_handler;
static void *ecg_record_fault_context;

enum ecg_record_block_state {
	ECG_RECORD_BLOCK_FREE,
	ECG_RECORD_BLOCK_FILLING,
	ECG_RECORD_BLOCK_QUEUED,
	ECG_RECORD_BLOCK_WRITING,
};

struct ecg_record_block {
	struct k_work work;
	uint8_t data[MSENSE_ECG_BLOCK_BYTES];
	atomic_t state;
	uint16_t sample_count;
	uint32_t first_rtc_tick;
	uint32_t first_sample_index;
	bool sync_after_write;
};

static struct ecg_record_block ecg_record_blocks[ECG_RECORD_BUFFER_COUNT] __aligned(4);
static struct ecg_record_block *ecg_record_filling_block;
static struct fs_file_t ecg_record_file;
static struct k_work ecg_record_control_work;
static struct k_work ecg_record_prepare_work;
static struct k_sem ecg_record_control_done;
static bool ecg_record_writer_initialized;
static bool ecg_record_file_open;
static bool ecg_record_next_prepared;
static atomic_t ecg_record_writer_error;
static uint32_t ecg_record_dropped_samples;
static uint32_t ecg_record_chunk_index;
static uint32_t ecg_record_chunk_block_count;
static uint32_t ecg_record_session_full_block_count;
static char ecg_record_path[ECG_RECORD_PATH_MAX];
static char ecg_record_next_path[ECG_RECORD_PATH_MAX];

enum ecg_record_control_operation {
	ECG_RECORD_CONTROL_NONE,
	ECG_RECORD_CONTROL_OPEN,
	ECG_RECORD_CONTROL_CLOSE,
	ECG_RECORD_CONTROL_ABORT,
};

static enum ecg_record_control_operation ecg_record_control_operation;
static int ecg_record_control_result;

static int ecg_record_activate_next_chunk(void);
static void ecg_record_control_work_handler(struct k_work *work);
static void ecg_record_prepare_work_handler(struct k_work *work);

enum ecg_record_anchor_state {
	ECG_RECORD_ANCHOR_IDLE = 0,
	ECG_RECORD_ANCHOR_WAITING,
	ECG_RECORD_ANCHOR_CAPTURING,
	ECG_RECORD_ANCHOR_CAPTURED,
	ECG_RECORD_ANCHOR_ERROR,
};


static void ecg_record_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(ecg_record_thread_id,
		ECG_RECORD_THREAD_STACK_SIZE,
		ecg_record_thread,
		NULL,
		NULL,
		NULL,
		ECG_RECORD_THREAD_PRIORITY,
		0,
		0);

/**
 * @brief GPIO interrupt handler for the MAX30001 INTB line.
 *
 * Runs in interrupt context when the MAX30001 asserts INTB (the ECG FIFO has
 * reached its configured threshold). It does the minimum possible work:
 * giving ecg_fifo_sem to wake the recorder thread, which performs the actual
 * SPI FIFO drain in thread context where blocking is allowed.
 *
 * @param port GPIO port device (unused).
 * @param cb   Callback structure (unused).
 * @param pins Bitmask of triggering pins (unused).
 */
static void ecg_record_intb_handler(const struct device *port,
				    struct gpio_callback *cb,
				    uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_sem_give(&ecg_fifo_sem);
}

/**
 * @brief Latch the RTC tick for the first post-SYNCH ECG SAMP pulse.
 *
 * INT2B is configured as a self-clearing SAMP output. The MAX30001 emits the
 * pulse when the corresponding filtered ECG sample is placed in the FIFO,
 * making this ISR the only point where the FIFO sample timeline is tied to
 * the collection RTC. The atomic state transition rejects any extra pulses
 * that arrive before the recorder thread has masked SAMP at the sensor.
 */
static void ecg_record_intb2_handler(const struct device *port,
				     struct gpio_callback *cb,
				     uint32_t pins)
{
	uint32_t rtc_tick;
	int ret;

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (!atomic_cas(&ecg_anchor_state, ECG_RECORD_ANCHOR_WAITING,
			ECG_RECORD_ANCHOR_CAPTURING)) {
		return;
	}

	ret = rtc0_collection_counter_get(&rtc_tick);
	if (ret != 0) {
		atomic_set(&ecg_anchor_error, ret);
		atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_ERROR);
	} else {
		ecg_anchor_rtc_tick = rtc_tick;
		atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_CAPTURED);
	}

	k_sem_give(&ecg_anchor_sem);
}

/**
 * @brief Prepare the MAX30001 INTB GPIO for use as a data-ready interrupt.
 *
 * Configures the INTB pin (taken from the max30001 devicetree node) as an
 * input and registers ecg_record_intb_handler() as its callback. The
 * callback is only added once per boot (tracked by ecg_intb_callback_added)
 * so repeated start/stop cycles do not accumulate duplicate callbacks. The
 * pin interrupt itself is left DISABLED; ecg_record_run() enables edge
 * triggering only after the sensor is fully configured, avoiding spurious
 * wakeups during bring-up.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the GPIO controller is not ready.
 * @retval Other negative errno from GPIO configuration calls.
 */
static int ecg_record_configure_intb(void)
{
	int ret;

	if (!gpio_is_ready_dt(&ecg_intb)) {
		LOG_ERR("MAX30001 INTB GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&ecg_intb, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	if (!ecg_intb_callback_added) {
		gpio_init_callback(&ecg_intb_callback,
				   ecg_record_intb_handler,
				   BIT(ecg_intb.pin));

		ret = gpio_add_callback(ecg_intb.port, &ecg_intb_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

/**
 * @brief Prepare the MAX30001 INT2B GPIO for the one-shot SAMP anchor.
 */
static int ecg_record_configure_intb2(void)
{
	int ret;

	if (!gpio_is_ready_dt(&ecg_intb2)) {
		LOG_ERR("MAX30001 INT2B GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&ecg_intb2, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	if (!ecg_intb2_callback_added) {
		gpio_init_callback(&ecg_intb2_callback,
				   ecg_record_intb2_handler,
				   BIT(ecg_intb2.pin));

		ret = gpio_add_callback(ecg_intb2.port, &ecg_intb2_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb2_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_DISABLE);
}

static void ecg_record_disable_gpio_interrupts(void)
{
	(void)gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_DISABLE);
	(void)gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

static int ecg_record_wait_for_anchor(uint32_t *rtc_tick)
{
	int ret;

	ret = k_sem_take(&ecg_anchor_sem, K_MSEC(ECG_RECORD_ANCHOR_TIMEOUT_MS));
	if (atomic_get(&ecg_record_requested) == 0) {
		return -ECANCELED;
	}
	if (ret != 0) {
		return -ETIMEDOUT;
	}

	if (atomic_get(&ecg_anchor_state) == ECG_RECORD_ANCHOR_CAPTURED) {
		*rtc_tick = ecg_anchor_rtc_tick;
		return 0;
	}
	if (atomic_get(&ecg_anchor_state) == ECG_RECORD_ANCHOR_ERROR) {
		return (int)atomic_get(&ecg_anchor_error);
	}

	return -EIO;
}

static void ecg_record_report_writer_fault(int error)
{
	if (atomic_cas(&ecg_record_writer_error, 0, error == 0 ? -EIO : error)) {
		msense_sensor_stream_storage_failed(error == 0 ? -EIO : error);
		LOG_ERR("ECG block storage failed: %d", error == 0 ? -EIO : error);
	}
}

static void ecg_record_block_work_handler(struct k_work *work)
{
	struct ecg_record_block *block =
		CONTAINER_OF(work, struct ecg_record_block, work);
	ssize_t written;
	int ret = 0;

	atomic_set(&block->state, ECG_RECORD_BLOCK_WRITING);
	if (atomic_get(&ecg_record_writer_error) != 0) {
		ret = -ECANCELED;
	} else if (!ecg_record_file_open) {
		ret = -EIO;
	} else {
		if (ecg_record_chunk_block_count == MSENSE_ECG_FILE_DATA_BLOCKS) {
			ret = ecg_record_activate_next_chunk();
		}
	}
	if (ret == 0) {
		written = fs_write(&ecg_record_file, block->data, MSENSE_ECG_BLOCK_BYTES);
		if (written != (ssize_t)MSENSE_ECG_BLOCK_BYTES) {
			ret = written < 0 ? (int)written : -EIO;
		} else {
			ecg_record_chunk_block_count++;
			if (block->sync_after_write) {
				ret = fs_sync(&ecg_record_file);
			}
		}
	}

	if (ret != 0) {
		ecg_record_report_writer_fault(ret);
	}
	atomic_set(&block->state, ECG_RECORD_BLOCK_FREE);
}

static void ecg_record_writer_initialize(void)
{
	size_t index;

	if (ecg_record_writer_initialized) {
		return;
	}

	for (index = 0U; index < ARRAY_SIZE(ecg_record_blocks); index++) {
		k_work_init(&ecg_record_blocks[index].work, ecg_record_block_work_handler);
		atomic_set(&ecg_record_blocks[index].state, ECG_RECORD_BLOCK_FREE);
	}
	k_work_init(&ecg_record_control_work, ecg_record_control_work_handler);
	k_work_init(&ecg_record_prepare_work, ecg_record_prepare_work_handler);
	k_sem_init(&ecg_record_control_done, 0, 1);
	ecg_record_writer_initialized = true;
}

static struct ecg_record_block *ecg_record_take_free_block(void)
{
	size_t index;

	for (index = 0U; index < ARRAY_SIZE(ecg_record_blocks); index++) {
		if (atomic_cas(&ecg_record_blocks[index].state, ECG_RECORD_BLOCK_FREE,
			       ECG_RECORD_BLOCK_FILLING)) {
			memset(ecg_record_blocks[index].data, 0, sizeof(ecg_record_blocks[index].data));
			ecg_record_blocks[index].sample_count = 0U;
			ecg_record_blocks[index].first_rtc_tick = 0U;
			ecg_record_blocks[index].first_sample_index = 0U;
			ecg_record_blocks[index].sync_after_write = false;
			return &ecg_record_blocks[index];
		}
	}

	return NULL;
}

static int ecg_record_open_current_chunk(void)
{
	uint8_t *metadata = filesystem_scratch_buffer();
	int ret;

	ret = filesystem_make_recording_chunk_path(ecg_record_path, sizeof(ecg_record_path),
						  "ecg", ecg_record_session_id,
						  ecg_record_chunk_index);
	if (ret != 0) {
		return ret;
	}
	msense_ecg_file_header_build(metadata, ecg_record_session_id,
					     ecg_record_chunk_index);
	ret = filesystem_preallocate_file(
		&ecg_record_file, ecg_record_path, MSENSE_ECG_FILE_BYTES,
		metadata, MSENSE_ECG_FILE_HEADER_BYTES, true);
	if (ret != 0) {
		return ret;
	}
	ecg_record_file_open = true;
	LOG_INF("ECG recording to %s", ecg_record_path);
	return 0;
}

static int ecg_record_prepare_next_chunk(void)
{
	struct fs_file_t file;
	uint8_t *metadata = filesystem_scratch_buffer();
	uint32_t next_index = ecg_record_chunk_index + 1U;
	int ret;

	ret = filesystem_make_recording_chunk_path(
		ecg_record_next_path, sizeof(ecg_record_next_path), "ecg",
		ecg_record_session_id, next_index);
	if (ret != 0) {
		return ret;
	}
	msense_ecg_file_header_build(metadata,
				     ecg_record_session_id, next_index);
	ret = filesystem_preallocate_file(
		&file, ecg_record_next_path, MSENSE_ECG_FILE_BYTES,
		metadata, MSENSE_ECG_FILE_HEADER_BYTES, false);
	if (ret == 0) {
		ecg_record_next_prepared = true;
	}
	return ret;
}

static int ecg_record_close_current_chunk(bool sync_before_close)
{
	int ret = 0;
	int close_ret;

	if (!ecg_record_file_open) {
		return 0;
	}
	if (sync_before_close) {
		ret = fs_sync(&ecg_record_file);
	}

	close_ret = fs_close(&ecg_record_file);
	ecg_record_file_open = false;
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}
	return ret;
}

static int ecg_record_activate_next_chunk(void)
{
	int ret;

	if (!ecg_record_next_prepared) {
		return -ENOSPC;
	}
	ret = ecg_record_close_current_chunk(true);
	if (ret != 0) {
		return ret;
	}

	ecg_record_chunk_index++;
	ecg_record_chunk_block_count = 0U;
	ret = filesystem_open_preallocated_file(
		&ecg_record_file, ecg_record_next_path,
		MSENSE_ECG_FILE_HEADER_BYTES);
	if (ret != 0) {
		return ret;
	}
	ecg_record_file_open = true;
	strcpy(ecg_record_path, ecg_record_next_path);
	ecg_record_next_prepared = false;
	ret = k_work_submit_to_queue(&my_work_q, &ecg_record_prepare_work);
	return ret < 0 ? ret : 0;
}

static void ecg_record_prepare_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);
	if (!ecg_record_file_open || ecg_record_next_prepared ||
	    atomic_get(&ecg_record_writer_error) != 0) {
		return;
	}
	ret = ecg_record_prepare_next_chunk();
	if (ret != 0) {
		ecg_record_report_writer_fault(ret);
	}
}

static void ecg_record_control_work_handler(struct k_work *work)
{
	enum ecg_record_control_operation operation = ecg_record_control_operation;
	int ret = 0;

	ARG_UNUSED(work);
	switch (operation) {
	case ECG_RECORD_CONTROL_OPEN:
		ret = ecg_record_open_current_chunk();
		if (ret == 0) {
			ret = ecg_record_prepare_next_chunk();
		}
		if (ret != 0 && ecg_record_file_open) {
			(void)ecg_record_close_current_chunk(false);
			(void)fs_unlink(ecg_record_path);
		}
		break;
	case ECG_RECORD_CONTROL_CLOSE:
		ret = ecg_record_close_current_chunk(true);
		if (ret == 0) {
			ret = atomic_get(&ecg_record_writer_error);
		}
		break;
	case ECG_RECORD_CONTROL_ABORT:
		ret = ecg_record_close_current_chunk(false);
		break;
	case ECG_RECORD_CONTROL_NONE:
	default:
		ret = -EINVAL;
		break;
	}
	if (operation != ECG_RECORD_CONTROL_OPEN &&
	    ecg_record_next_prepared) {
		int unlink_ret = fs_unlink(ecg_record_next_path);

		if (unlink_ret != 0 && ret == 0) {
			ret = unlink_ret;
		}
		ecg_record_next_prepared = false;
	}
	ecg_record_control_result = ret;
	k_sem_give(&ecg_record_control_done);
}

static int ecg_record_submit_control(enum ecg_record_control_operation operation)
{
	int ret;

	k_sem_reset(&ecg_record_control_done);
	ecg_record_control_operation = operation;
	ret = k_work_submit_to_queue(&my_work_q, &ecg_record_control_work);
	if (ret <= 0) {
		return ret < 0 ? ret : -EALREADY;
	}
	ret = k_sem_take(&ecg_record_control_done,
			 K_FOREVER);
	return ret == 0 ? ecg_record_control_result : ret;
}

static int ecg_record_take_filling_block(void)
{
	if (ecg_record_filling_block != NULL) {
		return 0;
	}
	ecg_record_filling_block = ecg_record_take_free_block();
	return ecg_record_filling_block == NULL ? -ENOMEM : 0;
}

static int ecg_record_queue_finalized_block(struct ecg_record_block *block)
{
	int ret;

	block->sync_after_write =
		(ecg_record_session_full_block_count % ECG_RECORD_SYNC_INTERVAL_BLOCKS) == 0U;
	atomic_set(&block->state, ECG_RECORD_BLOCK_QUEUED);
	ret = k_work_submit_to_queue(&my_work_q, &block->work);
	if (ret <= 0) {
		atomic_set(&block->state, ECG_RECORD_BLOCK_FREE);
		return ret < 0 ? ret : -EALREADY;
	}

	return 0;
}

static int ecg_record_finalize_filling_block(void)
{
	struct ecg_record_block *block = ecg_record_filling_block;
	int ret;

	if (block == NULL || block->sample_count == 0U) {
		return 0;
	}
	ret = msense_ecg_block_finalize(block->data, block->sample_count);
	if (ret != 0) {
		atomic_set(&block->state, ECG_RECORD_BLOCK_FREE);
		ecg_record_filling_block = NULL;
		return ret;
	}
	if (block->sample_count == MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK) {
		ecg_record_session_full_block_count++;
	}
	ret = msense_sensor_stream_accept_record(block->data, MSENSE_ECG_BLOCK_BYTES);
	if (ret != 0) {
		atomic_set(&block->state, ECG_RECORD_BLOCK_FREE);
		ecg_record_filling_block = NULL;
		return ret;
	}
	ret = ecg_record_queue_finalized_block(block);
	if (ret != 0) {
		ecg_record_report_writer_fault(ret);
		atomic_set(&block->state, ECG_RECORD_BLOCK_FREE);
		ecg_record_filling_block = NULL;
		return ret;
	}
	ecg_record_filling_block = NULL;
	return 0;
}

static int ecg_record_finish_file(bool normal_stop)
{
	if (!normal_stop && ecg_record_filling_block != NULL) {
		atomic_set(&ecg_record_filling_block->state, ECG_RECORD_BLOCK_FREE);
		ecg_record_filling_block = NULL;
	}
	return ecg_record_submit_control(normal_stop ? ECG_RECORD_CONTROL_CLOSE :
					       ECG_RECORD_CONTROL_ABORT);
}

/* Append one time-valid FIFO word to the current immutable ECB2 page. */
static int ecg_record_store_sample(const struct max30001_ecg_sample *sample)
{
	struct ecg_record_block *block;
	int ret;

	if (sample == NULL || !sample->time_valid) {
		return -EINVAL;
	}
	ret = ecg_record_take_filling_block();
	if (ret != 0) {
		return ret;
	}
	block = ecg_record_filling_block;
	if (block->sample_count == 0U) {
		block->first_rtc_tick = ecg_next_rtc_tick;
		block->first_sample_index = ecg_next_sample_index;
		msense_ecg_block_begin(block->data, block->first_rtc_tick,
				       block->first_sample_index);
	}

	ret = msense_ecg_block_append_sample(block->data, block->sample_count, sample->raw);
	if (ret != 0) {
		return ret;
	}
	block->sample_count++;
	ecg_next_rtc_tick++;
	ecg_next_sample_index++;
	if (block->sample_count != MSENSE_ECG_BLOCK_SAMPLES_PER_FULL_BLOCK) {
		return 0;
	}

	return ecg_record_finalize_filling_block();
}

/**
 * @brief Store a batch of ECG samples read from the FIFO.
 *
 * Iterates over a batch produced by max30001_ecg_read_fifo(), skipping any
 * sample whose time_valid flag is false (words that do not represent a real
 * sample slot), and writes each remaining sample to storage via
 * ecg_record_store_sample(). The RTC tick advances once per stored time-valid
 * sample, independent of how many samples were batched before this drain.
 *
 * @param samples Array of decoded samples.
 * @param count   Number of valid entries in samples.
 */
static int ecg_record_process_samples(const struct max30001_ecg_sample *samples,
				      size_t count)
{
	int ret;
	size_t index;

	for (index = 0U; index < count; index++) {
		if (atomic_get(&ecg_record_stop_requested) != 0 &&
		    ecg_record_filling_block == NULL) {
			break;
		}
		if (!samples[index].time_valid) {
			continue;
		}

		ret = ecg_record_store_sample(&samples[index]);
		if (ret == -ENOMEM) {
			ecg_next_rtc_tick++;
			ecg_next_sample_index++;
			ecg_record_dropped_samples++;
			continue;
		}
		if (ret != 0) {
			return ret;
		}
		if (atomic_get(&ecg_record_stop_requested) != 0 &&
		    ecg_record_filling_block == NULL) {
			break;
		}
	}

	return 0;
}



/**
 * @brief Empty the MAX30001 FIFO and dispatch the samples.
 *
 * Called from the recorder thread each time the INTB interrupt fires. Reads
 * the FIFO in bursts of up to
 * MAX30001_ECG_FIFO_MAX_SAMPLES, for at most 4 passes, stopping early when a
 * read returns fewer than a full burst or ends on an EOF-tagged sample —
 * both signs the FIFO is empty. The pass limit bounds time spent here if
 * samples arrive as fast as they are drained.
 *
 * Each burst is persisted via ecg_record_process_samples(). No IMU or
 * collection-progress data is transmitted over BLE from this path.
 *
 * FIFO read errors are returned to the recording-session supervisor. A FIFO
 * overflow means the sample timeline has a gap, so the recording session
 * ends rather than fabricating consecutive timestamps across the loss.
 */
static int ecg_record_drain_fifo(void)
{
	int ret;
	int pass;

	ret = atomic_get(&ecg_record_writer_error);
	if (ret != 0) {
		return ret;
	}

	for (pass = 0; pass < 4; pass++) {
		struct max30001_ecg_sample samples[MAX30001_ECG_FIFO_MAX_SAMPLES];
		size_t count = 0;

		if (atomic_get(&ecg_record_stop_requested) != 0 &&
		    ecg_record_filling_block == NULL) {
			return 0;
		}

		ret = max30001_ecg_read_fifo(samples, ARRAY_SIZE(samples), &count);
		if (ret != 0) {
			LOG_ERR("MAX30001 ECG FIFO read failed: %d", ret);
			return ret;
		}

		if (count == 0) {
			return 0;
		}

		ret = ecg_record_process_samples(samples, count);
		if (ret != 0) {
			return ret;
		}
		if (atomic_get(&ecg_record_stop_requested) != 0 &&
		    ecg_record_filling_block == NULL) {
			return 0;
		}

		if (count < ARRAY_SIZE(samples) || samples[count - 1].eof) {
			return 0;
		}
	}

	return 0;
}

/**
 * @brief Execute one complete ECG recording session.
 *
 * The main body of a recording configures both MAX30001 interrupt GPIOs and
 * captures the first post-SYNCH SAMP pulse on INT2B. That pulse provides the
 * RTC tick for the first FIFO sample; every later time-valid FIFO sample is
 * assigned the next tick. Once anchored, SAMP is masked and only the 16-word
 * FIFO watermark interrupt remains active during steady-state recording.
 *
 * On a normal stop, acquisition stays enabled until the current full block
 * boundary, then interrupts are disabled and the sensor is powered down.
 * Setup and FIFO failures discard the incomplete block so no sample is written
 * without a valid, continuous timing base. Requires the NAND filesystem to be
 * ready before starting.
 *
 * @retval 0 on a clean stop.
 * @retval -ENODEV if the filesystem is not ready.
 * @retval Other negative errno if sensor or GPIO setup fails.
 */
static int ecg_record_run(void)
{
	int ret;
	int stop_ret;
	int close_ret;
	int64_t stop_deadline_ms = 0;
	uint32_t rtc_tick;
	bool sensor_configured = false;
	bool anchor_ready = false;
	bool start_reported = false;
	bool writer_open = false;
	bool normal_stop = false;

	if (!file_system_ready) {
		LOG_ERR("Filesystem is not ready for ECG recording");
		ret = -ENODEV;
		goto out;
	}
	ecg_record_writer_initialize();
	if (ecg_record_file_open) {
		ret = -EALREADY;
		goto out;
	}
	atomic_clear(&ecg_record_writer_error);
	atomic_clear(&ecg_record_stop_requested);
	ecg_record_filling_block = NULL;
	ecg_record_chunk_index = 0U;
	ecg_record_chunk_block_count = 0U;
	ecg_record_session_full_block_count = 0U;
	ecg_record_dropped_samples = 0U;
	ecg_record_next_prepared = false;
	ret = ecg_record_submit_control(ECG_RECORD_CONTROL_OPEN);
	if (ret != 0) {
		LOG_ERR("Failed to create ECG recording chunk: %d", ret);
		goto out;
	}
	writer_open = true;

	ret = ecg_record_configure_intb();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INTB: %d", ret);
		goto out;
	}

	ret = ecg_record_configure_intb2();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INT2B: %d", ret);
		goto out;
	}

	k_sem_reset(&ecg_fifo_sem);
	k_sem_reset(&ecg_anchor_sem);
	atomic_set(&ecg_anchor_error, 0);
	atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_WAITING);
	ecg_anchor_rtc_tick = 0U;
	ecg_next_rtc_tick = 0U;
	ecg_next_sample_index = 0U;

	/* Fail before touching the sensor if collection timing is not active. */
	ret = rtc0_collection_counter_get(&rtc_tick);
	if (ret != 0) {
		LOG_ERR("Collection RTC is unavailable for ECG timing: %d", ret);
		goto out;
	}

	sensor_configured = true;
	ret = max30001_ecg_init_512();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG init failed: %d", ret);
		goto out;
	}

	/* Both GPIOs are armed while their MAX30001 sources remain masked. */
	ret = gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable MAX30001 INTB interrupt: %d", ret);
		goto out;
	}

	ret = gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable MAX30001 INT2B interrupt: %d", ret);
		goto out;
	}

	ret = max30001_ecg_start();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG start failed: %d", ret);
		goto out;
	}

	ret = ecg_record_wait_for_anchor(&rtc_tick);
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG SAMP anchor failed: %d", ret);
		goto out;
	}
	anchor_ready = true;
	ecg_next_rtc_tick = rtc_tick;

	/* Disable the MCU edge first, then remove SAMP from the sensor output. */
	ret = gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_DISABLE);
	if (ret != 0) {
		LOG_ERR("Failed to disable MAX30001 INT2B interrupt: %d", ret);
		goto out;
	}

	ret = max30001_ecg_disable_samp_interrupt();
	if (ret != 0) {
		LOG_ERR("Failed to mask MAX30001 SAMP interrupt: %d", ret);
		goto out;
	}

	k_sem_give(&ecg_fifo_sem);
	LOG_INF("ECG NAND recording active, first sample RTC tick=%u",
		(unsigned int)ecg_next_rtc_tick);
	atomic_set(&ecg_record_start_result, 0);
	k_sem_give(&ecg_record_started_sem);
	start_reported = true;

	while (atomic_get(&ecg_record_requested) != 0) {
		ret = atomic_get(&ecg_record_writer_error);
		if (ret != 0) {
			goto out;
		}
		if (atomic_get(&ecg_record_stop_requested) != 0 &&
		    ecg_record_filling_block == NULL) {
			normal_stop = true;
			atomic_clear(&ecg_record_requested);
			break;
		}
		if (atomic_get(&ecg_record_stop_requested) != 0 && stop_deadline_ms == 0) {
			stop_deadline_ms = k_uptime_get() + ECG_RECORD_STOP_CAPTURE_TIMEOUT_MS;
		}
		ret = k_sem_take(&ecg_fifo_sem, K_SECONDS(1));
		if (ret == 0) {
			ret = ecg_record_drain_fifo();
			if (ret != 0) {
				goto out;
			}
		}
		if (atomic_get(&ecg_record_stop_requested) != 0 &&
		    ecg_record_filling_block == NULL) {
			normal_stop = true;
			atomic_clear(&ecg_record_requested);
			break;
		}
		if (stop_deadline_ms != 0 && k_uptime_get() >= stop_deadline_ms) {
			LOG_ERR("ECG normal-stop boundary timed out after %u ms",
				ECG_RECORD_STOP_CAPTURE_TIMEOUT_MS);
			ret = -ETIMEDOUT;
			goto out;
		}
	}
	ret = 0;

out:
	ecg_record_disable_gpio_interrupts();
	if (sensor_configured) {
		stop_ret = max30001_ecg_stop();
		if (ret == 0 && stop_ret != 0) {
			msense_sensor_stream_recording_failed(stop_ret);
			ret = stop_ret;
		}
	}
	if (writer_open) {
		if (ret == 0 && anchor_ready && normal_stop) {
			ret = ecg_record_finish_file(true);
		} else {
			close_ret = ecg_record_finish_file(false);

			if (ret == 0 && close_ret != 0) {
				ret = close_ret;
			}
		}
		writer_open = false;
	}
	if (ecg_record_dropped_samples != 0U) {
		LOG_WRN("ECG recording dropped %u samples from RAM pressure",
			(unsigned int)ecg_record_dropped_samples);
	}
	atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_IDLE);
	if (!start_reported) {
		atomic_set(&ecg_record_start_result, ret);
		k_sem_give(&ecg_record_started_sem);
	}

	return ret;
}

static void ecg_record_request_stop(bool complete_current_block)
{
	if (complete_current_block) {
		atomic_set(&ecg_record_stop_requested, 1);
	} else {
		atomic_clear(&ecg_record_stop_requested);
		atomic_clear(&ecg_record_requested);
	}
	k_sem_give(&ecg_anchor_sem);
	k_sem_give(&ecg_fifo_sem);
}

static int ecg_record_wait_for_stop_confirmation(void)
{
	int ret;

	ret = k_sem_take(&ecg_record_stopped_sem, K_MSEC(ECG_RECORD_STOP_WAIT_TIMEOUT_MS));
	if (ret != 0) {
		return ret;
	}

	atomic_clear(&ecg_record_stop_confirmation_pending);
	return 0;
}

/**
 * @brief Dedicated recorder thread: supervises ECG recording sessions.
 *
 * Created at boot by K_THREAD_DEFINE and never exits. It blocks on
 * ecg_record_start_sem until ecg_recorder_start() signals a session, then
 * invokes ecg_record_run() when that request is still live. Around each
 * consumed start request it maintains ecg_record_active for diagnostics and
 * gives ecg_record_stopped_sem, which is the sole stop-quiescence proof.
 *
 * If a session ends while a recording is still requested (i.e. it aborted on
 * error rather than being stopped), the request is cleared and the optional
 * collection fault handler is notified so the application can retain MSC
 * medium absence rather than retrying against an uncertain filesystem state.
 *
 * @param arg1 Unused.
 * @param arg2 Unused.
 * @param arg3 Unused.
 */
static void ecg_record_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&ecg_record_start_sem, K_FOREVER);

		/*
		 * Every consumed start token receives one terminal stopped
		 * acknowledgement, including one cancelled before this thread ran.
		 */
		int ret = 0;

		atomic_set(&ecg_record_active, 1);
		if (atomic_get(&ecg_record_requested) != 0) {
			ret = ecg_record_run();
			if (atomic_get(&ecg_record_requested) == 0 && ret == -ECANCELED) {
				ret = 0;
			}
		}

		if (atomic_get(&ecg_record_requested) != 0) {
			LOG_ERR("ECG recording stopped after error: %d", ret);
			atomic_clear(&ecg_record_requested);
			if (atomic_get(&ecg_record_writer_error) == 0) {
				msense_sensor_stream_recording_failed(ret);
			}
			if (atomic_get(&ecg_record_start_result) == 0 &&
			    ecg_record_fault_handler != NULL) {
				ecg_record_fault_handler(ecg_record_fault_context);
			}
		}

		atomic_set(&ecg_record_last_error, ret);
		atomic_clear(&ecg_record_active);
		k_sem_give(&ecg_record_stopped_sem);
	}
}

/**
 * @brief Request that ECG recording begin (public API).
 *
 * Synchronous start-confirmation entry point called from application code
 * (e.g. when the user enters ECG collection mode). After confirming the
 * filesystem is ready, it wakes the recorder thread and waits for that thread
 * to report either a configured recording path or its setup failure. A failed
 * confirmation cancels the submitted request and requires the corresponding
 * terminal stopped acknowledgement before it returns an ordinary setup error.
 *
 * @param session_id Collection ID recorded in the ECF2 header and filename.
 * @retval 0 if recording was started.
 * @retval -ENODEV if the filesystem is not ready.
 * @retval -EBUSY if a prior submitted request has no consumed terminal stop
 *         acknowledgement.
 * @retval Other negative errno from recorder setup or start confirmation.
 */
int ecg_recorder_start(uint64_t session_id)
{
	int start_ret;
	int stop_ret;
	int ret;

	if (!file_system_ready) {
		return -ENODEV;
	}

	if (atomic_get(&ecg_record_stop_confirmation_pending) != 0) {
		return -EBUSY;
	}

	if (atomic_cas(&ecg_record_requested, 0, 1)) {
		ecg_record_session_id = session_id;
		k_sem_reset(&ecg_record_stopped_sem);
		k_sem_reset(&ecg_record_started_sem);
		atomic_set(&ecg_record_stop_confirmation_pending, 1);
		atomic_clear(&ecg_record_last_error);
		atomic_set(&ecg_record_start_result, -EINPROGRESS);
		k_sem_give(&ecg_record_start_sem);
		ret = k_sem_take(&ecg_record_started_sem, K_FOREVER);
		if (ret != 0) {
			start_ret = ret;
			ecg_record_request_stop(false);
			stop_ret = ecg_record_wait_for_stop_confirmation();
			return (stop_ret != 0) ? stop_ret : start_ret;
		}

		ret = atomic_get(&ecg_record_start_result);
		if (ret != 0) {
			start_ret = ret;
			ecg_record_request_stop(false);
			stop_ret = ecg_record_wait_for_stop_confirmation();
			return (stop_ret != 0) ? stop_ret : start_ret;
		}
	}

	return 0;
}

/**
 * @brief Request that ECG recording stop and wait for it to finish
 *        (public API).
 *
 * Requests a full-block boundary, then gives ecg_fifo_sem to wake the recorder
 * thread immediately rather than letting it wait out its 1-second semaphore
 * timeout. If no submitted session awaits a terminal acknowledgement the call
 * returns at once; otherwise it waits up to 6 seconds: 3.5 seconds for the
 * remaining 2.65-second sample block and 2.5 seconds for bounded writer drain,
 * sync, and sensor shutdown. It does not use a sampled active flag as proof of
 * recorder quiescence.
 *
 * @retval 0 once recording has stopped (or no acknowledgement was pending).
 * @retval -EAGAIN if the submitted session did not confirm shutdown within
 *         6 s.
 */
int ecg_recorder_stop(void)
{
	int ret;

	if (atomic_get(&ecg_record_stop_confirmation_pending) == 0) {
		return 0;
	}

	ecg_record_request_stop(true);
	ret = ecg_record_wait_for_stop_confirmation();
	if (ret != 0) {
		return ret;
	}

	return atomic_get(&ecg_record_last_error);
}

bool ecg_recorder_shutdown_confirmed(void)
{
	return atomic_get(&ecg_record_stop_confirmation_pending) == 0;
}

void ecg_recorder_set_fault_handler(ecg_recorder_fault_handler_t handler,
					     void *context)
{
	ecg_record_fault_handler = handler;
	ecg_record_fault_context = context;
}
