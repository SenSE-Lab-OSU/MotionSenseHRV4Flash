#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "zephyrfilesystem.h"

struct k_work_q my_work_q;
bool file_system_ready;
static uint32_t rtc0_test_counter;

int rtc0_collection_counter_get(uint32_t *ticks)
{
	if (ticks == NULL) {
		return -EINVAL;
	}

	*ticks = rtc0_test_counter;
	return 0;
}

int filesystem_make_recording_path(char *path, size_t path_size,
				   const char *stream_prefix,
				   uint64_t collection_id)
{
	ARG_UNUSED(path);
	ARG_UNUSED(path_size);
	ARG_UNUSED(stream_prefix);
	ARG_UNUSED(collection_id);
	return 0;
}

int filesystem_make_recording_chunk_path(char *path, size_t path_size,
					 const char *stream_prefix,
					 uint64_t collection_id,
					 uint32_t chunk_index)
{
	ARG_UNUSED(path);
	ARG_UNUSED(path_size);
	ARG_UNUSED(stream_prefix);
	ARG_UNUSED(collection_id);
	ARG_UNUSED(chunk_index);
	return 0;
}

#define CONFIG_LOG_LEVEL_ICM20948_ACCEL 3
#include "../../../src/accelRecorder.c"

static void reset_recorder_state(void)
{
	accel_record_initialize();
	atomic_clear(&accel_record_active);
	atomic_clear(&accel_record_failed);
	accel_record_file_open = false;
	accel_record_filling_block = NULL;
	accel_record_metadata_block = NULL;
	accel_record_session_full_block_count = 0U;
	accel_record_chunk_full_block_count = 0U;
	accel_record_chunk_data_bytes = 0U;
	accel_record_chunk_full = false;
	accel_record_pending_order = 0U;
	atomic_clear(&accel_record_rotating);
	rtc0_test_counter = 0U;
	for (size_t i = 0U; i < ARRAY_SIZE(accel_record_blocks); i++) {
		atomic_set(&accel_record_blocks[i].state, ACCEL_RECORD_BLOCK_FREE);
	}
}

ZTEST(accel_recorder, test_pool_transitions_and_no_free_block)
{
	struct accel_record_block *blocks[ACCEL_RECORD_BUFFER_COUNT];

	reset_recorder_state();
	for (size_t i = 0U; i < ARRAY_SIZE(blocks); i++) {
		blocks[i] = accel_record_take_free_block();
		zassert_not_null(blocks[i], "free block %u was unavailable", i);
		zassert_equal(atomic_get(&blocks[i]->state),
			      ACCEL_RECORD_BLOCK_FILLING, "block state transition failed");
	}
	zassert_is_null(accel_record_take_free_block(),
			"pool supplied a fifth 4 KiB buffer");
	for (size_t i = 0U; i < ARRAY_SIZE(blocks); i++) {
		atomic_set(&blocks[i]->state, ACCEL_RECORD_BLOCK_FREE);
	}
}

ZTEST(accel_recorder, test_fifo_batch_boundaries_and_endian_conversion)
{
	static const uint8_t fifo_batch[] = {
		0x12U, 0x34U, 0xfeU, 0xdcU, 0x80U, 0x01U,
		0x00U, 0x02U, 0xffU, 0xfeU, 0x7fU, 0xffU,
	};
	static const uint8_t expected_sample[] = {0x34U, 0x12U, 0xdcU,
						  0xfeU, 0x01U, 0x80U};
	struct accel_record_block *block;

	reset_recorder_state();
	block = accel_record_take_free_block();
	zassert_not_null(block, "missing filling block");
	accel_record_filling_block = block;
	rtc0_test_counter = 0x0042U;
	atomic_set(&accel_record_active, 1);

	zassert_ok(accel_recorder_consume_fifo(fifo_batch, sizeof(fifo_batch), NULL),
		   "valid FIFO batch failed");
	zassert_equal(block->sample_count, 2U, "wrong batch sample count");
	zassert_mem_equal(&block->data[16], expected_sample, sizeof(expected_sample),
			  "first sample was not encoded as little-endian");
	zassert_equal(block->reserved_timer_output, 0x0042U,
		      "block did not capture its RTC0 tick");
	zassert_equal(accel_recorder_consume_fifo(fifo_batch, 7U, NULL), -EINVAL,
		      "malformed FIFO batch was accepted");
}

ZTEST(accel_recorder, test_no_free_buffer_is_acquisition_failure)
{
	static const uint8_t fifo_sample[] = {0U, 1U, 0U, 2U, 0U, 3U};

	reset_recorder_state();
	atomic_set(&accel_record_active, 1);
	zassert_equal(accel_recorder_consume_fifo(fifo_sample, sizeof(fifo_sample), NULL),
		      -ENOMEM, "missing filling buffer was accepted");
	zassert_equal(atomic_get(&accel_record_failed), 1,
		      "missing buffer did not become an acquisition fault");
}

ZTEST_SUITE(accel_recorder, NULL, NULL, NULL, NULL, NULL);
