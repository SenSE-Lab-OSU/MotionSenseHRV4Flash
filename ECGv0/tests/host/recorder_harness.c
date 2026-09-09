/* I/O stubs around production recorder boundary/rotation functions. */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#define MSENSE_ECG_FILE_DATA_BLOCKS 1023U
#define ECG_RECORD_BLOCK_FREE 0
#define ECG_RECORD_BLOCK_QUEUED 2
#define ECG_RECORD_BLOCK_WRITING 3
#define ECG_RECORD_SYNC_INTERVAL_BLOCKS 8U
#define MSENSE_ECG_BLOCK_BYTES 4096U
#define CONTAINER_OF(pointer, type, member) ((type *)((char *)(pointer) - offsetof(type, member)))
struct k_work { int unused; };
struct ecg_record_block {
    int state;
    struct k_work work;
    uint8_t data[MSENSE_ECG_BLOCK_BYTES];
    unsigned sample_count;
    bool sync_after_write;
} block;
struct max30001_ecg_sample { bool time_valid; };
enum ecg_record_control_operation {
    ECG_RECORD_CONTROL_NONE,
    ECG_RECORD_CONTROL_OPEN,
    ECG_RECORD_CONTROL_CLOSE,
    ECG_RECORD_CONTROL_ABORT,
};
static struct ecg_record_block *ecg_record_filling_block;
static int ecg_record_stop_requested, ecg_record_writer_error, my_work_q, ecg_record_file;
static bool ecg_record_file_open;
static unsigned ecg_record_chunk_block_count, ecg_record_chunk_index;
static unsigned ecg_record_session_full_block_count;
static uint32_t ecg_next_rtc_tick, ecg_next_sample_index, ecg_record_dropped_samples;
static int stores, store_error, stop_on_store, writes, syncs, activations;
static int write_result, sync_error, activate_error, submit_result;
static enum ecg_record_control_operation last_control;

static int atomic_get(const int *value) { return *value; }
static void atomic_set(int *value, int next) { *value = next; }
static int k_work_submit_to_queue(int *queue, struct k_work *work)
{
    (void)queue; (void)work; return submit_result;
}
static int ecg_record_activate_next_chunk(void)
{
    activations++;
    if (!activate_error) {
        ecg_record_chunk_index++;
        ecg_record_chunk_block_count = 0;
    }
    return activate_error;
}
static int fs_write(int *file, const void *data, size_t size)
{
    (void)file; (void)data; writes++;
    return write_result ? write_result : (int)size;
}
static int fs_sync(int *file) { (void)file; syncs++; return sync_error; }
static struct ecg_record_block *ecg_record_take_free_block(void) { return &block; }
void ecg_record_report_writer_fault(int error)
{
    if (!ecg_record_writer_error) ecg_record_writer_error = error;
}
static int ecg_record_submit_control(enum ecg_record_control_operation operation)
{
    last_control = operation;
    return 0;
}
static int ecg_record_store_sample(const struct max30001_ecg_sample *sample)
{
    assert(sample->time_valid);
    stores++;
    if (stop_on_store == stores) ecg_record_stop_requested = 1;
    if (store_error) return store_error;
    if (!ecg_record_filling_block) {
        block.sample_count = 0;
        ecg_record_filling_block = &block;
    }
    if (++block.sample_count == 1358U) ecg_record_filling_block = NULL;
    return 0;
}

/* PRODUCTION_FUNCTIONS */

static void reset(void)
{
    ecg_record_filling_block = NULL;
    ecg_record_file_open = true;
    ecg_record_stop_requested = ecg_record_writer_error = 0;
    ecg_record_chunk_block_count = ecg_record_chunk_index = 0;
    ecg_next_rtc_tick = ecg_next_sample_index = ecg_record_dropped_samples = 0;
    stores = store_error = stop_on_store = writes = syncs = activations = 0;
    write_result = sync_error = activate_error = 0;
    submit_result = 1;
    ecg_record_session_full_block_count = 8;
    block.state = 1;
    block.sample_count = 0;
    last_control = ECG_RECORD_CONTROL_NONE;
}

int main(void)
{
    struct max30001_ecg_sample samples[6] = {
        {true}, {true}, {true}, {true}, {true}, {true}
    };

    reset();
    assert(ecg_record_take_filling_block() == 0);
    assert(ecg_record_filling_block == &block);

    reset();
    ecg_record_stop_requested = 1;
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    assert(ecg_record_process_samples(samples, 6) == 0);
    assert(stores == 0); /* Empty boundary never starts the next block/chunk. */
    assert(ecg_record_finish_file(true) == 0);
    assert(last_control == ECG_RECORD_CONTROL_CLOSE);

    reset();
    ecg_record_stop_requested = 1;
    block.sample_count = 1356;
    ecg_record_filling_block = &block;
    assert(ecg_record_process_samples(samples, 6) == 0);
    assert(stores == 2 && ecg_record_filling_block == NULL);

    reset();
    block.sample_count = 1357;
    ecg_record_filling_block = &block;
    stop_on_store = 1; /* Request arriving during the boundary sample. */
    assert(ecg_record_process_samples(samples, 6) == 0);
    assert(stores == 1 && ecg_record_filling_block == NULL);

    reset();
    samples[1].time_valid = false;
    assert(ecg_record_process_samples(samples, 6) == 0 && stores == 5);
    store_error = -EIO;
    assert(ecg_record_process_samples(samples, 6) == -EIO && stores == 6);

    reset();
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    block.sync_after_write = true;
    ecg_record_block_work_handler(&block.work);
    assert(activations == 1 && writes == 1 && syncs == 1);
    assert(ecg_record_chunk_index == 1 && ecg_record_chunk_block_count == 1);
    assert(block.state == ECG_RECORD_BLOCK_FREE);

    reset();
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    activate_error = -ENOSPC;
    ecg_record_block_work_handler(&block.work);
    assert(activations == 1 && writes == 0);
    assert(ecg_record_writer_error == -ENOSPC);

    reset();
    submit_result = 2; /* Successful requeue while the previous handler returns. */
    assert(ecg_record_queue_finalized_block(&block) == 0);
    assert(block.state == ECG_RECORD_BLOCK_QUEUED && block.sync_after_write);
    assert(ecg_record_chunk_block_count == 0); /* Only the worker commits blocks. */
    reset();
    submit_result = -EBUSY;
    assert(ecg_record_queue_finalized_block(&block) == -EBUSY);
    assert(block.state == ECG_RECORD_BLOCK_FREE && ecg_record_chunk_block_count == 0);

    reset();
    samples[1].time_valid = true;
    store_error = -ENOMEM;
    assert(ecg_record_process_samples(samples, 6) == 0);
    assert(ecg_record_dropped_samples == 6);
    assert(ecg_next_rtc_tick == 6 && ecg_next_sample_index == 6);

    reset();
    block.sample_count = 1357;
    ecg_record_filling_block = &block;
    assert(ecg_record_finish_file(false) == 0);
    assert(block.state == ECG_RECORD_BLOCK_FREE && ecg_record_filling_block == NULL);
    assert(stores == 0 && last_control == ECG_RECORD_CONTROL_ABORT);
    puts("recorder host boundary/rotation checks passed");
    return 0;
}
