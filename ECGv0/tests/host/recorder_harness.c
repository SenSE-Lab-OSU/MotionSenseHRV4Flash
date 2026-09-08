/* I/O stubs around production recorder boundary/rotation functions. */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>

#define MSENSE_ECG_FILE_DATA_BLOCKS 1023U
#define ECG_RECORD_BLOCK_FREE 0
#define ECG_RECORD_BLOCK_QUEUED 2
#define ECG_RECORD_SYNC_INTERVAL_BLOCKS 8U
struct ecg_record_block {
    int state, work;
    unsigned sample_count;
    bool sync_after_write;
} block;
struct max30001_ecg_sample { bool time_valid; };
static struct ecg_record_block *ecg_record_filling_block;
static int ecg_record_stop_requested, ecg_record_writer_error, my_work_q;
static unsigned ecg_record_chunk_block_count, ecg_record_chunk_index;
static unsigned ecg_record_session_full_block_count;
static int stores, store_error, stop_on_store, drains, closes, opens;
static int drain_error, close_error, open_error, submit_result;
static bool last_sync;

static int atomic_get(const int *value) { return *value; }
static void atomic_set(int *value, int next) { *value = next; }
static int k_work_queue_drain(int *queue, bool plug)
{
    (void)queue; assert(!plug); drains++; return drain_error;
}
static int k_work_submit_to_queue(int *queue, int *work)
{
    (void)queue; (void)work; return submit_result;
}
static int ecg_record_close_current_chunk(bool sync)
{
    closes++; last_sync = sync; return close_error;
}
static int ecg_record_open_current_chunk(void) { opens++; return open_error; }
static struct ecg_record_block *ecg_record_take_free_block(void) { return &block; }
void ecg_record_report_writer_fault(int error)
{
    if (!ecg_record_writer_error) ecg_record_writer_error = error;
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
    ecg_record_stop_requested = ecg_record_writer_error = 0;
    ecg_record_chunk_block_count = ecg_record_chunk_index = 0;
    stores = store_error = stop_on_store = drains = closes = opens = 0;
    drain_error = close_error = open_error = 0;
    submit_result = 1;
    ecg_record_session_full_block_count = 8;
    block.state = 1;
    block.sample_count = 0;
    last_sync = false;
}

int main(void)
{
    struct max30001_ecg_sample samples[6] = {
        {true}, {true}, {true}, {true}, {true}, {true}
    };

    reset();
    ecg_record_stop_requested = 1;
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    assert(ecg_record_process_samples(samples, 6) == 0);
    assert(stores == 0); /* Empty boundary never starts the next block/chunk. */
    assert(ecg_record_finish_file(true) == 0);
    assert(opens == 0 && closes == 1 && last_sync);

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
    assert(ecg_record_rotate_chunk() == 0 && opens == 0 && drains == 0);
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    assert(ecg_record_rotate_chunk() == 0);
    assert(opens == 1 && closes == 1 && last_sync);
    assert(ecg_record_chunk_index == 1 && ecg_record_chunk_block_count == 0);

    reset();
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    ecg_record_writer_error = -EIO;
    assert(ecg_record_rotate_chunk() == -EIO && opens == 0 && closes == 0);

    reset();
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    close_error = -EIO;
    assert(ecg_record_take_filling_block() == -EIO && opens == 0);
    assert(ecg_record_writer_error == -EIO);

    reset();
    ecg_record_chunk_block_count = MSENSE_ECG_FILE_DATA_BLOCKS;
    open_error = -ENOSPC;
    assert(ecg_record_take_filling_block() == -ENOSPC);
    assert(ecg_record_writer_error == -ENOSPC);

    reset();
    submit_result = 2; /* Successful requeue while the previous handler returns. */
    assert(ecg_record_queue_finalized_block(&block) == 0);
    assert(block.state == ECG_RECORD_BLOCK_QUEUED && block.sync_after_write);
    assert(ecg_record_chunk_block_count == 1);
    reset();
    submit_result = -EBUSY;
    assert(ecg_record_queue_finalized_block(&block) == -EBUSY);
    assert(block.state == ECG_RECORD_BLOCK_FREE && ecg_record_chunk_block_count == 0);

    reset();
    block.sample_count = 1357;
    ecg_record_filling_block = &block;
    assert(ecg_record_finish_file(false) == 0);
    assert(block.state == ECG_RECORD_BLOCK_FREE && ecg_record_filling_block == NULL);
    assert(stores == 0 && !last_sync && closes == 1);
    puts("recorder host boundary/rotation checks passed");
    return 0;
}
