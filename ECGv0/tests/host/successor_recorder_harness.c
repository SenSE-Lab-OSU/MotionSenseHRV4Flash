#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>

#define ACCEL_RECORD_FORMAT_BLOCK_BYTES 4096U
#define ACCEL_RECORD_FORMAT_FILE_BYTES 65536U
#define MSENSE_ECG_FILE_HEADER_BYTES 4096U
#define MSENSE_ECG_FILE_BYTES 65536U
#define ARG_UNUSED(value) (void)(value)
#define LOG_INF(...) do { } while (0)

typedef int atomic_t;
typedef int k_spinlock_key_t;
struct fs_file_t { size_t position; };
struct k_work { int unused; };
struct k_sem { int unused; };
struct k_spinlock { int unused; };

enum accel_record_control_operation {
    ACCEL_RECORD_CONTROL_NONE,
    ACCEL_RECORD_CONTROL_OPEN,
    ACCEL_RECORD_CONTROL_ROTATE,
    ACCEL_RECORD_CONTROL_CLOSE,
    ACCEL_RECORD_CONTROL_ABORT,
};
enum ecg_record_control_operation {
    ECG_RECORD_CONTROL_NONE,
    ECG_RECORD_CONTROL_OPEN,
    ECG_RECORD_CONTROL_CLOSE,
    ECG_RECORD_CONTROL_ABORT,
};

static struct fs_file_t accel_record_file, ecg_record_file;
static struct k_work accel_record_prepare_work, ecg_record_prepare_work;
static struct k_sem accel_record_control_done, ecg_record_control_done;
static struct k_spinlock accel_record_state_lock;
static int my_work_q;
static atomic_t accel_record_failed, accel_record_rotating;
static bool accel_record_file_open, accel_record_next_prepared;
static bool accel_record_chunk_full;
static uint32_t accel_record_chunk_full_block_count;
static uint32_t accel_record_chunk_data_bytes, accel_record_chunk_index;
static uint64_t accel_record_session_id;
static char accel_record_path[96], accel_record_next_path[96];
static enum accel_record_control_operation accel_record_control_operation;
static int accel_record_control_result;

static bool ecg_record_file_open, ecg_record_next_prepared;
static atomic_t ecg_record_writer_error;
static uint32_t ecg_record_chunk_index, ecg_record_chunk_block_count;
static uint64_t ecg_record_session_id;
static char ecg_record_path[96], ecg_record_next_path[96];
static enum ecg_record_control_operation ecg_record_control_operation;
static int ecg_record_control_result;

static uint8_t scratch[4096];
static int preallocate_result, open_result, write_result, sync_result;
static int open_current_result, submit_result;
static int preallocations, opens, writes, syncs, closes, unlinks;
static int accel_header_builds, ecg_header_builds, fault_reports;
static uint32_t open_offset;
static size_t write_offsets[4];
static size_t write_sizes[4];
static char unlinked_path[96];
static uint64_t built_ecg_session;
static uint32_t built_ecg_chunk;

static int atomic_get(const atomic_t *value) { return *value; }
static void atomic_clear(atomic_t *value) { *value = 0; }
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    (void)lock; return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    (void)lock; (void)key;
}
static void k_sem_give(struct k_sem *sem) { (void)sem; }
static int k_work_submit_to_queue(int *queue, struct k_work *work)
{
    (void)queue; (void)work; return submit_result;
}
static uint8_t *filesystem_scratch_buffer(void) { return scratch; }
static int filesystem_make_recording_chunk_path(
    char *path, size_t size, const char *prefix, uint64_t session,
    uint32_t chunk)
{
    int count = snprintf(path, size, "%s_%llu_%u", prefix,
                         (unsigned long long)session, chunk);
    return count < 0 || (size_t)count >= size ? -ENAMETOOLONG : 0;
}
static int filesystem_preallocate_file(
    struct fs_file_t *file, const char *path, uint32_t bytes,
    const void *header, size_t header_bytes, bool leave_open)
{
    (void)file; (void)path; (void)bytes;
    preallocations++;
    assert(header == NULL && header_bytes == 0U && !leave_open);
    return preallocate_result;
}
static int filesystem_open_preallocated_file(
    struct fs_file_t *file, const char *path, uint32_t offset)
{
    (void)path; opens++; open_offset = offset;
    if (open_result == 0) file->position = offset;
    return open_result;
}
static ssize_t fs_write(struct fs_file_t *file, const void *data, size_t size)
{
    ssize_t result = write_result == -9999 ? (ssize_t)size : write_result;
    assert(writes < 4);
    write_offsets[writes] = file->position;
    write_sizes[writes] = size;
    if (writes == 0) assert(((const uint8_t *)data)[0] == 'A' ||
                            ((const uint8_t *)data)[0] == 'E');
    writes++;
    if (result > 0) file->position += (size_t)result;
    return result;
}
static int fs_sync(struct fs_file_t *file)
{
    (void)file; syncs++; return sync_result;
}
static int fs_close(struct fs_file_t *file)
{
    (void)file; closes++; return 0;
}
static int fs_unlink(const char *path)
{
    unlinks++; strcpy(unlinked_path, path); return 0;
}
static void accel_record_format_build_header(uint8_t *header)
{
    memset(header, 'A', ACCEL_RECORD_FORMAT_BLOCK_BYTES);
    accel_header_builds++;
}
static void msense_ecg_file_header_build(uint8_t *header, uint64_t session,
                                         uint32_t chunk)
{
    memset(header, 'E', MSENSE_ECG_FILE_HEADER_BYTES);
    ecg_header_builds++;
    built_ecg_session = session;
    built_ecg_chunk = chunk;
}
static int accel_record_close_current_chunk(void)
{
    closes++; accel_record_file_open = false; return 0;
}
static int ecg_record_close_current_chunk(bool sync_before_close)
{
    (void)sync_before_close; closes++; ecg_record_file_open = false; return 0;
}
static int accel_record_open_current_chunk(void)
{
    if (open_current_result == 0) {
        accel_record_file_open = true;
        strcpy(accel_record_path, "active_accel");
    }
    return open_current_result;
}
static int ecg_record_open_current_chunk(void)
{
    if (open_current_result == 0) {
        ecg_record_file_open = true;
        strcpy(ecg_record_path, "active_ecg");
    }
    return open_current_result;
}
static int accel_record_dispatch_pending_blocks(void) { return 0; }
static void accel_record_report_fault(int error)
{
    (void)error; fault_reports++;
}
/* ACCEL_SUCCESSOR_FUNCTIONS */

/* ECG_SUCCESSOR_FUNCTIONS */

static void reset(void)
{
    memset(scratch, 0, sizeof(scratch));
    memset(write_offsets, 0, sizeof(write_offsets));
    memset(write_sizes, 0, sizeof(write_sizes));
    unlinked_path[0] = '\0';
    preallocate_result = open_result = sync_result = open_current_result = 0;
    write_result = -9999;
    submit_result = 0;
    preallocations = opens = writes = syncs = closes = unlinks = 0;
    accel_header_builds = ecg_header_builds = fault_reports = 0;
    open_offset = 99U;
    built_ecg_session = 0U;
    built_ecg_chunk = 0U;
    accel_record_failed = accel_record_rotating = 0;
    accel_record_file_open = ecg_record_file_open = true;
    accel_record_next_prepared = ecg_record_next_prepared = false;
    accel_record_chunk_full = true;
    accel_record_chunk_full_block_count = 7U;
    accel_record_chunk_data_bytes = 123U;
    accel_record_chunk_index = ecg_record_chunk_index = 4U;
    ecg_record_chunk_block_count = 7U;
    accel_record_session_id = 11U;
    ecg_record_session_id = 22U;
    strcpy(accel_record_path, "active_accel");
    strcpy(ecg_record_path, "active_ecg");
    strcpy(accel_record_next_path, "next_accel");
    strcpy(ecg_record_next_path, "next_ecg");
    accel_record_control_operation = ACCEL_RECORD_CONTROL_NONE;
    ecg_record_control_operation = ECG_RECORD_CONTROL_NONE;
}

static void test_accel_successor(void)
{
    uint8_t payload = 1U;

    reset();
    assert(accel_record_prepare_next_chunk() == 0);
    assert(preallocations == 1 && writes == 0 && accel_record_next_prepared);
    assert(accel_record_activate_next_chunk() == 0);
    assert(opens == 1 && open_offset == 0U);
    assert(accel_header_builds == 1 && writes == 1 && syncs == 1);
    assert(write_offsets[0] == 0U && write_sizes[0] == 4096U);
    assert(!accel_record_next_prepared && accel_record_file_open);
    assert(accel_record_chunk_index == 5U);
    assert(fs_write(&accel_record_file, &payload, sizeof(payload)) == 1);
    assert(write_offsets[1] == 4096U);

    reset();
    accel_record_next_prepared = true;
    write_result = 100;
    assert(accel_record_activate_next_chunk() == -EIO);
    assert(!accel_record_next_prepared && accel_record_file_open);
    assert(writes == 1 && syncs == 0 && unlinks == 0);

    reset();
    accel_record_next_prepared = true;
    sync_result = -EIO;
    assert(accel_record_activate_next_chunk() == -EIO);
    assert(!accel_record_next_prepared && accel_record_file_open);
    assert(writes == 1 && syncs == 1 && unlinks == 0);
}

static void test_ecg_successor(void)
{
    uint8_t payload = 1U;

    reset();
    assert(ecg_record_prepare_next_chunk() == 0);
    assert(preallocations == 1 && writes == 0 && ecg_record_next_prepared);
    assert(ecg_record_activate_next_chunk() == 0);
    assert(opens == 1 && open_offset == 0U);
    assert(ecg_header_builds == 1 && writes == 1 && syncs == 1);
    assert(built_ecg_session == 22U && built_ecg_chunk == 5U);
    assert(write_offsets[0] == 0U && write_sizes[0] == 4096U);
    assert(!ecg_record_next_prepared && ecg_record_file_open);
    assert(fs_write(&ecg_record_file, &payload, sizeof(payload)) == 1);
    assert(write_offsets[1] == 4096U);

    reset();
    ecg_record_next_prepared = true;
    write_result = -ENOSPC;
    assert(ecg_record_activate_next_chunk() == -ENOSPC);
    assert(!ecg_record_next_prepared && ecg_record_file_open);
    assert(writes == 1 && syncs == 0 && unlinks == 0);

    reset();
    ecg_record_next_prepared = true;
    sync_result = -EIO;
    assert(ecg_record_activate_next_chunk() == -EIO);
    assert(!ecg_record_next_prepared && ecg_record_file_open);
    assert(writes == 1 && syncs == 1 && unlinks == 0);
}

static void test_cleanup_ownership(void)
{
    reset();
    accel_record_next_prepared = true;
    accel_record_control_operation = ACCEL_RECORD_CONTROL_CLOSE;
    accel_record_control_work_handler(NULL);
    assert(unlinks == 1 && strcmp(unlinked_path, "next_accel") == 0);

    reset();
    ecg_record_next_prepared = true;
    ecg_record_control_operation = ECG_RECORD_CONTROL_CLOSE;
    ecg_record_control_work_handler(NULL);
    assert(unlinks == 1 && strcmp(unlinked_path, "next_ecg") == 0);

    reset();
    accel_record_control_operation = ACCEL_RECORD_CONTROL_ABORT;
    accel_record_control_work_handler(NULL);
    assert(closes == 1 && unlinks == 0);

    reset();
    ecg_record_control_operation = ECG_RECORD_CONTROL_ABORT;
    ecg_record_control_work_handler(NULL);
    assert(closes == 1 && unlinks == 0);

    reset();
    preallocate_result = -ENOSPC;
    accel_record_control_operation = ACCEL_RECORD_CONTROL_OPEN;
    accel_record_control_work_handler(NULL);
    assert(closes == 1 && unlinks == 0 && !accel_record_file_open);

    reset();
    preallocate_result = -ENOSPC;
    ecg_record_control_operation = ECG_RECORD_CONTROL_OPEN;
    ecg_record_control_work_handler(NULL);
    assert(closes == 1 && unlinks == 0 && !ecg_record_file_open);
}

int main(void)
{
    test_accel_successor();
    test_ecg_successor();
    test_cleanup_ownership();
    puts("successor recorder lifecycle checks passed");
    return 0;
}
