#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#define ACCEL_RECORD_FORMAT_BLOCK_BYTES 4096U
#define ACCEL_RECORD_FORMAT_TRAILER_BYTES 4096U
#define ACCEL_RECORD_FORMAT_TRAILER_OFFSET 61440U
#define FS_SEEK_SET 0
#define ACCEL_RECORD_BLOCK_FREE 0
#define ACCEL_RECORD_BLOCK_WRITING 4
#define CONTAINER_OF(pointer, type, member) \
    ((type *)((char *)(pointer) - offsetof(type, member)))

typedef int atomic_t;
struct k_work { int unused; };
struct fs_file_t { void *filep; };
struct accel_record_block {
    struct k_work work;
    uint8_t data[ACCEL_RECORD_FORMAT_BLOCK_BYTES];
    atomic_t state;
    size_t write_length;
    bool sync_after_write;
};

enum event_type { EVENT_WRITE, EVENT_SYNC, EVENT_SEEK, EVENT_CLOSE };
static struct fs_file_t accel_record_file;
static bool accel_record_file_open;
static bool accel_record_file_retired;
static bool accel_record_retired_handle_live;
static bool accel_record_next_prepared;
static uint32_t accel_record_chunk_data_bytes;
static int events[8], event_count, writes, syncs, seeks, closes, releases, faults;
static int file_inits;
static size_t write_sizes[4];
static ssize_t write_results[4];
static int seek_result, sync_result, close_result;
static uint8_t scratch[ACCEL_RECORD_FORMAT_TRAILER_BYTES];

static void atomic_set(atomic_t *value, int next) { *value = next; }
static void fs_file_t_init(struct fs_file_t *file) { file->filep = NULL; file_inits++; }
static uint8_t *filesystem_scratch_buffer(void) { return scratch; }
static void accel_record_format_build_trailer(uint8_t *metadata, uint32_t valid_bytes)
{
    memset(metadata, (int)(valid_bytes & 0xffU), ACCEL_RECORD_FORMAT_TRAILER_BYTES);
}
static int fs_seek(struct fs_file_t *file, off_t offset, int whence)
{
    (void)file; (void)offset; (void)whence;
    events[event_count++] = EVENT_SEEK;
    seeks++;
    return seek_result;
}
static ssize_t fs_write(struct fs_file_t *file, const void *data, size_t size)
{
    ssize_t result;

    (void)file; (void)data;
    events[event_count++] = EVENT_WRITE;
    write_sizes[writes++] = size;
    result = write_results[writes - 1];
    return result == -9999 ? (ssize_t)size : result;
}
static int fs_sync(struct fs_file_t *file)
{
    (void)file;
    events[event_count++] = EVENT_SYNC;
    syncs++;
    return sync_result;
}
static int fs_close(struct fs_file_t *file)
{
	assert(file->filep != NULL);
	file->filep = NULL;
	events[event_count++] = EVENT_CLOSE;
    closes++;
    return close_result;
}
static void accel_record_release_block(struct accel_record_block *block)
{
    releases++;
    atomic_set(&block->state, ACCEL_RECORD_BLOCK_FREE);
}
static void accel_record_report_fault(int error)
{
    (void)error;
    faults++;
}

/* PRODUCTION_FUNCTIONS */

static void reset(void)
{
    int i;

	accel_record_file_open = true;
	accel_record_file_retired = false;
	accel_record_retired_handle_live = false;
	accel_record_next_prepared = false;
    accel_record_chunk_data_bytes = 19U;
    event_count = writes = syncs = seeks = closes = releases = faults = file_inits = 0;
    seek_result = sync_result = close_result = 0;
    memset(events, 0, sizeof(events));
    memset(write_sizes, 0, sizeof(write_sizes));
	for (i = 0; i < 4; i++) {
		write_results[i] = -9999;
	}
	accel_record_file.filep = &accel_record_file;
}

int main(void)
{
    struct accel_record_block block = {0};

    reset();
    block.write_length = ACCEL_RECORD_FORMAT_BLOCK_BYTES;
    block.sync_after_write = true;
    accel_record_block_work_handler(&block.work);
    assert(writes == 1 && write_sizes[0] == ACCEL_RECORD_FORMAT_BLOCK_BYTES);
    assert(syncs == 1 && closes == 0 && !accel_record_file_retired);

    reset();
    write_results[0] = -EIO;
    block.write_length = ACCEL_RECORD_FORMAT_BLOCK_BYTES;
    block.sync_after_write = true;
    accel_record_block_work_handler(&block.work);
	assert(accel_record_file_retired && accel_record_retired_handle_live &&
	       !accel_record_file_open && writes == 1);
	accel_record_block_work_handler(&block.work);
	assert(writes == 1 && syncs == 0 && closes == 0);
	accel_recorder_filesystem_unmounted();
	assert(!accel_record_file_retired && !accel_record_retired_handle_live &&
	       closes == 1 && file_inits == 1);
	accel_recorder_filesystem_unmounted();
	assert(closes == 1 && file_inits == 1);

    reset();
    block.write_length = 23U;
    block.sync_after_write = false;
    accel_record_block_work_handler(&block.work);
    assert(writes == 1 && syncs == 0 && closes == 0);
    assert(accel_record_close_current_chunk() == 0);
    assert(event_count == 4 && events[0] == EVENT_WRITE && events[1] == EVENT_SEEK &&
           events[2] == EVENT_WRITE && events[3] == EVENT_CLOSE);
    assert(write_sizes[0] == 23U && write_sizes[1] == ACCEL_RECORD_FORMAT_TRAILER_BYTES);
    assert(syncs == 0 && closes == 1 && !accel_record_file_retired);

    reset();
    write_results[0] = -EIO;
    block.write_length = 17U;
    block.sync_after_write = false;
    accel_record_block_work_handler(&block.work);
    assert(accel_record_file_retired && !accel_record_file_open && writes == 1);
    assert(accel_record_close_current_chunk() == -EIO);
    assert(writes == 1 && syncs == 0 && closes == 0);

    reset();
    seek_result = -EIO;
    assert(accel_record_close_current_chunk() == -EIO);
	assert(accel_record_file_retired && accel_record_retired_handle_live &&
	       seeks == 1 && writes == 0 && closes == 0);
	assert(accel_record_close_current_chunk() == -EIO && seeks == 1);

    reset();
    write_results[0] = -EIO;
    assert(accel_record_close_current_chunk() == -EIO);
	assert(accel_record_file_retired && accel_record_retired_handle_live &&
	       seeks == 1 && writes == 1 && closes == 0);
	assert(accel_record_close_current_chunk() == -EIO && writes == 1);

    reset();
	close_result = -EIO;
	assert(accel_record_close_current_chunk() == -EIO);
	assert(accel_record_file_retired && !accel_record_retired_handle_live &&
	       accel_record_file.filep == NULL && writes == 1 && closes == 1 &&
	       syncs == 0 && file_inits == 1);
	assert(accel_record_close_current_chunk() == -EIO && closes == 1);
	accel_record_next_prepared = true;
	accel_recorder_filesystem_unmounted();
	assert(!accel_record_file_retired && !accel_record_file_open &&
	       !accel_record_retired_handle_live && !accel_record_next_prepared &&
	       writes == 1 && closes == 1 && file_inits == 2);
	accel_recorder_filesystem_unmounted();
	assert(closes == 1 && file_inits == 2);

    puts("accelerometer terminal-write checks passed");
    return 0;
}
