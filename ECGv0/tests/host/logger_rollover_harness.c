#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#define RECORDING_FILE_BYTES (4U * 1024U * 1024U)
#define FS_O_WRITE 2
#define MIN(a, b) ((a) < (b) ? (a) : (b))

struct fs_file_t { void *filep; };
typedef struct LoggerFile {
    int current_writes;
    char file_name[96];
    struct fs_file_t self_file;
} LoggerFile;

static LoggerFile log_file;
static bool logger_started, logger_next_prepared, logger_prepare_requested;
static bool logger_file_open, logger_file_retired, logger_retired_handle_live;
static size_t logger_logical_bytes;
static uint8_t storage_percent_full;
static char logger_next_path[96];
static int open_result, close_result, write_result, capacity_result, truncate_result;
static int opens, closes, writes, capacity_queries, truncates;
static size_t write_sizes[4];
static off_t truncate_lengths[4];
static int events[8], event_count;

enum {
    EVENT_TRUNCATE,
    EVENT_CLOSE,
};

static void fs_file_t_init(struct fs_file_t *file) { file->filep = NULL; }
static int fs_open(struct fs_file_t *file, const char *path, int flags)
{
    (void)path; (void)flags;
    opens++;
    if (open_result == 0) {
        file->filep = file;
    }
    return open_result;
}
static int fs_close(struct fs_file_t *file)
{
    assert(file->filep != NULL);
    file->filep = NULL;
    events[event_count++] = EVENT_CLOSE;
    closes++;
    return close_result;
}
static int fs_truncate(struct fs_file_t *file, off_t length)
{
    (void)file;
    events[event_count++] = EVENT_TRUNCATE;
    truncate_lengths[truncates++] = length;
    return truncate_result;
}
static ssize_t fs_write(struct fs_file_t *file, const void *data, size_t size)
{
    (void)file; (void)data;
    write_sizes[writes++] = size;
    return write_result == -9999 ? (ssize_t)size : write_result;
}
static int get_storage_percent_full(void)
{
    capacity_queries++;
    storage_percent_full = (uint8_t)capacity_result;
    return capacity_result;
}

/* PRODUCTION_FUNCTIONS */

static void reset(void)
{
    memset(&log_file, 0, sizeof(log_file));
    logger_started = logger_next_prepared = logger_file_open = true;
    logger_prepare_requested = logger_file_retired = logger_retired_handle_live = false;
    logger_logical_bytes = 0U;
    strcpy(logger_next_path, "/SD:/log2.txt");
    log_file.self_file.filep = &log_file;
    open_result = close_result = capacity_result = truncate_result = 0;
    write_result = -9999;
    opens = closes = writes = capacity_queries = truncates = 0;
    memset(write_sizes, 0, sizeof(write_sizes));
    memset(truncate_lengths, 0, sizeof(truncate_lengths));
    memset(events, 0, sizeof(events));
    event_count = 0;
}

static void assert_close_length(size_t length)
{
    reset();
    logger_logical_bytes = length;
    assert(logger_close_current_file() == 0);
    assert(truncates == 1 && truncate_lengths[0] == (off_t)length);
    assert(closes == 1 && event_count == 2 &&
           events[0] == EVENT_TRUNCATE && events[1] == EVENT_CLOSE);
}

int main(void)
{
    uint8_t *data = malloc(RECORDING_FILE_BYTES + 7U);

    assert(data != NULL);
    reset();
    assert(logger_write_bytes(data, RECORDING_FILE_BYTES + 7U) == 0);
    assert(writes == 2 && write_sizes[0] == RECORDING_FILE_BYTES &&
           write_sizes[1] == 7U);
    assert(truncates == 1 && truncate_lengths[0] == RECORDING_FILE_BYTES);
    assert(closes == 1 && opens == 1 && capacity_queries == 1);
    assert(logger_logical_bytes == 7U && logger_file_open && !logger_file_retired);
    assert(logger_close_current_file() == 0);
    assert(truncates == 2 && truncate_lengths[1] == 7 && closes == 2);

    reset();
    capacity_result = 99;
    assert(logger_write_bytes(data, RECORDING_FILE_BYTES + 7U) == -ENOSPC);
    assert(writes == 1 && write_sizes[0] == RECORDING_FILE_BYTES);
    assert(truncates == 1 && truncate_lengths[0] == RECORDING_FILE_BYTES);
    assert(closes == 1 && opens == 0 && capacity_queries == 1 && !logger_file_open);

    reset();
    assert(logger_write_bytes(data, 13U) == 0);
    assert(logger_close_current_file() == 0);
    assert(writes == 1 && truncates == 1 && truncate_lengths[0] == 13 &&
           closes == 1);

    assert_close_length(0U);
    assert_close_length(4096U);
    assert_close_length(RECORDING_FILE_BYTES);

    reset();
    logger_logical_bytes = 13U;
    truncate_result = -EIO;
    assert(logger_close_current_file() == -EIO);
    assert(logger_file_retired && logger_retired_handle_live && !logger_file_open && truncates == 1 &&
           truncate_lengths[0] == 13 && closes == 0);
    assert(logger_close_current_file() == -EIO);
    assert(truncates == 1 && closes == 0);
    /* Post-unmount release frees the retired FIL without retrying truncate. */
    logger_release_retired_file();
    assert(truncates == 1 && closes == 1 && events[1] == EVENT_CLOSE &&
           !logger_file_retired && !logger_retired_handle_live && !logger_file_open);

    reset();
    logger_logical_bytes = 13U;
    close_result = -EIO;
    assert(logger_close_current_file() == -EIO);
    assert(logger_file_retired && !logger_retired_handle_live && !logger_file_open &&
           log_file.self_file.filep == NULL && truncates == 1 &&
           truncate_lengths[0] == 13 && closes == 1 &&
           events[0] == EVENT_TRUNCATE && events[1] == EVENT_CLOSE);
    assert(logger_close_current_file() == -EIO);
    assert(truncates == 1 && closes == 1);
    logger_release_retired_file();
    assert(truncates == 1 && closes == 1 && event_count == 2 &&
           !logger_file_retired && !logger_retired_handle_live && !logger_file_open);

    reset();
    write_result = -EIO;
    assert(logger_write_bytes(data, 13U) == -EIO);
    assert(logger_file_retired && logger_retired_handle_live && !logger_file_open && writes == 1);
    assert(logger_write_bytes(data, 13U) == -EIO);
    assert(logger_close_current_file() == -EIO);
    assert(writes == 1 && closes == 0);
    logger_release_retired_file();
    assert(writes == 1 && closes == 1 && !logger_file_retired &&
           !logger_retired_handle_live && !logger_file_open);

    free(data);
    puts("logger fixed-capacity rollover checks passed");
    return 0;
}
