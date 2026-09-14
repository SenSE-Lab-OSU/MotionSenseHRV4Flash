#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#define RECORDING_FILE_BYTES (4U * 1024U * 1024U)
#define PPG_RECORD_BYTES 16U
#define ACCEL_RECORD_BYTES 26U
#define FR_OK 0
#define FS_O_CREATE 1
#define FS_O_WRITE 2
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define IS_ENABLED(value) 0
#define CONFIG_DISK_DRIVER_RAW_NAND 0
static void mock_log(const char *format, ...) { (void)format; }
#define LOG_WRN(...) mock_log(__VA_ARGS__)

typedef int FRESULT;
typedef struct { int unused; } FIL;
struct fs_file_t { void *filep; };
struct fs_dirent { int unused; };
struct fs_mount_t { const char *mnt_point; };
typedef struct data_upload_buffer { char ignored[1]; size_t current_size; } data_upload_buffer;
typedef struct MotionSenseFile {
    int write_size;
    size_t record_bytes;
    size_t logical_bytes;
    uint32_t sequence;
    uint64_t start_time;
    uint64_t file_id;
    bool first_sample_init;
    bool file_id_valid;
    bool file_open;
    bool retired;
    bool retired_handle_live;
    char sensor_string[5];
    char file_name[96];
    char sensor_format[90];
    struct fs_file_t self_file;
    bool switch_buffer;
    data_upload_buffer buffer1;
    data_upload_buffer buffer2;
} MotionSenseFile;
enum sensor_type { ppg, accelorometer, passthrough, customlog };

static struct fs_mount_t fs_mnt = { "/SD:" };
static MotionSenseFile ppg_file = { .record_bytes = PPG_RECORD_BYTES, .sensor_string = "ppg" };
static MotionSenseFile accel_file = { .record_bytes = ACCEL_RECORD_BYTES, .sensor_string = "ac" };
static MotionSenseFile log_file = { .record_bytes = 1U, .sensor_string = "log" };
static bool file_system_ready = true, filesystem_mounted = true;
static uint8_t storage_percent_full;
static bool use_random_files;
static int patient_num, total_log_files;
static bool file_system_malfunction;
static int stat_result, open_result, expand_result, write_result, close_result, capacity_result, truncate_result;
static int opens, expands, writes, closes, capacity_queries, truncates;
static size_t write_sizes[4];
static off_t truncate_lengths[4];
static int write_generations[4], generation;
static char opened_paths[4][96];
static int random_calls;
static int events[8], event_count;

enum {
    EVENT_TRUNCATE,
    EVENT_CLOSE,
};

static void status_reg_ble_notification(void) { }
static bool get_read_only(void) { return false; }
static uint32_t sys_rand32_get(void) { return 1600U + (uint32_t)random_calls++; }
static int get_storage_percent_full(void)
{
    capacity_queries++;
    storage_percent_full = (uint8_t)capacity_result;
    return capacity_result;
}
static void fs_file_t_init(struct fs_file_t *file) { file->filep = NULL; }
static int fs_stat(const char *path, struct fs_dirent *entry)
{ (void)path; (void)entry; return stat_result; }
static int fs_open(struct fs_file_t *file, const char *path, int flags)
{
    (void)flags;
    assert(opens < 4);
    strcpy(opened_paths[opens], path);
    opens++;
    generation = opens - 1;
    if (open_result == 0) file->filep = file;
    return open_result;
}
static FRESULT f_expand(FIL *file, uint32_t bytes, int contiguous)
{ (void)file; assert(bytes == RECORDING_FILE_BYTES && contiguous == 1); expands++; return expand_result; }
static ssize_t fs_write(struct fs_file_t *file, const void *data, size_t size)
{
    (void)file; (void)data;
    assert(writes < 4);
    write_sizes[writes] = size;
    write_generations[writes] = generation;
    writes++;
    return write_result == -9999 ? (ssize_t)size : write_result;
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

/* PRODUCTION_FUNCTIONS */

static void reset(void)
{
    memset(&ppg_file, 0, sizeof(ppg_file));
    memset(&accel_file, 0, sizeof(accel_file));
    memset(&log_file, 0, sizeof(log_file));
    ppg_file.record_bytes = PPG_RECORD_BYTES; strcpy(ppg_file.sensor_string, "ppg");
    accel_file.record_bytes = ACCEL_RECORD_BYTES; strcpy(accel_file.sensor_string, "ac");
    log_file.record_bytes = 1U; strcpy(log_file.sensor_string, "log");
    file_system_ready = filesystem_mounted = true;
    storage_percent_full = 0U;
    stat_result = -ENOENT; open_result = expand_result = close_result = capacity_result = truncate_result = 0;
    write_result = -9999;
    opens = expands = writes = closes = capacity_queries = truncates = generation = random_calls = 0;
    memset(write_sizes, 0, sizeof(write_sizes));
    memset(truncate_lengths, 0, sizeof(truncate_lengths));
    memset(write_generations, 0, sizeof(write_generations));
    memset(opened_paths, 0, sizeof(opened_paths));
    memset(events, 0, sizeof(events));
    event_count = 0;
    patient_num = total_log_files = 0;
    use_random_files = false;
    file_system_malfunction = false;
    ppg_file.start_time = accel_file.start_time = log_file.start_time = 100U;
}

static void test_accel_record_carry(void)
{
    size_t first_file = (RECORDING_FILE_BYTES / ACCEL_RECORD_BYTES) * ACCEL_RECORD_BYTES;
    uint8_t *data = malloc(first_file + ACCEL_RECORD_BYTES);

    assert(data != NULL);
    reset();
    assert(sensor_write_to_file(data, first_file + ACCEL_RECORD_BYTES, accelorometer) == 0);
    assert(writes == 2 && write_sizes[0] == first_file && write_sizes[1] == ACCEL_RECORD_BYTES);
    assert(write_generations[0] == 0 && write_generations[1] == 1);
    assert(closes == 1 && capacity_queries == 1);
    free(data);
}

static void test_exact_ppg_capacity_and_names(void)
{
    uint8_t *data = malloc(RECORDING_FILE_BYTES);

    assert(data != NULL);
    reset();
    assert(sensor_write_to_file(data, RECORDING_FILE_BYTES, ppg) == 0);
    assert(writes == 1 && write_sizes[0] == RECORDING_FILE_BYTES);
    assert(opens == 1 && closes == 1 && truncates == 0 && ppg_file.sequence == 1U);
    assert(strcmp(opened_paths[0], "/SD:/ppg100.bin") == 0);
    free(data);

    reset();
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == 0);
    assert(rollover_sensor_file(&ppg_file) == 0);
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == 0);
    assert(strcmp(opened_paths[0], "/SD:/ppg100.bin") == 0);
    assert(strcmp(opened_paths[1], "/SD:/ppg101.bin") == 0);
}

static void test_random_base_is_stable(void)
{
    reset();
    use_random_files = true;
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == 0);
    assert(rollover_sensor_file(&ppg_file) == 0);
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == 0);
    assert(random_calls == 1);
    assert(strcmp(opened_paths[0], "/SD:/ppg700.bin") == 0);
    assert(strcmp(opened_paths[1], "/SD:/ppg701.bin") == 0);
}

static void test_rollover_stops_when_storage_fills(void)
{
    uint8_t *data = malloc(RECORDING_FILE_BYTES + PPG_RECORD_BYTES);

    assert(data != NULL);
    reset();
    capacity_result = 99;
    assert(sensor_write_to_file(data, RECORDING_FILE_BYTES + PPG_RECORD_BYTES, ppg) == -ENOSPC);
    assert(writes == 1 && write_sizes[0] == RECORDING_FILE_BYTES);
    assert(opens == 1 && closes == 1 && capacity_queries == 1);
    free(data);
}

static void test_numbered_log_names(void)
{
    reset();
    assert(sensor_write_to_file("log", 3U, customlog) == 0);
    assert(rollover_sensor_file(&log_file) == 0);
    assert(sensor_write_to_file("log", 3U, customlog) == 0);
    assert(total_log_files == 2);
    assert(strcmp(opened_paths[0], "/SD:/log1.txt") == 0);
    assert(strcmp(opened_paths[1], "/SD:/log2.txt") == 0);
}

static void test_log_terminal_truncation(void)
{
    uint8_t *data = malloc(RECORDING_FILE_BYTES);

    assert(data != NULL);

    reset();
    assert(open_sensor_file(&log_file, customlog) == 0);
    assert(close_sensor_file(&log_file) == 0);
    assert(truncates == 1 && truncate_lengths[0] == 0 && closes == 1 &&
           event_count == 2 && events[0] == EVENT_TRUNCATE &&
           events[1] == EVENT_CLOSE);

    reset();
    assert(sensor_write_to_file("log", 3U, customlog) == 0);
    assert(close_sensor_file(&log_file) == 0);
    assert(truncates == 1 && truncate_lengths[0] == 3 && closes == 1 &&
           events[0] == EVENT_TRUNCATE && events[1] == EVENT_CLOSE);

    reset();
    assert(sensor_write_to_file(data, RECORDING_FILE_BYTES, customlog) == 0);
    assert(truncates == 1 && truncate_lengths[0] == RECORDING_FILE_BYTES &&
           closes == 1 && events[0] == EVENT_TRUNCATE &&
           events[1] == EVENT_CLOSE);

    reset();
    assert(sensor_write_to_file("log", 3U, customlog) == 0);
    truncate_result = -EIO;
    assert(close_sensor_file(&log_file) == -EIO);
    assert(log_file.retired && log_file.retired_handle_live && !log_file.file_open &&
           truncates == 1 &&
           truncate_lengths[0] == 3 && closes == 0);
    assert(close_sensor_file(&log_file) == -EIO);
    assert(truncates == 1 && closes == 0);
    release_retired_sensor_file(&log_file);
    assert(truncates == 1 && closes == 1 && events[1] == EVENT_CLOSE &&
           !log_file.retired && !log_file.retired_handle_live && !log_file.file_open);

    free(data);
}

static void test_collision_terminal_close_and_no_retry(void)
{
    reset();
    stat_result = 0;
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == -EEXIST);
    assert(opens == 0 && writes == 0);

    reset();
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == 0);
    assert(close_sensor_file(&ppg_file) == 0);
    assert(closes == 1);

    reset();
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == 0);
    close_result = -EIO;
    assert(close_sensor_file(&ppg_file) == -EIO);
    assert(ppg_file.retired && !ppg_file.retired_handle_live && !ppg_file.file_open &&
           ppg_file.self_file.filep == NULL && closes == 1);
    assert(close_sensor_file(&ppg_file) == -EIO);
    assert(closes == 1);
    release_retired_sensor_file(&ppg_file);
    assert(closes == 1 && !ppg_file.retired && !ppg_file.retired_handle_live &&
           !ppg_file.file_open);

    reset();
    write_result = -EIO;
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == -EIO);
    assert(ppg_file.retired && ppg_file.retired_handle_live && writes == 1);
    assert(sensor_write_to_file("1234567890123456", PPG_RECORD_BYTES, ppg) == -EIO);
    assert(close_sensor_file(&ppg_file) == -EIO);
    assert(writes == 1 && closes == 0);
    release_retired_sensor_file(&ppg_file);
    assert(writes == 1 && closes == 1 && !ppg_file.retired &&
           !ppg_file.retired_handle_live && !ppg_file.file_open);
}

int main(void)
{
    test_accel_record_carry();
    test_exact_ppg_capacity_and_names();
    test_random_base_is_stable();
    test_rollover_stops_when_storage_fills();
    test_numbered_log_names();
    test_log_terminal_truncation();
    test_collision_terminal_close_and_no_retry();
    puts("PPG direct-write rollover checks passed");
    return 0;
}
