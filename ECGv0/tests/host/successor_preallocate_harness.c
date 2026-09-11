#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <errno.h>

#define FS_O_CREATE 1
#define FS_O_WRITE 2
#define FR_OK 0
typedef int FRESULT;
typedef int FIL;
struct fs_file_t { void *filep; };
struct fs_dirent { int unused; };

static int stat_result;
static int open_result;
static int expand_result;
static ssize_t write_result;
static int sync_result;
static int close_result;
static int writes;
static int closes;
static int unlinks;

static void fs_file_t_init(struct fs_file_t *file) { file->filep = NULL; }
static int fs_stat(const char *path, struct fs_dirent *entry)
{
    (void)path; (void)entry; return stat_result;
}
static int fs_open(struct fs_file_t *file, const char *path, int flags)
{
    (void)path; (void)flags;
    if (open_result == 0) file->filep = file;
    return open_result;
}
static FRESULT f_expand(FIL *file, uint32_t bytes, int contiguous)
{
    (void)file; (void)bytes; (void)contiguous; return expand_result;
}
static ssize_t fs_write(struct fs_file_t *file, const void *data, size_t bytes)
{
    (void)file; (void)data; writes++;
    return write_result == -9999 ? (ssize_t)bytes : write_result;
}
static int fs_sync(struct fs_file_t *file) { (void)file; return sync_result; }
static int fs_close(struct fs_file_t *file)
{
    (void)file; closes++; return close_result;
}
static int fs_unlink(const char *path) { (void)path; unlinks++; return 0; }

/* FILESYSTEM_PREALLOCATE_FUNCTION */

static void reset(void)
{
    stat_result = -ENOENT;
    open_result = expand_result = sync_result = close_result = 0;
    write_result = -9999;
    writes = closes = unlinks = 0;
}

int main(void)
{
    struct fs_file_t file;
    uint8_t header[4096] = {0};

    reset();
    assert(filesystem_preallocate_file(&file, "/reserve", 65536U,
                                       NULL, 0U, false) == 0);
    assert(writes == 0 && closes == 1 && unlinks == 0);

    reset();
    sync_result = -EIO;
    assert(filesystem_preallocate_file(&file, "/reserve", 65536U,
                                       NULL, 0U, false) == -EIO);
    assert(writes == 0 && closes == 1 && unlinks == 1);

    reset();
    expand_result = 1;
    assert(filesystem_preallocate_file(&file, "/reserve", 65536U,
                                       NULL, 0U, false) == -EIO);
    assert(writes == 0 && closes == 1 && unlinks == 1);

    reset();
    write_result = 100;
    assert(filesystem_preallocate_file(&file, "/active", 65536U,
                                       header, sizeof(header), true) == -EIO);
    assert(writes == 1 && closes == 1 && unlinks == 0);

    reset();
    write_result = -ENOSPC;
    assert(filesystem_preallocate_file(&file, "/active", 65536U,
                                       header, sizeof(header), true) == -ENOSPC);
    assert(writes == 1 && closes == 1 && unlinks == 0);

    reset();
    sync_result = -EIO;
    assert(filesystem_preallocate_file(&file, "/active", 65536U,
                                       header, sizeof(header), true) == -EIO);
    assert(writes == 1 && closes == 1 && unlinks == 0);

    puts("successor preallocation checks passed");
    return 0;
}
