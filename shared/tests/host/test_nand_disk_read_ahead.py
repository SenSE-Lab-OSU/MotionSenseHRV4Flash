"""Host checks for bounded asynchronous USB MSC read-ahead."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract_function(source, name):
    match = re.search(
        r"(?:static\s+)?(?:bool|int|void)\s+" + name + r"\([^;]*?\)\s*\{", source
    )
    if match is None:
        raise RuntimeError(f"production function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    root = Path(__file__).resolve().parents[2]
    source = (root / "drivers" / "nand" / "nand_disk.c").read_text()
    functions = "\n\n".join(
        extract_function(source, name)
        for name in (
            "read_ahead_invalidate_locked",
            "disk_nand_read_sector",
            "read_ahead_find_locked",
            "read_ahead_schedule_locked",
            "read_ahead_replenish_locked",
            "read_ahead_handler",
            "disk_nand_read_ahead_quiesce",
        )
    )
    read_function = re.search(
        r"int\s+disk_nand_access_read\([^;]*?\)\s*\{", source
    )
    if read_function is None:
        raise RuntimeError("production function not found: disk_nand_access_read")
    depth, end = 1, read_function.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    functions += "\n\n" + source[read_function.start():end]

    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define USB_READ_AHEAD_SECTORS 2
#define NAND_SECTOR_SIZE 4096
#define K_FOREVER 0
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ARG_UNUSED(value) (void)(value)
#ifndef __aligned
#define __aligned(value) __attribute__((aligned(value)))
#endif
#define __ASSERT_NO_MSG(condition) assert(condition)
#define LOG_DBG(...) do { } while (0)
#define LOG_WRN(...) do { } while (0)
#define LOG_ERR(...) do { } while (0)

struct device { uint32_t sectors; };
struct disk_info { const struct device *dev; };
struct k_work { int unused; };
struct k_mutex { int unused; };
struct read_ahead_slot {
    uint32_t sector;
    bool valid;
    uint8_t data[NAND_SECTOR_SIZE] __aligned(4);
};

static struct read_ahead_slot read_ahead[USB_READ_AHEAD_SECTORS];
static struct k_work read_ahead_work;
static struct k_mutex disk_access_mutex;
static uint32_t read_ahead_next_sector;
static uint32_t read_ahead_remaining;
static uint32_t update_counter;
static const uint32_t file_table_sector_num = 180;
static const struct device test_device = { .sectors = 1000 };
static struct disk_info nand_disk = { .dev = &test_device };
static const char *current_thread_name = "usb_mass";
static int physical_reads;
static int fail_sector = -1;
static bool defer_work;
static bool work_queued;
static bool read_on_yield;
static uint32_t yield_sector;
static uint8_t yield_buf[NAND_SECTOR_SIZE];

int disk_nand_access_read(struct disk_info *disk, uint8_t *buf,
                          uint32_t sector, uint32_t count);

static void k_mutex_lock(struct k_mutex *mutex, int timeout)
{
    (void)mutex;
    (void)timeout;
}

static void k_mutex_unlock(struct k_mutex *mutex) { (void)mutex; }
static void k_yield(void)
{
    if (read_on_yield) {
        read_on_yield = false;
        defer_work = true;
        assert(disk_nand_access_read(&nand_disk, yield_buf, yield_sector, 1) == 0);
        defer_work = false;
    }
}
static void *k_current_get(void) { return NULL; }
static const char *k_thread_name_get(void *thread)
{
    (void)thread;
    return current_thread_name;
}
static uint32_t dev_total_sector_count(const struct device *dev)
{
    return dev->sectors;
}
static void fill_sector(uint8_t *buf, uint32_t sector)
{
    memset(buf, (uint8_t)sector, NAND_SECTOR_SIZE);
}
static int physical_read(uint8_t *buf, uint32_t sector)
{
    assert(sector < test_device.sectors);
    physical_reads++;
    if ((int)sector == fail_sector) {
        fail_sector = -1;
        return -5;
    }
    fill_sector(buf, sector);
    return 0;
}
static int file_table_access(void *buf, int sector, bool write)
{
    assert(!write);
    return physical_read(buf, (uint32_t)sector);
}
static int multi_nand_page_read(const struct device *dev, uint32_t sector,
                                uint8_t *buf)
{
    (void)dev;
    return physical_read(buf, sector);
}
static void print_flash_status_info(void) { }
static int k_work_submit(struct k_work *work);

/* PRODUCTION_FUNCTIONS */

static int k_work_submit(struct k_work *work)
{
	if (defer_work) {
		work_queued = true;
		return 0;
	}
    read_ahead_handler(work);
    return 0;
}

static void expect_sector(const uint8_t *buf, uint32_t sector)
{
    for (int i = 0; i < NAND_SECTOR_SIZE; i++) {
        assert(buf[i] == (uint8_t)sector);
    }
}

int main(void)
{
    uint8_t buf[NAND_SECTOR_SIZE];

    assert(disk_nand_access_read(&nand_disk, buf, 200, 1) == 0);
    expect_sector(buf, 200);
    assert(physical_reads == 3);

    for (uint32_t sector = 201; sector <= 204; sector++) {
        assert(read_ahead_find_locked(sector) >= 0);
        int reads_before = physical_reads;
        assert(disk_nand_access_read(&nand_disk, buf, sector, 1) == 0);
        expect_sector(buf, sector);
        assert(physical_reads == reads_before + 1);
        assert(read_ahead_find_locked(sector + USB_READ_AHEAD_SECTORS) >= 0);
    }
    assert(physical_reads == 7);

    current_thread_name = "main";
    assert(disk_nand_access_read(&nand_disk, buf, 205, 1) == 0);
    expect_sector(buf, 205);
    assert(physical_reads == 8);

    current_thread_name = "usb_mass";
    fail_sector = 301;
    assert(disk_nand_access_read(&nand_disk, buf, 300, 1) == 0);
    expect_sector(buf, 300);
    assert(read_ahead_remaining == 0);

    disk_nand_read_ahead_quiesce();
    int reads_before_interleave = physical_reads;
    yield_sector = 998;
    read_on_yield = true;
    assert(disk_nand_access_read(&nand_disk, buf, 997, 1) == 0);
    expect_sector(buf, 997);
    expect_sector(yield_buf, 998);
    assert(!read_on_yield);
    assert(physical_reads == reads_before_interleave + 3);
    assert(read_ahead_find_locked(999) >= 0);

    disk_nand_read_ahead_quiesce();
    int reads_before_end = physical_reads;
    assert(disk_nand_access_read(&nand_disk, buf, 998, 1) == 0);
    expect_sector(buf, 998);
    assert(physical_reads == reads_before_end + 2);
    assert(read_ahead_find_locked(999) >= 0);
    assert(disk_nand_access_read(&nand_disk, buf, 999, 1) == 0);
    expect_sector(buf, 999);
    assert(physical_reads == reads_before_end + 2);
    assert(read_ahead_remaining == 0);

	defer_work = true;
	assert(disk_nand_access_read(&nand_disk, buf, 400, 1) == 0);
	expect_sector(buf, 400);
	assert(work_queued && read_ahead_remaining == USB_READ_AHEAD_SECTORS);
	int reads_before_quiesce = physical_reads;
	disk_nand_read_ahead_quiesce();
	defer_work = false;
	read_ahead_handler(&read_ahead_work);
	assert(physical_reads == reads_before_quiesce);

    puts("NAND USB read-ahead checks passed");
    return 0;
}
'''.replace("/* PRODUCTION_FUNCTIONS */", functions)

    cc = shutil.which("gcc") or r"C:\cygwin64\bin\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="nand-read-ahead-") as temp:
        source_path = Path(temp) / "harness.c"
        executable = Path(temp) / "harness.exe"
        source_path.write_text(harness)
        subprocess.run(
            [cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
             str(source_path), "-o", str(executable)],
            env=env,
            check=True,
        )
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    main()
