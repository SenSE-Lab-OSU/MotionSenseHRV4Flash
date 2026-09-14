"""Host checks for NAND command sequencing and bounded ready polling."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract_function(source, name):
    match = re.search(
        r"(?:static\s+)?(?:int|uint8_t)\s+" + name + r"\([^;]*?\)\s*\{",
        source,
    )
    if match is None:
        raise RuntimeError(f"production function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    here = Path(__file__).resolve().parent
    source = (here.parents[1] / "drivers" / "nand" / "spi_nand.c").read_text()
    defines = "\n".join(
        re.findall(r"^#define NAND_(?:STATUS|PAGE|BLOCK|RESET)[^\n]+", source, re.M)
    )
    functions = "\n\n".join(
        extract_function(source, name)
        for name in (
            "get_features",
            "get_status",
            "spi_nand_wait_until_ready",
            "spi_nand_page_write",
            "spi_nand_block_erase",
            "spi_nand_reset",
            "flash_reset_and_unlock",
        )
    )
    harness = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#define BIT(n) (1U << (n))
#define SPI_NOR_WIP_BIT BIT(0)
#define SPI_NAND_GF 0x0f
#define SPI_NAND_PL 0x02
#define SPI_NAND_PE 0x10
#define SPI_NAND_RESET 0xff
#define SPI_NOR_CMD_BE 0xd8
#define REGISTER_STATUS 0xc0
#define K_USEC(value) (value)
#define LOG_DBG(...) do { } while (0)
#define LOG_ERR(...) do { } while (0)
#define LOG_WRN(...) do { } while (0)

/* PRODUCTION_DEFINES */

struct spi_flash_config { int dies_per_flash; };
struct device { const void *config; };
typedef uint64_t k_timepoint_t;
typedef struct {
    uint8_t opcode;
    bool is_write;
    const void *addr;
    size_t addr_length;
    const void *data;
    size_t data_length;
} spi_send_request;

enum event {
    EV_LOCK,
    EV_WREN,
    EV_LOAD,
    EV_EXECUTE,
    EV_ERASE,
    EV_GET,
    EV_WRDI,
    EV_RESET,
    EV_SET_DIE,
    EV_UNLOCK_MEMORY,
    EV_UNLOCK,
};

static const struct spi_flash_config test_config = { .dies_per_flash = 2 };
static const struct device test_device = { .config = &test_config };
static enum event events[256];
static uint64_t event_times[256];
static int event_count;
static int wren_ret, load_ret, execute_ret, erase_ret, wrdi_ret;
static int reset_ret;
static int get_error_call, get_error_ret, get_calls, sleeps;
static int set_die_error_call, set_die_error_ret, set_die_calls;
static int unlock_error_call, unlock_error_ret, unlock_calls;
static int selected_dies[8];
static uint8_t status_values[16], default_status;
static size_t status_count, status_index;
static uint64_t now_us;
static uint64_t sleep_extra_us;
static int current_writes, current_erases;
static int current_flash;
static int current_die[1];

static void record(enum event event)
{
    assert(event_count < (int)(sizeof(events) / sizeof(events[0])));
    event_times[event_count] = now_us;
    events[event_count++] = event;
}

static void reset_fake(void)
{
    memset(events, 0, sizeof(events));
    memset(event_times, 0, sizeof(event_times));
    memset(status_values, 0, sizeof(status_values));
    memset(selected_dies, 0, sizeof(selected_dies));
    event_count = get_calls = sleeps = set_die_calls = unlock_calls = 0;
    wren_ret = load_ret = execute_ret = erase_ret = wrdi_ret = 0;
    reset_ret = 0;
    get_error_call = -1;
    get_error_ret = -EIO;
    set_die_error_call = unlock_error_call = -1;
    set_die_error_ret = unlock_error_ret = -EIO;
    status_count = status_index = 0U;
    default_status = 0U;
    now_us = 0U;
    sleep_extra_us = 0U;
}

static void set_statuses(const uint8_t *values, size_t count)
{
    assert(count <= sizeof(status_values));
    memcpy(status_values, values, count);
    status_count = count;
    status_index = 0U;
}

static int spi_nand_access(const struct device *dev, spi_send_request *request)
{
    (void)dev;
    switch (request->opcode) {
    case SPI_NAND_GF:
        record(EV_GET);
        if (get_calls++ == get_error_call) {
            return get_error_ret;
        }
        *(uint8_t *)request->data = status_index < status_count
            ? status_values[status_index++] : default_status;
        return 0;
    case SPI_NAND_PL:
        record(EV_LOAD);
        return load_ret;
    case SPI_NAND_PE:
        record(EV_EXECUTE);
        return execute_ret;
    case SPI_NOR_CMD_BE:
        record(EV_ERASE);
        return erase_ret;
    default:
        assert(false);
        return -EINVAL;
    }
}

static k_timepoint_t sys_timepoint_calc(uint64_t timeout_us)
{
    return now_us + timeout_us;
}

static bool sys_timepoint_expired(k_timepoint_t deadline)
{
    return now_us >= deadline;
}

static void k_sleep(uint64_t duration_us)
{
    sleeps++;
    now_us += duration_us + sleep_extra_us;
    sleep_extra_us = 0U;
}

static void acquire_device(const struct device *dev)
{
    (void)dev;
    record(EV_LOCK);
}

static void release_device(const struct device *dev)
{
    (void)dev;
    record(EV_UNLOCK);
}

static int write_enable(const struct device *dev)
{
    (void)dev;
    record(EV_WREN);
    return wren_ret;
}

static int write_disable(const struct device *dev)
{
    (void)dev;
    record(EV_WRDI);
    return wrdi_ret;
}

static int spi_cmd(const struct device *dev, uint8_t opcode, void *dest,
                   size_t length)
{
    (void)dev;
    (void)dest;
    (void)length;
    assert(opcode == SPI_NAND_RESET);
    record(EV_RESET);
    return reset_ret;
}

static int set_die(const struct device *dev, int die)
{
    (void)dev;
    record(EV_SET_DIE);
    selected_dies[set_die_calls] = die;
    if (set_die_calls++ == set_die_error_call) {
        return set_die_error_ret;
    }
    current_die[current_flash] = die;
    return 0;
}

static int spi_unlock_memory(const struct device *dev)
{
    (void)dev;
    record(EV_UNLOCK_MEMORY);
    if (unlock_calls++ == unlock_error_call) {
        return unlock_error_ret;
    }
    return 0;
}

/* PRODUCTION_FUNCTIONS */

static int count_event(enum event wanted)
{
    int count = 0;
    for (int i = 0; i < event_count; ++i) {
        count += events[i] == wanted;
    }
    return count;
}

static void assert_released_once(void)
{
    assert(count_event(EV_LOCK) == 1);
    assert(count_event(EV_UNLOCK) == 1);
    assert(events[0] == EV_LOCK);
    assert(events[event_count - 1] == EV_UNLOCK);
}

static void test_feature_error(void)
{
    uint8_t status = 0;
    reset_fake();
    get_error_call = 0;
    assert(get_features(&test_device, REGISTER_STATUS, &status) == -EIO);
    assert(get_calls == 1 && sleeps == 0);
    assert(get_features(&test_device, REGISTER_STATUS, NULL) == -EINVAL);
}

static void test_wait_bounds(void)
{
    uint8_t status = 0;
    const uint8_t ready_before[] = { BIT(0), BIT(0), 0 };
    const uint8_t ready_after_final_sleep[] = { BIT(0), BIT(0), BIT(0), 0 };
    const uint8_t ready_after_delayed_wake[] = { BIT(0), 0 };

    reset_fake();
    default_status = BIT(0);
    assert(spi_nand_wait_until_ready(&test_device, 250U, &status) == -ETIMEDOUT);
    assert(get_calls == 4 && sleeps == 3 && now_us == 300U);

    reset_fake();
    default_status = BIT(0);
    assert(spi_nand_wait_until_ready(&test_device, 200U, &status) == -ETIMEDOUT);
    assert(get_calls == 3 && sleeps == 2 && now_us == 200U);

    reset_fake();
    set_statuses(ready_before, sizeof(ready_before));
    assert(spi_nand_wait_until_ready(&test_device, 250U, &status) == 0);
    assert(status == 0 && get_calls == 3 && sleeps == 2 && now_us == 200U);

    reset_fake();
    set_statuses(ready_after_final_sleep, sizeof(ready_after_final_sleep));
    assert(spi_nand_wait_until_ready(&test_device, 250U, &status) == 0);
    assert(status == 0 && get_calls == 4 && sleeps == 3 && now_us == 300U);

    reset_fake();
    set_statuses(ready_after_delayed_wake, sizeof(ready_after_delayed_wake));
    sleep_extra_us = 250U;
    assert(spi_nand_wait_until_ready(&test_device, 200U, &status) == 0);
    assert(status == 0 && get_calls == 2 && sleeps == 1 && now_us == 350U);
}

static void test_operation_status_bits(void)
{
    const uint8_t busy_ready[] = { BIT(0), 0 };
    uint8_t page[16] = { 0 };

    reset_fake();
    set_statuses(busy_ready, sizeof(busy_ready));
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == 0);
    assert(count_event(EV_WREN) == 1 && count_event(EV_LOAD) == 1);
    assert(count_event(EV_EXECUTE) == 1 && count_event(EV_WRDI) == 1);
    assert(get_calls == 2 && sleeps == 1);
    assert_released_once();

    reset_fake();
    default_status = 0x10 | NAND_STATUS_ERASE_FAIL;
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == 0);
    assert_released_once();

    reset_fake();
    default_status = NAND_STATUS_PROGRAM_FAIL;
    wrdi_ret = -ENOSPC;
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == -EIO);
    assert_released_once();

    reset_fake();
    default_status = 0x10 | NAND_STATUS_PROGRAM_FAIL;
    assert(spi_nand_block_erase(&test_device, 64) == 0);
    assert_released_once();

    reset_fake();
    default_status = NAND_STATUS_ERASE_FAIL;
    assert(spi_nand_block_erase(&test_device, 64) == -EIO);
    assert_released_once();
}

static void test_program_failures(void)
{
    const uint8_t busy_ready[] = { BIT(0), 0 };
    uint8_t page[16] = { 0 };

    reset_fake();
    wren_ret = -EACCES;
    wrdi_ret = -ENOSPC;
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == -EACCES);
    assert(count_event(EV_LOAD) == 0 && count_event(EV_EXECUTE) == 0);
    assert(count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    load_ret = -EINVAL;
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == -EINVAL);
    assert(count_event(EV_EXECUTE) == 0 && count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    execute_ret = -EIO;
    set_statuses(busy_ready, sizeof(busy_ready));
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == -EIO);
    assert(count_event(EV_EXECUTE) == 1 && get_calls == 2);
    assert(count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    get_error_call = 0;
    get_error_ret = -EBUSY;
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == -EBUSY);
    assert(count_event(EV_EXECUTE) == 1 && count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    wrdi_ret = -EIO;
    assert(spi_nand_page_write(&test_device, 7, page, sizeof(page)) == -EIO);
    assert_released_once();
}

static void test_erase_paths(void)
{
    const uint8_t busy_ready[] = { BIT(0), 0 };

    reset_fake();
    wren_ret = -EACCES;
    wrdi_ret = -ENOSPC;
    assert(spi_nand_block_erase(&test_device, 64) == -EACCES);
    assert(count_event(EV_ERASE) == 0 && count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    assert(spi_nand_block_erase(&test_device, 64) == 0);
    assert(count_event(EV_ERASE) == 1 && count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    erase_ret = -EIO;
    wrdi_ret = -ENOSPC;
    set_statuses(busy_ready, sizeof(busy_ready));
    assert(spi_nand_block_erase(&test_device, 64) == -EIO);
    assert(count_event(EV_ERASE) == 1 && get_calls == 2);
    assert(count_event(EV_WRDI) == 1);
    assert_released_once();

    reset_fake();
    default_status = BIT(0);
    assert(spi_nand_block_erase(&test_device, 64) == -ETIMEDOUT);
    assert(get_calls <= 121 && count_event(EV_ERASE) == 1);
    assert(count_event(EV_WRDI) == 1);
    assert_released_once();
}

static void test_reset_sequence(void)
{
    const uint8_t busy_ready[] = { BIT(0), 0 };

    reset_fake();
    set_statuses(busy_ready, sizeof(busy_ready));
    assert(flash_reset_and_unlock(&test_device) == 0);
    assert(events[0] == EV_LOCK && events[1] == EV_RESET);
    assert(count_event(EV_RESET) == 1 && count_event(EV_GET) == 2);
    assert(event_times[2] == NAND_RESET_NO_COMMAND_US);
    assert(events[4] == EV_SET_DIE && events[5] == EV_UNLOCK_MEMORY);
    assert(events[6] == EV_SET_DIE && events[7] == EV_UNLOCK_MEMORY);
    assert(events[8] == EV_SET_DIE);
    assert(selected_dies[0] == 0 && selected_dies[1] == 1);
    assert(selected_dies[2] == 0 && set_die_calls == 3);
    assert(unlock_calls == 2 && current_die[0] == 0);
    assert_released_once();

    reset_fake();
    reset_ret = -EIO;
    assert(flash_reset_and_unlock(&test_device) == -EIO);
    assert(now_us == NAND_RESET_NO_COMMAND_US);
    assert(count_event(EV_GET) == 0 && count_event(EV_SET_DIE) == 0);
    assert(count_event(EV_UNLOCK_MEMORY) == 0);
    assert_released_once();

    reset_fake();
    default_status = BIT(0);
    assert(flash_reset_and_unlock(&test_device) == -ETIMEDOUT);
    assert(now_us == NAND_RESET_NO_COMMAND_US + NAND_RESET_POLL_TIMEOUT_US);
    assert(count_event(EV_SET_DIE) == 0 && count_event(EV_UNLOCK_MEMORY) == 0);
    assert_released_once();

    reset_fake();
    unlock_error_call = 1;
    unlock_error_ret = -EBUSY;
    set_die_error_call = 2;
    set_die_error_ret = -ENOSPC;
    assert(flash_reset_and_unlock(&test_device) == -EBUSY);
    assert(set_die_calls == 3 && selected_dies[2] == 0);
    assert_released_once();

    reset_fake();
    set_die_error_call = 2;
    set_die_error_ret = -ENOSPC;
    assert(flash_reset_and_unlock(&test_device) == -ENOSPC);
    assert(set_die_calls == 3 && unlock_calls == 2);
    assert_released_once();
}

int main(void)
{
    test_feature_error();
    test_wait_bounds();
    test_operation_status_bits();
    test_program_failures();
    test_erase_paths();
    test_reset_sequence();
    puts("SPI NAND operation checks passed");
    return 0;
}
'''
    generated = harness.replace("/* PRODUCTION_DEFINES */", defines)
    generated = generated.replace("/* PRODUCTION_FUNCTIONS */", functions)
    cc = shutil.which("gcc") or r"C:\cygwin64\bin\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="spi-nand-operations-") as temp:
        source_path = Path(temp) / "harness.c"
        executable = Path(temp) / "harness.exe"
        source_path.write_text(generated)
        subprocess.run(
            [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", str(source_path),
             "-o", str(executable)],
            env=env,
            check=True,
        )
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    main()
