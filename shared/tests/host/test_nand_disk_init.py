"""Host checks that NAND disk registration is gated by hardware init."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract_function(source, name):
    match = re.search(r"static\s+int\s+" + name + r"\([^;]*?\)\s*\{", source)
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
    function = extract_function(source, "nand_disk_device_init")
    harness = r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>

#define LOG_ERR(...) do { } while (0)

struct device { int unused; };
struct disk_info {
    const struct device *dev;
    const char *name;
};

static struct disk_info nand_disk;
static int init_ret, register_ret, init_calls, register_calls;

static int spi_init(const struct device *dev)
{
    (void)dev;
    init_calls++;
    return init_ret;
}

static int disk_access_register(struct disk_info *disk)
{
    assert(disk == &nand_disk);
    register_calls++;
    return register_ret;
}

/* PRODUCTION_FUNCTION */

int main(void)
{
    const struct device dev = { 0 };

    init_ret = -ENODEV;
    assert(nand_disk_device_init(&dev) == -ENODEV);
    assert(init_calls == 1 && register_calls == 0);

    init_ret = 0;
    register_ret = -EEXIST;
    assert(nand_disk_device_init(&dev) == -EEXIST);
    assert(init_calls == 2 && register_calls == 1);

    register_ret = 0;
    assert(nand_disk_device_init(&dev) == 0);
    assert(init_calls == 3 && register_calls == 2);
    assert(nand_disk.dev == &dev);
    assert(nand_disk.name != NULL);
    puts("NAND disk init checks passed");
    return 0;
}
'''.replace("/* PRODUCTION_FUNCTION */", function)

    cc = shutil.which("gcc") or r"C:\cygwin64\bin\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="nand-disk-init-") as temp:
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
