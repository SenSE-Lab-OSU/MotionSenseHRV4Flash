"""Compile UUID direct write/close behavior against deterministic filesystem stubs."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract(source, name):
    match = re.search(r"static\s+int\s+" + name + r"\([^;]*?\)\s*\{", source)
    if match is None:
        raise RuntimeError(f"production function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    here = Path(__file__).resolve().parent
    source = (here.parents[1] / "uuid_file.c").read_text()
    function = extract(source, "write_uuid_contents")
    assert "fs_sync" not in function
    harness = r'''
#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>
#define FS_O_CREATE 1
#define FS_O_WRITE 2
struct fs_file_t { void *filep; };
static int open_result, close_result, write_result, opens, writes, closes;
static void fs_file_t_init(struct fs_file_t *file) { file->filep = NULL; }
static int fs_open(struct fs_file_t *file, const char *path, int flags)
{ (void)path; (void)flags; opens++; if (!open_result) file->filep = file; return open_result; }
static ssize_t fs_write(struct fs_file_t *file, const void *contents, size_t length)
{ (void)file; (void)contents; writes++; return write_result == -9999 ? (ssize_t)length : write_result; }
static int fs_close(struct fs_file_t *file) { (void)file; closes++; return close_result; }
/* PRODUCTION_FUNCTION */
static void reset(void)
{ open_result = close_result = 0; write_result = -9999; opens = writes = closes = 0; }
int main(void)
{
 reset(); assert(write_uuid_contents("/SD:/uuid.txt", "uuid", 4U) == 0);
 assert(opens == 1 && writes == 1 && closes == 1);
 reset(); write_result = -EIO;
 assert(write_uuid_contents("/SD:/uuid.txt", "uuid", 4U) == -EIO);
 assert(opens == 1 && writes == 1 && closes == 0);
 reset(); write_result = 3;
 assert(write_uuid_contents("/SD:/uuid.txt", "uuid", 4U) == -EIO);
 assert(opens == 1 && writes == 1 && closes == 0);
 reset(); close_result = -EIO;
 assert(write_uuid_contents("/SD:/uuid.txt", "uuid", 4U) == -EIO);
 assert(opens == 1 && writes == 1 && closes == 1);
 puts("uuid direct-write checks passed"); return 0;
}
'''
    generated = harness.replace("/* PRODUCTION_FUNCTION */", function)
    cc = shutil.which("gcc") or r"C:\\cygwin64\\bin\\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="uuid-file-") as temp:
        source_path = Path(temp) / "harness.c"
        executable = Path(temp) / "harness.exe"
        source_path.write_text(generated)
        subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        str(source_path), "-o", str(executable)], env=env, check=True)
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    main()
