"""Compile terminal ACB1 write/close sequencing against filesystem stubs."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract(source, name):
    match = re.search(r"(?:static\s+)?(?:int|void)\s+" + name +
                      r"\([^;]*?\)\s*\{", source)
    if match is None:
        raise RuntimeError(f"production function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    here = Path(__file__).resolve().parent
    source = (here.parents[1] / "src" / "accelRecorder.c").read_text()
    assert "block->sync_after_write = false;" in source
    assert "accel_record_retired_handle_live" in source
    functions = [extract(source, name) for name in (
        "accel_record_write_trailer", "accel_record_close_current_chunk",
        "accel_record_block_work_handler", "accel_recorder_filesystem_unmounted")]
    assert "fs_close" in functions[-1]
    assert "fs_write" not in functions[-1] and "fs_unlink" not in functions[-1]
    assert "if (accel_record_retired_handle_live)" in functions[-1]
    harness = (here / "accel_terminal_harness.c").read_text()
    generated = harness.replace("/* PRODUCTION_FUNCTIONS */", "\n\n".join(functions))
    cc = shutil.which("gcc") or r"C:\\cygwin64\\bin\\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="accel-terminal-") as temp:
        source_path = Path(temp) / "harness.c"
        executable = Path(temp) / "harness.exe"
        source_path.write_text(generated)
        subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        str(source_path), "-o", str(executable)], env=env, check=True)
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    main()
