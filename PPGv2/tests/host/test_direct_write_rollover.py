"""Compile the production PPG fixed-capacity write path against host I/O stubs."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract(source, name):
    match = re.search(r"static\s+(?:int|void|MotionSenseFile\s*\*)\s*" +
                      name + r"\([^;]*?\)\s*\{", source)
    if match is None:
        raise RuntimeError(f"production function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    here = Path(__file__).resolve().parent
    source = (here.parents[1] / "src" / "zephyrfilesystem.c").read_text()
    for contract in ("#if FF_FS_TINY != 0", "#if FF_MAX_SS != 4096",
                     "PPG_RECORD_BYTES 16U", "ACCEL_RECORD_BYTES 26U"):
        assert contract in source, contract
    functions = [extract(source, name) for name in (
        "sensor_write_failure", "sensor_file", "sensor_make_path",
        "close_sensor_file", "release_retired_sensor_file", "open_sensor_file", "rollover_sensor_file",
        "sensor_write_to_file")]
    close_sensor = functions[3]
    assert "msense_file == &log_file" in close_sensor
    assert close_sensor.index("fs_truncate") < close_sensor.index("fs_close")
    assert "retired_handle_live" in close_sensor
    assert "fs_truncate" not in functions[4]
    assert "retired_handle_live" in functions[4]
    harness = (here / "ppg_direct_write_harness.c").read_text()
    generated = harness.replace("/* PRODUCTION_FUNCTIONS */", "\n\n".join(functions))
    cc = shutil.which("gcc") or r"C:\cygwin64\bin\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="ppg-direct-write-") as temp:
        output = Path(temp) / "harness.c"
        executable = Path(temp) / "harness.exe"
        output.write_text(generated)
        subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        str(output), "-o", str(executable)], env=env, check=True)
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    main()
