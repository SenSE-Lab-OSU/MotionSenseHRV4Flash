"""Compile ECG log direct-write rollover functions against deterministic stubs."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract(source, name):
    match = re.search(r"static\s+(?:int|void)\s+" + name +
                      r"\([^;]*?\)\s*\{", source)
    if match is None:
        raise RuntimeError(f"production function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def contains_identifier(text, identifier):
    return re.search(r"\b" + re.escape(identifier) + r"\b", text) is not None


def main():
    here = Path(__file__).resolve().parent
    source = (here.parents[1] / "src" / "zephyrfilesystem.c").read_text()
    header = (here.parents[1] / "src" / "zephyrfilesystem.h").read_text()
    assert "#if FF_FS_TINY != 0" in source
    assert "#if FF_MAX_SS != 4096" in source
    assert "#define ECG_FILE_FORMAT \"ECB2 4096-byte MAX30001 ECG blocks with CRC-32\"" in source
    assert "#define TEST_FILE_PREALLOCATED_BYTES RECORDING_FILE_BYTES" in source
    assert "f_expand((FIL *)test_file.filep, TEST_FILE_PREALLOCATED_BYTES, 1)" in source
    assert contains_identifier("int store_data(void);", "store_data")
    assert not contains_identifier("logger_write_to_file", "write_to_file")
    for legacy in ("ecg_file", "ecg_work_item", "sensor_type",
                   "passthrough", "sensor_write_to_file",
                   "write_to_file", "submit_write", "store_data",
                   "flush_data_buffer", "sensor_enum_to_string",
                   "enable_read_only", "filesystem_set_collection_id",
                   "filesystem_clear_collection_id", "max_writes"):
        assert not contains_identifier(source, legacy)
        assert not contains_identifier(header, legacy)
    assert "int filesystem_logger_append(" in source
    assert "int filesystem_logger_flush(" in source
    assert "logger_retired_handle_live" in source
    names = ("logger_release_retired_file", "logger_retire_file", "logger_close_current_file",
             "logger_activate_next_file", "logger_rollover_file",
             "logger_write_bytes")
    functions = [extract(source, name) for name in names]
    assert "fs_write" not in functions[0] and "fs_unlink" not in functions[0]
    assert "fs_truncate" not in functions[0]
    assert "logger_retired_handle_live" in functions[0]
    assert "fs_sync" not in functions[2]
    assert functions[2].index("fs_truncate") < functions[2].index("fs_close")
    harness = (here / "logger_rollover_harness.c").read_text()
    generated = harness.replace("/* PRODUCTION_FUNCTIONS */", "\n\n".join(functions))
    cc = shutil.which("gcc") or r"C:\\cygwin64\\bin\\gcc.exe"
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix="logger-rollover-") as temp:
        source_path = Path(temp) / "harness.c"
        executable = Path(temp) / "harness.exe"
        source_path.write_text(generated)
        subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        str(source_path), "-o", str(executable)], env=env, check=True)
        subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    main()
