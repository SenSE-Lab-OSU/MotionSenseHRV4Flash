"""Exit ECG ship mode through the debugger without changing its firmware."""

import argparse
import pathlib
import socket
import subprocess
import sys
import time


DEFAULT_SERIAL = "000261006427"
DEFAULT_ELF = pathlib.Path(r"C:\nathan\MotionSenseHRV4Flash\ECGv0\build\ECGv0\zephyr\zephyr.elf")
DEFAULT_HEX = pathlib.Path(r"C:\nathan\MotionSenseHRV4Flash\ECGv0\build\merged.hex")
DEFAULT_SOURCE = pathlib.Path(r"C:\nathan\MotionSenseHRV4Flash\ECGv0\src\main.c")
DEFAULT_GDB = pathlib.Path(
    r"C:\ncs\toolchains\b620d30767\opt\zephyr-sdk\arm-zephyr-eabi\bin"
    r"\arm-zephyr-eabi-gdb.exe"
)
DEFAULT_SERVER = pathlib.Path(r"C:\Program Files\SEGGER\JLink_V924a\JLinkGDBServerCL.exe")


def run_checked(arguments, timeout=120):
    print("RUN " + subprocess.list2cmdline([str(item) for item in arguments]), flush=True)
    return subprocess.run(arguments, check=True, timeout=timeout)


def ship_wait_line(source_path):
    marker = 'LOG_INF("Ship mode active; press button0 to enable BLE and status LEDs")'
    for number, line in enumerate(source_path.read_text(encoding="utf-8").splitlines(), 1):
        if marker in line:
            return number
    raise RuntimeError("ship-mode log marker was not found in the ECG source")


def wait_for_server(port, process, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"J-Link GDB server exited early with {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                return
        except OSError:
            time.sleep(0.2)
    raise TimeoutError("J-Link GDB server did not open its GDB port")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default=DEFAULT_SERIAL)
    parser.add_argument("--elf", type=pathlib.Path, default=DEFAULT_ELF)
    parser.add_argument("--firmware", type=pathlib.Path, default=DEFAULT_HEX)
    parser.add_argument("--source", type=pathlib.Path, default=DEFAULT_SOURCE)
    parser.add_argument("--gdb", type=pathlib.Path, default=DEFAULT_GDB)
    parser.add_argument("--server", type=pathlib.Path, default=DEFAULT_SERVER)
    parser.add_argument("--gdb-port", type=int, default=2331)
    parser.add_argument("--reset-kind", choices=("RESET_PIN", "RESET_SYSTEM"),
                        default="RESET_PIN")
    parser.add_argument("--skip-reset", action="store_true")
    args = parser.parse_args()

    for path in (args.elf, args.firmware, args.source, args.gdb, args.server):
        if not path.is_file():
            raise FileNotFoundError(path)

    line = ship_wait_line(args.source)
    run_checked([
        "nrfutil", "--log-output", "stdout", "device", "fw-verify",
        "--serial-number", args.serial, "--core", "application",
        "--firmware", str(args.firmware),
    ])
    if not args.skip_reset:
        run_checked([
            "nrfutil", "--log-output", "stdout", "device", "reset",
            "--serial-number", args.serial, "--core", "application",
            "--reset-kind", args.reset_kind,
        ])

    server_arguments = [
        str(args.server), "-select", f"USB={args.serial}", "-device",
        "nRF5340_xxAA_APP", "-if", "SWD", "-speed", "4000", "-port",
        str(args.gdb_port), "-swoport", str(args.gdb_port + 1), "-telnetport",
        str(args.gdb_port + 2), "-noir", "-silent",
    ]
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    server = subprocess.Popen(server_arguments, creationflags=creationflags)
    try:
        wait_for_server(args.gdb_port, server)
        source_spec = f"{args.source.as_posix()}:{line}"
        gdb_arguments = [
            str(args.gdb), "--quiet", "--batch", str(args.elf),
            "-ex", "set pagination off",
            "-ex", f"target remote 127.0.0.1:{args.gdb_port}",
            "-ex", "monitor reset",
            "-ex", "tbreak main",
            "-ex", "continue",
            "-ex", f"tbreak {source_spec}",
            "-ex", "continue",
            "-ex", "set variable ship_mode = SHIP_MODE_STARTING",
            "-ex", "set variable ship_mode_exit_sem.count = 1",
            "-ex", "tbreak battery_maintenance",
            "-ex", "continue",
            "-ex", "printf \"SHIP_MODE_EXIT_OK\\n\"",
            "-ex", "monitor go",
            "-ex", "disconnect",
        ]
        run_checked(gdb_arguments, timeout=180)
    finally:
        if server.poll() is None:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)

    print(f"SHIP_MODE_EXIT_COMPLETE serial={args.serial} reset={args.reset_kind}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"SHIP_MODE_EXIT_FAILED {error}", file=sys.stderr)
        sys.exit(1)
