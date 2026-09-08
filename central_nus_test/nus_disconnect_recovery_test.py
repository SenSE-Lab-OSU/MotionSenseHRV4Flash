"""Exercise NUS recovery after the Central disappears during a stream."""

import argparse
import subprocess
import time

import serial


def read_until(command, data, needle, timeout):
    deadline = time.monotonic() + timeout
    text = ""
    binary_bytes = 0
    while time.monotonic() < deadline:
        waiting = command.in_waiting
        if waiting:
            chunk = command.read(waiting).decode(errors="replace")
            text += chunk
            print(chunk, end="", flush=True)
            if needle in text:
                return text, binary_bytes
        binary_bytes += len(data.read(data.in_waiting or 1))
        time.sleep(0.01)
    raise RuntimeError(f"timeout waiting for {needle!r}; last output: {text[-500:]}")


def send(command, line):
    command.write((line + "\r\n").encode())
    command.flush()


def wait_for_binary(command, data, minimum, timeout):
    deadline = time.monotonic() + timeout
    count = 0
    while time.monotonic() < deadline:
        waiting = command.in_waiting
        if waiting:
            print(command.read(waiting).decode(errors="replace"), end="", flush=True)
        count += len(data.read(data.in_waiting or 1))
        if count >= minimum:
            return count
        time.sleep(0.01)
    raise RuntimeError(f"received only {count} binary bytes")


def connect_named(command, data, name):
    for _ in range(12):
        command.reset_input_buffer()
        send(command, "connect ecg")
        try:
            output, _ = read_until(command, data, "PEER_READY", 12)
        except RuntimeError:
            send(command, "disconnect")
            time.sleep(2)
            continue
        if f"peer_name={name} " in output:
            return
        send(command, "disconnect")
        try:
            read_until(command, data, "DISCONNECTED", 12)
        except RuntimeError:
            pass
    raise RuntimeError(f"could not select {name}")


def start_when_ready(command, data, session_id):
    for _ in range(4):
        command.reset_input_buffer()
        send(command, f"start {session_id}")
        try:
            return read_until(command, data, "START_ACK", 15)
        except RuntimeError:
            time.sleep(12)
            session_id += 1
    raise RuntimeError("peripheral history did not become ready")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-port", required=True)
    parser.add_argument("--data-port", required=True)
    parser.add_argument("--peer-name", required=True)
    parser.add_argument("--central-serial", required=True)
    parser.add_argument("--debugger-pause", action="store_true")
    args = parser.parse_args()

    with serial.Serial(args.command_port, 115200, timeout=0.05) as command, \
            serial.Serial(args.data_port, 1_000_000, timeout=0.02) as data:
        command.dtr = True
        data.dtr = True
        send(command, "disconnect")
        time.sleep(5)
        command.reset_input_buffer()
        connect_named(command, data, args.peer_name)
        if args.debugger_pause:
            input("PAUSED_BEFORE_INITIAL_START\n")
        start_when_ready(command, data, 4242001)
        wait_for_binary(command, data, 1024, 12)

        subprocess.run([
            "nrfutil", "--log-output", "stdout", "--log-level", "error",
            "device", "reset", "--reset-kind", "RESET_SYSTEM",
            "--serial-number", args.central_serial,
        ], check=True)
        read_until(command, data, "MSENSE_CENTRAL_READY", 20)
        connect_named(command, data, args.peer_name)
        if args.debugger_pause:
            input("PAUSED_BEFORE_RECOVERY_START\n")
        output, _ = start_when_ready(command, data, 4242010)
        if "START_ACK" not in output:
            raise RuntimeError("reconnected stream did not acknowledge START")
        wait_for_binary(command, data, 1024, 12)

    print("PASS: NUS data resumed after Central reset without Peripheral reset")


if __name__ == "__main__":
    main()
