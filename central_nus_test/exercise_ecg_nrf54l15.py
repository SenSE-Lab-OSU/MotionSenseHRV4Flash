import pathlib
import re
import sys
import time

import serial


COMMAND_PORT = "COM23"
RELAY_PORT = "COM22"
CAPTURE_PATH = pathlib.Path(__file__).parent / "captures" / "ecg_nrf54l15_live.mrly"


def read_text(port):
    return port.read(port.in_waiting or 1).decode("utf-8", errors="replace")


def send(port, command):
    print(f"COMMAND_SEND port={port.port} command={command}", flush=True)
    port.write((command + "\r\n").encode())
    port.flush()


def collect(port, seconds, stop_pattern=None):
    deadline = time.monotonic() + seconds
    output = ""
    while time.monotonic() < deadline:
        chunk = read_text(port)
        if chunk:
            output += chunk
            print(chunk, end="", flush=True)
            if stop_pattern and re.search(stop_pattern, output):
                break
        time.sleep(0.03)
    return output


def relay_frames(data):
    frames = 0
    payload_bytes = 0
    offset = 0
    while offset + 12 <= len(data):
        marker = data.find(b"MRLY", offset)
        if marker < 0 or marker + 12 > len(data):
            break
        length = int.from_bytes(data[marker + 6:marker + 8], "little")
        end = marker + 12 + length
        if length > 512 or end > len(data):
            break
        frames += 1
        payload_bytes += length
        offset = end
    return frames, payload_bytes


def main():
    relay_data = bytearray()
    CAPTURE_PATH.parent.mkdir(exist_ok=True)
    with serial.Serial(COMMAND_PORT, 115200, timeout=0.05, write_timeout=1,
                       rtscts=False, dsrdtr=False) as command, \
         serial.Serial(RELAY_PORT, 1_000_000, timeout=0.02, write_timeout=1,
                       rtscts=False, dsrdtr=False) as relay:
        command.dtr = True
        relay.dtr = True
        command.reset_input_buffer()
        relay.reset_input_buffer()

        send(command, "help")
        if "COMMANDS:" not in collect(command, 2, r"COMMANDS:"):
            print("TEST_FAIL command channel did not identify itself")
            return 2

        send(command, "status")
        status = collect(command, 2, r"STATUS[^\r\n]*")
        if not (re.search(r"state=(READY|COMPLETE)", status) and
                re.search(r"peer_name=MSense4ECG-", status)):
            if not re.search(r"state=IDLE", status):
                send(command, "disconnect")
                collect(command, 5, r"DISCONNECTED|ERR")
            send(command, "connect ecg")
            connection = collect(command, 40, r"NUS_READY|ERROR|DISCONNECTED")
            if "NUS_READY" not in connection:
                print("TEST_FAIL ECG did not reach NUS_READY")
                return 3

        send(command, "start")
        deadline = time.monotonic() + 90
        response = ""
        terminal_at = None
        while time.monotonic() < deadline:
            chunk = read_text(command)
            if chunk:
                response += chunk
                print(chunk, end="", flush=True)
                if re.search(r"STREAM_OK|STREAM_END|START_RESULT|PROTOCOL_ERROR", response):
                    terminal_at = terminal_at or time.monotonic()
            if relay.in_waiting:
                relay_data.extend(relay.read(relay.in_waiting))
            if terminal_at and time.monotonic() - terminal_at >= 4 and not relay.in_waiting:
                break
            time.sleep(0.01)

    CAPTURE_PATH.write_bytes(relay_data)
    frames, payload_bytes = relay_frames(relay_data)
    terminal = re.search(r"(STREAM_OK|STREAM_END|START_RESULT|PROTOCOL_ERROR)[^\r\n]*", response)
    print(f"RELAY_CAPTURE path={CAPTURE_PATH} bytes={len(relay_data)} "
          f"frames={frames} payload_bytes={payload_bytes}")
    print(f"TERMINAL_RESULT={terminal.group(0) if terminal else 'none'}")
    return 0 if terminal and terminal.group(1) == "STREAM_OK" else 4


if __name__ == "__main__":
    sys.exit(main())
