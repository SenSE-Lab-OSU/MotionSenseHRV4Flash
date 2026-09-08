import re
import sys
import time

import serial


COMMAND_PORT = "COM23"
RELAY_PORT = "COM22"
COMMAND_BAUD = 115200
RELAY_BAUD = 1_000_000


def drain_text(port):
    data = port.read(port.in_waiting or 1)
    return data.decode("utf-8", errors="replace")


def send(port, command):
    print(f"COMMAND_SEND port={port.port} command={command}", flush=True)
    port.write((command + "\r\n").encode())
    port.flush()


def collect(port, seconds, stop_pattern=None):
    deadline = time.monotonic() + seconds
    text = ""
    while time.monotonic() < deadline:
        chunk = drain_text(port)
        if chunk:
            text += chunk
            print(chunk, end="", flush=True)
            if stop_pattern and re.search(stop_pattern, text):
                break
        time.sleep(0.03)
    return text


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
    with serial.Serial(COMMAND_PORT, COMMAND_BAUD, timeout=0.05, write_timeout=1,
                       rtscts=False, dsrdtr=False) as command, \
         serial.Serial(RELAY_PORT, RELAY_BAUD, timeout=0.02, write_timeout=1,
                       rtscts=True, dsrdtr=False) as relay:
        command.dtr = True
        relay.dtr = True
        command.reset_input_buffer()
        relay.reset_input_buffer()

        send(command, "help")
        help_text = collect(command, 2, r"COMMANDS:")
        if "COMMANDS:" not in help_text:
            print("TEST_FAIL command channel did not identify itself")
            return 2

        send(command, "status")
        status = collect(command, 2, r"STATUS[^\r\n]*")
        if not re.search(r"state=(READY|COMPLETE)", status):
            send(command, "connect ppg")
            connection = collect(command, 35, r"NUS_READY|ERROR|DISCONNECTED")
            if "NUS_READY" not in connection:
                print("TEST_FAIL PPG did not reach NUS_READY")
                return 3

        send(command, "start")
        deadline = time.monotonic() + 15
        response = ""
        terminal_at = None
        while time.monotonic() < deadline:
            chunk = drain_text(command)
            if chunk:
                response += chunk
                print(chunk, end="", flush=True)
                if re.search(r"START_RESULT[^\r\n]*|STREAM_OK[^\r\n]*|PROTOCOL_ERROR[^\r\n]*", response):
                    terminal_at = terminal_at or time.monotonic()
            waiting = relay.in_waiting
            if waiting:
                relay_data.extend(relay.read(waiting))
            if terminal_at and time.monotonic() - terminal_at >= 2:
                break
            time.sleep(0.02)

        frames, payload_bytes = relay_frames(relay_data)
        result = re.search(r"START_RESULT[^\r\n]*", response)
        stream = re.search(r"STREAM_OK[^\r\n]*", response)
        print(f"RELAY_CAPTURE bytes={len(relay_data)} frames={frames} payload_bytes={payload_bytes}")
        if result:
            print(f"TEST_PASS received_{result.group(0)}")
            return 0
        if stream:
            print(f"TEST_PASS received_{stream.group(0)}")
            return 0
        print("TEST_FAIL no START_RESULT or STREAM_OK received")
        return 4


if __name__ == "__main__":
    sys.exit(main())
