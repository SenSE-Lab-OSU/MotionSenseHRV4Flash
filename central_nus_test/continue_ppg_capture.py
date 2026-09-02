import pathlib
import re
import time

import serial


COMMAND_PORT = "COM23"
RELAY_PORT = "COM22"
CAPTURE_PATH = pathlib.Path(__file__).parent / "captures" / "ppg_continued.mrly"


def count_frames(data):
    count = 0
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
        count += 1
        payload_bytes += length
        offset = end
    return count, payload_bytes


def main():
    relay_data = bytearray()
    command_text = ""
    CAPTURE_PATH.parent.mkdir(exist_ok=True)
    with serial.Serial(COMMAND_PORT, 115200, timeout=0.02, write_timeout=1,
                       rtscts=False, dsrdtr=False) as command, \
         serial.Serial(RELAY_PORT, 115200, timeout=0.02, write_timeout=1,
                       rtscts=False, dsrdtr=False) as relay:
        command.dtr = True
        relay.dtr = True
        command.reset_input_buffer()
        relay.reset_input_buffer()
        command.write(b"status\r\n")
        command.flush()
        deadline = time.monotonic() + 60
        disconnected_at = None
        while time.monotonic() < deadline:
            if command.in_waiting:
                chunk = command.read(command.in_waiting).decode(errors="replace")
                command_text += chunk
                print(chunk, end="", flush=True)
                if re.search(r"DISCONNECTED|PROTOCOL_ERROR", command_text):
                    disconnected_at = disconnected_at or time.monotonic()
            if relay.in_waiting:
                relay_data.extend(relay.read(relay.in_waiting))
            if disconnected_at and time.monotonic() - disconnected_at >= 2:
                break
            time.sleep(0.01)
    CAPTURE_PATH.write_bytes(relay_data)
    frames, payload_bytes = count_frames(relay_data)
    print(f"CAPTURE path={CAPTURE_PATH} bytes={len(relay_data)} "
          f"frames={frames} payload_bytes={payload_bytes}")


if __name__ == "__main__":
    main()
