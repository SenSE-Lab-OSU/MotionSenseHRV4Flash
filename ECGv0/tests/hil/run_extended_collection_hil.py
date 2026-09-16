#!/usr/bin/env python3
"""Run a destructive 3-4 hour ECG collection/stream/recovery HIL campaign."""

from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

import run_cleanup_hil as cleanup


CONFIRMATION = "I_FLASHED_PRISTINE_ECG_AND_FORMATTED_FATFS_0X44"
FINITE_BYTES = 131072
COLLECTION_START_TIMEOUT = 30
MRLY_HEADER = 12
NUS_HEADER = 12
RELAY_RX_BUFFER = 1024 * 1024
PEER = re.compile(
    r"PEER_READY nus=1 smp=[01] legacy=[01] peer_name=(\S+) "
    r"peer_addr=([0-9A-Fa-f:]+) peer_addr_type=(\d+)"
)
STATUS_PEER = re.compile(
    r"STATUS state=(READY|COMPLETE)[^\r\n]*peer_name=(\S+) "
    r"peer_addr=([0-9A-Fa-f:]+) peer_addr_type=(\d+)"
)
CENTRAL_FAILURE = re.compile(
    r"PROTOCOL_ERROR|\bBUFFER_OVERFLOW\b|\bSTORAGE_ERROR\b|"
    r"ERROR relay|ERROR NUS|relay (?:queue|slab) .*overflow|(?:^|[\r\n])(?:ERROR|ERR)\b",
    re.IGNORECASE,
)
NATIVE_FAILURE = re.compile(
    r"assert(?:ion)? failed|fatal error|kernel panic|watchdog[^\r\n]*(?:reset|timeout|expired)|"
    r"hard[ _-]?fault|"
    r"stack overflow|(?:fifo|buffer) overflow|"
    r"dropped \d+ samples|"
    r"(?:ecg|accel|recorder)[^\r\n]*(?:failed|failure|fatal|corrupt|\bEIO\b)",
    re.IGNORECASE,
)
NATIVE_REVIEW = re.compile(
    r"<wrn>|\bwarning\b|slow write|bad[ _-]?block|bq274|battery gauge",
    re.IGNORECASE,
)
ECG_FILE = re.compile(r"^(?:\d+)?ecg\d+_(\d{4})\.bin$", re.IGNORECASE)
ACCEL_FILE = re.compile(r"^(?:\d+)?ac(\d+)_(\d{4})\.bin$", re.IGNORECASE)
LINE_END = r"(?:\r\n|\r|\n)"
LINE_START = r"(?:^|[\r\n])"
COMMANDS_LINE = rf"{LINE_START}COMMANDS:[^\r\n]*{LINE_END}"
PEER_READY_LINE = rf"{LINE_START}PEER_READY[^\r\n]*{LINE_END}"
STATUS_LINE = rf"{LINE_START}STATUS state=[^\r\n]*{LINE_END}"
DISCONNECTED_LINE = rf"{LINE_START}DISCONNECTED[^\r\n]*{LINE_END}"
RELAY_IDLE_LINE = rf"{LINE_START}RELAY_IDLE{LINE_END}"
STREAM_DISCONNECTED_LINE = (
    rf"{LINE_START}STREAM_END status=DISCONNECTED[^\r\n]*{LINE_END}")
ECG_PREFIX_LINE = rf"{LINE_START}ECG_PREFIX[^\r\n]*{LINE_END}"
FINITE_PREFIX = re.compile(
    r"ECG_PREFIX id=(\d+) valid_blocks=(\d+) valid_bytes=(\d+) "
    r"skipped_history_blocks=(\d+)"
)
REMOTE_STATUS_LINE = rf"{LINE_START}REMOTE_STATUS status=[^\r\n]*{LINE_END}"
REMOTE_STATUS_SUCCESS = re.compile(
    rf"{LINE_START}REMOTE_STATUS status=success length=(\d+) "
    rf"hex=([0-9a-fA-F]+){LINE_END}"
)
COLLECTION_STARTED = re.compile(
    r"ecg_recorder: ECG NAND recording active, first sample RTC tick=\d+"
)
EXPECTED_STREAM_DISCONNECT = (
    DISCONNECTED_LINE, RELAY_IDLE_LINE, STREAM_DISCONNECTED_LINE)


def central_help_capable(text: str) -> bool:
    connect = re.search(r"\bconnect\s+([a-z|]+)", text, re.IGNORECASE)
    return bool(connect and "ecg" in connect.group(1).lower().split("|") and
                re.search(r"\bstart\b", text, re.IGNORECASE) and
                re.search(r"\bdisconnect\b", text, re.IGNORECASE))


class CentralWaitTimeout(cleanup.HilError):
    pass


def wait_for_central_capabilities(send, wait, deadline: float, retry: bool,
                                  clock=time.monotonic) -> str:
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            raise cleanup.HilError("Central did not return valid capabilities before timeout")
        send("help")
        try:
            response = wait(COMMANDS_LINE, min(5, remaining))
        except CentralWaitTimeout:
            if retry:
                continue
            raise
        if not central_help_capable(response):
            raise cleanup.HilError("command port did not identify the required Central firmware")
        return response


def wait_for_collection_ready(send, wait, deadline: float, clock=time.monotonic,
                              sleeper=time.sleep) -> dict[str, object]:
    polls = 0
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            raise cleanup.HilError("ECG collection did not become ready before timeout")
        send("remote status")
        polls += 1
        try:
            response = wait(REMOTE_STATUS_LINE, min(2, remaining))
        except CentralWaitTimeout:
            continue
        match = REMOTE_STATUS_SUCCESS.search(response)
        if not match:
            raise cleanup.HilError("ECG remote status read failed")
        length, status_hex = int(match.group(1)), match.group(2)
        if length != 8 or len(status_hex) != 16:
            raise cleanup.HilError("ECG remote status has invalid length")
        status = bytes.fromhex(status_hex)
        if status[:3] == b"\x01\x01\x01":
            return {"status_hex": status_hex.lower(), "polls": polls}
        sleeper(min(0.1, max(0, deadline - clock())))


def native_collection_start(text: str) -> str | None:
    for line in text.splitlines(keepends=True):
        if line.endswith(("\n", "\r")) and COLLECTION_STARTED.search(line):
            return line.rstrip("\r\n")
    return None


def validate_finite_prefix(text: str, session_id: int) -> dict[str, int]:
    lines = [line for line in text.splitlines(keepends=True)
             if line.startswith("ECG_PREFIX")]
    if not lines:
        raise cleanup.HilError("finite stream lacks a complete ECG_PREFIX line")
    if not lines[-1].endswith(("\r", "\n")):
        raise cleanup.HilError("finite stream has a partial ECG_PREFIX line")
    match = FINITE_PREFIX.fullmatch(lines[-1].rstrip("\r\n"))
    if not match:
        raise cleanup.HilError("finite stream has a malformed ECG_PREFIX line")
    prefix_id, valid_blocks, valid_bytes, skipped = map(int, match.groups())
    if prefix_id != session_id:
        raise cleanup.HilError("finite ECG_PREFIX session ID mismatch")
    if skipped > 8:
        raise cleanup.HilError("finite ECG_PREFIX skipped excessive history")
    if valid_blocks + skipped != 32:
        raise cleanup.HilError("finite ECG_PREFIX does not account for 32 blocks")
    if valid_bytes != valid_blocks * 4096:
        raise cleanup.HilError("finite ECG_PREFIX byte count is inconsistent")
    return {"valid_blocks": valid_blocks, "valid_bytes": valid_bytes,
            "skipped_history_blocks": skipped}


def finite_stream_ok_line(session_id: int) -> str:
    return (rf"{LINE_START}STREAM_OK id={session_id} bytes={FINITE_BYTES}"
            rf"(?: [^\r\n]*)?{LINE_END}")


class Central:
    def __init__(self, args, output: Path):
        self.args = args
        self.output = output
        self.command_log = (output / "central-command.txt").open("a", encoding="utf-8", buffering=1)
        self.command_raw = (output / "central-command.bin").open("ab", buffering=0)
        self.command = self.relay = None
        self.text = ""
        self.identity: tuple[str, str, int] | None = None
        self.stream_number = 0
        self.open(restarting=False)

    def open(self, restarting: bool) -> None:
        deadline = time.monotonic() + self.args.central_reopen_timeout
        last_error = "ports unavailable"
        while time.monotonic() < deadline:
            try:
                self.command = serial.Serial(self.args.command_port, 115200, timeout=0.01,
                                             write_timeout=2, rtscts=False, dsrdtr=False)
                self.relay = serial.Serial(self.args.relay_port, 1_000_000, timeout=0.02,
                                           write_timeout=2, rtscts=self.args.relay_rtscts,
                                           dsrdtr=False)
                self.relay.set_buffer_size(rx_size=RELAY_RX_BUFFER)
                self.command.dtr = self.relay.dtr = True
                break
            except (OSError, serial.SerialException) as error:
                last_error = str(error)
                self.close_ports()
                time.sleep(1)
        else:
            raise cleanup.HilError(f"Central ports did not open: {last_error}")
        help_deadline = deadline if restarting else time.monotonic() + 5
        wait_for_central_capabilities(self.send, self.wait, help_deadline, restarting)

    def close_ports(self) -> None:
        for port in (self.relay, self.command):
            if port is not None:
                try:
                    port.close()
                except (OSError, serial.SerialException):
                    pass
        self.command = self.relay = None

    def close(self) -> None:
        self.close_ports()
        self.command_log.close()
        self.command_raw.close()

    def read_command(self) -> str:
        try:
            data = self.command.read(self.command.in_waiting or 1)
        except (OSError, serial.SerialException) as error:
            raise cleanup.HilError(f"Central command port failed: {error}") from error
        if not data:
            return ""
        self.command_raw.write(data)
        text = data.decode("utf-8", errors="replace")
        self.command_log.write(text)
        self.text += text
        return text

    def read_relay(self) -> bytes:
        try:
            return self.relay.read(self.relay.in_waiting or 1)
        except (OSError, serial.SerialException) as error:
            raise cleanup.HilError(f"Central relay port failed: {error}") from error

    def send(self, line: str) -> None:
        self.command_log.write(f"\n[{datetime.now(timezone.utc).isoformat()}] COMMAND {line}\n")
        try:
            self.command.write((line + "\r\n").encode("ascii"))
            self.command.flush()
        except (OSError, serial.SerialException) as error:
            raise cleanup.HilError(f"Central command write failed: {error}") from error

    def wait(self, pattern: str | tuple[str, ...], timeout: float,
             relay: bytearray | None = None,
             allow_disconnect: bool = False) -> str:
        patterns = (pattern,) if isinstance(pattern, str) else pattern
        start = len(self.text)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if relay is not None and self.relay.in_waiting:
                relay.extend(self.read_relay())
            self.read_command()
            if relay is not None and self.relay.in_waiting:
                relay.extend(self.read_relay())
            response = self.text[start:]
            failure = CENTRAL_FAILURE.search(response)
            if failure:
                raise cleanup.HilError(f"Central reported {failure.group(0)}")
            stream_end = re.search(
                rf"STREAM_END status=([A-Z_]+)[^\r\n]*{LINE_END}", response)
            if stream_end and (not allow_disconnect or stream_end.group(1) != "DISCONNECTED"):
                raise cleanup.HilError(f"unexpected stream end: {stream_end.group(1)}")
            if not allow_disconnect and "stream_aborted" in response:
                raise cleanup.HilError("unexpected BLE disconnect during stream")
            if all(re.search(required, response) for required in patterns):
                return response
            time.sleep(0.005)
        raise CentralWaitTimeout(
            f"timeout waiting for {patterns!r}; tail={self.text[-1000:]!r}")

    @staticmethod
    def identity_from(text: str) -> tuple[str, str, int]:
        matches = PEER.findall(text)
        if len(matches) != 1:
            raise cleanup.HilError(f"ambiguous or absent PEER_READY identity ({len(matches)} matches)")
        name, address, address_type = matches[0]
        return name, address.upper(), int(address_type)

    def connect(self) -> dict[str, object]:
        self.send("connect ecg")
        response = self.wait(rf"(?:{PEER_READY_LINE}|{DISCONNECTED_LINE})",
                             self.args.connect_timeout)
        identity = self.identity_from(response)
        if identity[0] != self.args.peer_name:
            raise cleanup.HilError(f"Central selected unexpected peer {identity[0]!r}")
        if self.identity is not None and identity != self.identity:
            raise cleanup.HilError(f"peer identity changed from {self.identity} to {identity}")
        self.identity = identity
        return {"name": identity[0], "address": identity[1], "address_type": identity[2]}

    def initial_peer(self) -> dict[str, object]:
        self.send("status")
        response = self.wait(STATUS_LINE, 5)
        if "STATUS state=IDLE" in response:
            return self.connect()
        matches = STATUS_PEER.findall(response)
        if len(matches) != 1:
            raise cleanup.HilError("Central startup state or connected peer is ambiguous")
        _, name, address, address_type = matches[0]
        identity = name, address.upper(), int(address_type)
        if identity[0] != self.args.peer_name:
            raise cleanup.HilError(f"Central is connected to unexpected peer {identity[0]!r}")
        self.identity = identity
        return {"name": identity[0], "address": identity[1], "address_type": identity[2]}

    def disconnect(self, expected_stream: bool = False, relay: bytearray | None = None) -> str:
        self.send("disconnect")
        required = EXPECTED_STREAM_DISCONNECT if expected_stream else DISCONNECTED_LINE
        response = self.wait(required, 15, relay, allow_disconnect=expected_stream)
        if expected_stream:
            if "stream_aborted" not in response and "stream_aborted" not in self.text[-2000:]:
                raise cleanup.HilError("intentional disconnect did not abort the active stream")
        return response

    def _capture(self, session_id: int, infinity: bool, terminal: bool,
                 allow_trailing: bool = False) -> dict[str, object]:
        self.stream_number += 1
        raw = bytearray()
        prefix = None
        self.relay.reset_input_buffer()
        self.send(f"start {'infinity ' if infinity else ''}{session_id}")
        if terminal:
            response = self.wait((
                ECG_PREFIX_LINE,
                finite_stream_ok_line(session_id),
                RELAY_IDLE_LINE,
            ), self.args.stream_timeout, raw)
            prefix = validate_finite_prefix(response, session_id)
        else:
            self.wait((
                rf"START_ACK type=ECG mode=INFINITY[^\r\n]*{LINE_END}",
                rf"ECG_PREFIX id={session_id} valid_blocks=(?:9|[1-9]\d+)"
                rf"[^\r\n]*{LINE_END}",
            ), self.args.stream_timeout, raw)
        path = self.output / f"stream-{self.stream_number:02d}-{session_id}.mrly"
        path.write_bytes(raw)
        parsed = validate_mrly(bytes(raw), session_id, infinity, terminal,
                               allow_trailing=allow_trailing)
        parsed.update({"file": str(path), "sha256": cleanup.sha256(path)})
        if prefix is not None:
            parsed["ecg_prefix"] = prefix
        return parsed

    def finite(self, session_id: int) -> dict[str, object]:
        return self._capture(session_id, False, True)

    def disconnect_dropout(self, session_id: int) -> dict[str, object]:
        capture = self._capture(session_id, True, False)
        path = Path(capture["file"])
        raw = bytearray(path.read_bytes())
        self.disconnect(expected_stream=True, relay=raw)
        path.write_bytes(raw)
        capture = validate_mrly(bytes(raw), session_id, True, False)
        capture.update({"file": str(path), "sha256": cleanup.sha256(path),
                        "expected_end": "DISCONNECTED"})
        time.sleep(self.args.dropout_seconds)
        capture["reconnected_peer"] = self.connect()
        return capture

    def central_restart(self, session_id: int, reset_index: int) -> dict[str, object]:
        capture = self._capture(session_id, True, False, allow_trailing=True)
        self.close_ports()
        command = [str(self.args.nrfutil), "--log-output", "stdout", "--log-level", "error",
                   "device", "reset", "--reset-kind", "RESET_SYSTEM",
                   "--serial-number", self.args.central_jlink_serial]
        completed = subprocess.run(command, text=True, capture_output=True, timeout=60)
        prefix = self.output / f"central-reset-{reset_index}"
        prefix.with_suffix(".stdout.txt").write_text(completed.stdout, encoding="utf-8")
        prefix.with_suffix(".stderr.txt").write_text(completed.stderr, encoding="utf-8")
        if completed.returncode != 0:
            raise cleanup.HilError(f"Central RESET_SYSTEM failed with {completed.returncode}")
        self.open(restarting=True)
        capture["central_reset"] = {"argv": command, "exit_code": completed.returncode}
        capture["reconnected_peer"] = self.connect()
        return capture


def validate_mrly(raw: bytes, session_id: int, infinity: bool, terminal_required: bool,
                  allow_trailing: bool = False) -> dict[str, object]:
    cursor = 0
    relay_sequence = None
    messages = data_messages = sensor_bytes = 0
    expected_offset = 0
    ack = end = False
    while cursor < len(raw):
        if len(raw) - cursor < MRLY_HEADER:
            if allow_trailing:
                break
            raise cleanup.HilError("truncated MRLY header")
        if raw[cursor:cursor + 4] != b"MRLY" or raw[cursor + 4:cursor + 6] != b"\x01\x01":
            next_magic = raw.find(b"MRLY", cursor + 1)
            detail = ""
            if next_magic >= 0 and next_magic + MRLY_HEADER <= len(raw):
                next_sequence = struct.unpack_from("<I", raw, next_magic + 8)[0]
                missing = ((next_sequence - relay_sequence - 1) & 0xffff_ffff
                           if relay_sequence is not None else "unknown")
                detail = (f"; {next_magic - cursor} unframed bytes before relay sequence "
                          f"{next_sequence} ({missing} missing frame(s))")
            raise cleanup.HilError(f"invalid MRLY header at byte {cursor}{detail}")
        length, sequence = struct.unpack_from("<HI", raw, cursor + 6)
        end_cursor = cursor + MRLY_HEADER + length
        if end_cursor > len(raw):
            if allow_trailing:
                break
            raise cleanup.HilError("truncated MRLY payload")
        if relay_sequence is not None and sequence != (relay_sequence + 1) & 0xffff_ffff:
            raise cleanup.HilError("MRLY relay sequence gap")
        relay_sequence = sequence
        nus = raw[cursor + MRLY_HEADER:end_cursor]
        if len(nus) < NUS_HEADER or nus[:3] != b"MS\x00":
            raise cleanup.HilError("invalid NUS v0 header")
        message_type = nus[3]
        message_session, payload_length, flags = struct.unpack_from("<IHH", nus, 4)
        if message_session != session_id or flags != 0 or len(nus) != NUS_HEADER + payload_length:
            raise cleanup.HilError("invalid NUS session, flags, or payload length")
        payload = nus[NUS_HEADER:]
        if end:
            raise cleanup.HilError("NUS message followed END")
        if message_type == 0x81:
            if ack or len(payload) != 16 or payload[0] != int(infinity) or payload[1] != 8 or \
                    payload[2:8] != bytes(6) or struct.unpack_from("<Q", payload, 8)[0] != \
                    (0 if infinity else FINITE_BYTES):
                raise cleanup.HilError("invalid START_ACK")
            ack = True
        elif message_type == 0x82:
            if not ack or len(payload) < 8:
                raise cleanup.HilError("DATA before ACK or without offset")
            offset = struct.unpack_from("<Q", payload)[0]
            if offset != expected_offset:
                raise cleanup.HilError(f"NUS data offset gap: {offset} != {expected_offset}")
            expected_offset += len(payload) - 8
            sensor_bytes += len(payload) - 8
            data_messages += 1
        elif message_type == 0x83:
            if len(payload) != 2 or struct.unpack_from("<H", payload)[0] != 0:
                raise cleanup.HilError("finite stream ended with non-success status")
            end = True
        else:
            raise cleanup.HilError(f"unexpected NUS message type 0x{message_type:02x}")
        messages += 1
        cursor = end_cursor
    if not ack or not data_messages:
        raise cleanup.HilError("stream relay lacks ACK or DATA")
    if terminal_required and (not end or sensor_bytes != FINITE_BYTES or cursor != len(raw)):
        raise cleanup.HilError("finite relay is incomplete")
    return {"validation": "PASS", "messages": messages, "data_messages": data_messages,
            "sensor_bytes": sensor_bytes, "terminal": end,
            "trailing_bytes": len(raw) - cursor}


def scan_native(path: Path) -> dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="replace")
    storage = cleanup.scan_storage_errors(text)
    failures = [line for line in text.splitlines() if NATIVE_FAILURE.search(line)]
    if failures:
        raise cleanup.HilError("native UART reported actionable faults: " + " | ".join(failures))
    review = [line for line in text.splitlines() if NATIVE_REVIEW.search(line)]
    return {"result": "PASS", "storage": storage, "actionable_faults": [],
            "review_lines": review}


def watch_native(path: Path, deadline: float) -> None:
    """Wait until a deadline while failing promptly on newly captured native faults."""
    offset = 0
    pending = ""
    while time.monotonic() < deadline:
        if path.is_file():
            with path.open("rb") as source:
                source.seek(offset)
                data = source.read()
                offset += len(data)
            if data:
                text = pending + data.decode("utf-8", errors="replace")
                lines = text.splitlines(keepends=True)
                pending = "" if lines and lines[-1].endswith(("\n", "\r")) else lines.pop()
                complete = "".join(lines)
                cleanup.scan_storage_errors(complete)
                failure = NATIVE_FAILURE.search(complete)
                if failure:
                    raise cleanup.HilError(f"native UART reported {failure.group(0)}")
        time.sleep(min(2, max(0, deadline - time.monotonic())))


def _ecg_edges(path: Path) -> tuple[int, int, tuple[int, int], tuple[int, int]]:
    raw = path.read_bytes()
    recording_id = struct.unpack_from("<Q", raw, 8)[0]
    chunk = struct.unpack_from("<I", raw, 4)[0]
    edges = []
    for offset in range(4096, len(raw), 4096):
        block = raw[offset:offset + 4096]
        if block == b"\xff" * len(block):
            break
        edges.append(struct.unpack_from("<II", block, 4))
    if not edges:
        raise cleanup.HilError(f"no ECG blocks in {path.name}")
    return recording_id, chunk, edges[0], edges[-1]


def _accel_edges(path: Path) -> tuple[str, int, int, int]:
    match = ACCEL_FILE.fullmatch(path.name)
    if not match:
        raise cleanup.HilError(f"ambiguous accelerometer filename {path.name}")
    raw = path.read_bytes()
    valid_bytes = struct.unpack_from("<I", raw, 4 * 1024 * 1024 - 4096 + 4)[0]
    logical = raw[4096:4096 + valid_bytes]
    cursor = 0
    first = last_end = None
    while cursor < len(logical):
        length = min(4096, len(logical) - cursor)
        if length < 22 or (length - 16) % 6:
            raise cleanup.HilError(f"invalid ACB1 length in {path.name}")
        count = (length - 16) // 6
        sequence = struct.unpack_from("<I", logical, cursor + 8)[0]
        if last_end is not None and sequence != last_end:
            raise cleanup.HilError(f"accelerometer sequence gap in {path.name}")
        first = sequence if first is None else first
        last_end = (sequence + count) & 0xffff_ffff
        cursor += length
    if first is None:
        raise cleanup.HilError(f"no accelerometer blocks in {path.name}")
    return match.group(1), int(match.group(2)), first, last_end


def validate_rollovers(paths: list[Path]) -> dict[str, object]:
    ecg = sorted((_ecg_edges(path) for path in paths if ECG_FILE.fullmatch(path.name)),
                 key=lambda item: (item[0], item[1]))
    for previous, current in zip(ecg, ecg[1:]):
        if current[0] == previous[0]:
            if current[1] != previous[1] + 1:
                raise cleanup.HilError("ECG chunk-index gap")
            expected = tuple((value + 1358) & 0xffff_ffff for value in previous[3])
            if current[2] != expected:
                raise cleanup.HilError("ECG tick/index gap across rollover")
    accel = sorted((_accel_edges(path) for path in paths if ACCEL_FILE.fullmatch(path.name)),
                   key=lambda item: (item[0], item[1]))
    for previous, current in zip(accel, accel[1:]):
        if current[0] == previous[0]:
            if current[1] != previous[1] + 1 or current[2] != previous[3]:
                raise cleanup.HilError("accelerometer sequence/chunk gap across rollover")
    return {"result": "PASS", "ecg_files": len(ecg), "accel_files": len(accel)}


def evidence_manifest(output: Path) -> list[dict[str, object]]:
    return [{"file": path.relative_to(output).as_posix(), "bytes": path.stat().st_size,
             "mtime_ns": path.stat().st_mtime_ns, "sha256": cleanup.sha256(path)}
            for path in sorted(output.rglob("*"))
            if path.is_file() and path.name not in ("summary.json", "manifest.json")]


def self_test() -> None:
    actual_help = ("COMMANDS: help scan connect ppg|ecg|any collect on|off status "
                   "start [infinity] [id] stop [id] disconnect")
    assert not re.search(COMMANDS_LINE, actual_help)
    assert re.search(COMMANDS_LINE, actual_help + "\r\n")
    assert central_help_capable(actual_help)
    assert not central_help_capable(actual_help.replace("ppg|ecg|any", "ppg|any"))
    sent = []
    complete_help = actual_help + "\r\n"

    def send_help(command):
        sent.append(command)

    def help_without_banner(_pattern, _timeout):
        return complete_help

    assert wait_for_central_capabilities(
        send_help, help_without_banner, 1, True, clock=lambda: 0) == complete_help
    assert sent == ["help"]
    attempts = 0

    def lost_then_ready(_pattern, _timeout):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise CentralWaitTimeout("lost startup write")
        return complete_help

    sent.clear()
    assert wait_for_central_capabilities(
        send_help, lost_then_ready, 1, True, clock=lambda: 0) == complete_help
    assert sent == ["help", "help"]
    try:
        wait_for_central_capabilities(
            send_help,
            lambda _pattern, _timeout: complete_help.replace(" disconnect", ""),
            1, True, clock=lambda: 0)
    except cleanup.HilError:
        pass
    else:
        raise AssertionError("Central help without disconnect was accepted")
    before_serial_failure = len(sent)

    def serial_failure(_pattern, _timeout):
        raise cleanup.HilError("serial read failed")

    try:
        wait_for_central_capabilities(
            send_help, serial_failure, 1, True, clock=lambda: 0)
    except cleanup.HilError as error:
        assert str(error) == "serial read failed" and len(sent) == before_serial_failure + 1
    else:
        raise AssertionError("Central serial failure was retried")
    now = [0.0]

    def no_help(_pattern, timeout):
        now[0] += timeout
        raise CentralWaitTimeout("no response")

    try:
        wait_for_central_capabilities(
            send_help, no_help, 6, True, clock=lambda: now[0])
    except cleanup.HilError as error:
        assert "before timeout" in str(error)
    else:
        raise AssertionError("Central capability timeout was accepted")
    ready_line = (
        "REMOTE_STATUS status=success length=8 hex=0101010000000000\r\n")
    assert not re.search(REMOTE_STATUS_LINE, ready_line.rstrip("\r\n"))
    assert re.search(REMOTE_STATUS_LINE, ready_line)
    remote_responses = iter((
        "REMOTE_STATUS status=success length=8 hex=0100000000000000\r\n",
        ready_line,
    ))
    now = [0.0]
    remote_commands = []
    ready = wait_for_collection_ready(
        remote_commands.append, lambda _pattern, _timeout: next(remote_responses), 1,
        clock=lambda: now[0], sleeper=lambda delay: now.__setitem__(0, now[0] + delay))
    assert ready == {"status_hex": "0101010000000000", "polls": 2}
    assert remote_commands == ["remote status", "remote status"]
    now = [0.0]

    def partial_remote(_pattern, timeout):
        now[0] += timeout
        raise CentralWaitTimeout("partial REMOTE_STATUS line")

    try:
        wait_for_collection_ready(
            lambda _command: None, partial_remote, 0.25, clock=lambda: now[0],
            sleeper=lambda _delay: None)
    except cleanup.HilError as error:
        assert "before timeout" in str(error)
    else:
        raise AssertionError("partial remote status line was accepted")
    native_start = (
        "[00:05:26.035,675] <inf> ecg_recorder: ECG NAND recording active, "
        "first sample RTC tick=166764\r\n")
    assert native_collection_start(native_start) == native_start.rstrip("\r\n")
    assert native_collection_start(native_start.rstrip("\r\n")) is None
    assert native_collection_start("Entering ECG collection mode\r\n") is None
    assert native_collection_start("Leaving ECG collection mode\r\n") is None
    def prefix(valid_blocks, skipped, valid_bytes=None, ending="\r\n"):
        valid_bytes = valid_blocks * 4096 if valid_bytes is None else valid_bytes
        return (f"ECG_PREFIX id=7 valid_blocks={valid_blocks} valid_bytes={valid_bytes} "
                f"skipped_history_blocks={skipped}{ending}")

    assert validate_finite_prefix(prefix(32, 0), 7) == {
        "valid_blocks": 32, "valid_bytes": FINITE_BYTES, "skipped_history_blocks": 0}
    assert validate_finite_prefix(prefix(24, 8), 7)["valid_bytes"] == 98304
    assert validate_finite_prefix(prefix(28, 4), 7)["skipped_history_blocks"] == 4
    assert validate_finite_prefix(prefix(23, 8) + prefix(24, 8), 7)["valid_blocks"] == 24
    for invalid in (prefix(23, 8), prefix(25, 8), prefix(23, 9), prefix(33, -1),
                    prefix(24, 8, 98305), prefix(24, 8, ending=""),
                    "ECG_PREFIX id=7 valid_blocks=24 valid_bytes=98304\r\n",
                    prefix(32, 0) + "ECG_PREFIX malformed\r\n",
                    prefix(32, 0) + prefix(24, 8, ending="")):
        try:
            validate_finite_prefix(invalid, 7)
        except cleanup.HilError:
            pass
        else:
            raise AssertionError(f"invalid finite ECG prefix was accepted: {invalid!r}")
    stream_ok = f"STREAM_OK id=7 bytes={FINITE_BYTES} data_messages=285"
    assert not re.search(finite_stream_ok_line(7), stream_ok)
    assert re.search(finite_stream_ok_line(7), stream_ok + "\r\n")
    status = ("STATUS state=READY subscribed=1 relay_dropped=0 "
              "peer_name=MSense4ECG-X peer_addr=AA:BB:CC:DD:EE:FF peer_addr_type=1")
    assert not re.search(STATUS_LINE, status)
    assert re.search(STATUS_LINE, status + "\r\n")
    peer = ("PEER_READY nus=1 smp=0 legacy=1 peer_name=MSense4ECG-X "
            "peer_addr=AA:BB:CC:DD:EE:FF peer_addr_type=1")
    assert not re.search(PEER_READY_LINE, peer)
    assert re.search(PEER_READY_LINE, peer + "\n")
    early_disconnect = ("DISCONNECTED reason=0x13 stream_aborted\r\n"
                        "RELAY_IDLE\r\n")
    assert not all(re.search(pattern, early_disconnect)
                   for pattern in EXPECTED_STREAM_DISCONNECT)
    complete_disconnect = early_disconnect + (
        "STREAM_END status=DISCONNECTED(0x000d) bytes=40960 data=90\r\n")
    assert all(re.search(pattern, complete_disconnect)
               for pattern in EXPECTED_STREAM_DISCONNECT)
    assert not re.search(STREAM_DISCONNECTED_LINE,
                         "PEER_READY nus=1 peer_name=MSense4ECG-X\r\n")
    ack = b"MS\x00\x81" + struct.pack("<IHH", 7, 16, 0) + \
        bytes((0, 8)) + bytes(6) + struct.pack("<Q", FINITE_BYTES)
    data = b"MS\x00\x82" + struct.pack("<IHHQ", 7, 12, 0, 0) + b"test"
    end = b"MS\x00\x83" + struct.pack("<IHHH", 7, 2, 0, 0)
    framed = b"".join(b"MRLY\x01\x01" + struct.pack("<HI", len(item), index) + item
                      for index, item in enumerate((ack, data, end)))
    try:
        validate_mrly(framed, 7, False, True)
    except cleanup.HilError:
        pass
    else:
        raise AssertionError("short finite stream was accepted")
    items = [ack]
    for offset in range(0, FINITE_BYTES, 32768):
        items.append(b"MS\x00\x82" + struct.pack("<IHHQ", 7, 32776, 0, offset) +
                     bytes(32768))
    items.append(end)
    framed = b"".join(b"MRLY\x01\x01" + struct.pack("<HI", len(item), index) + item
                      for index, item in enumerate(items))
    assert validate_mrly(framed, 7, False, True)["sensor_bytes"] == FINITE_BYTES
    broken = bytearray(framed)
    struct.pack_into("<I", broken, MRLY_HEADER + len(ack) + 8, 4)
    try:
        validate_mrly(bytes(broken), 7, False, True)
    except cleanup.HilError:
        pass
    else:
        raise AssertionError("MRLY sequence gap was accepted")
    small_data = [
        b"MS\x00\x82" + struct.pack("<IHHQ", 7, 483, 0, offset) + bytes(475)
        for offset in (0, 475, 950)]
    loss_frames = [ack, *small_data]
    loss_capture = b"".join(
        b"MRLY\x01\x01" + struct.pack("<HI", len(item), sequence) + item
        for sequence, item in enumerate(loss_frames, 100))
    damaged_frame = MRLY_HEADER + len(ack) + MRLY_HEADER + len(small_data[0])
    loss_capture = loss_capture[:damaged_frame] + loss_capture[damaged_frame + 322:]
    try:
        validate_mrly(loss_capture, 7, False, False)
    except cleanup.HilError as error:
        assert "185 unframed bytes" in str(error) and "1 missing frame(s)" in str(error)
    else:
        raise AssertionError("partially lost MRLY frame was accepted")
    assert Central.identity_from(
        "PEER_READY nus=1 smp=0 legacy=1 peer_name=MSense4ECG-X "
        "peer_addr=AA:BB:CC:DD:EE:FF peer_addr_type=1") == \
        ("MSense4ECG-X", "AA:BB:CC:DD:EE:FF", 1)
    with tempfile.TemporaryDirectory(prefix="ecg-extended-hil-") as directory:
        clean = Path(directory) / "uart.txt"
        clean.write_text("Storage log end\n", encoding="utf-8")
        assert scan_native(clean)["result"] == "PASS"
        clean.write_text("watchdog enabled\nslow write observed\n", encoding="utf-8")
        assert scan_native(clean)["review_lines"] == ["slow write observed"]
        for message in ("fatal error in recorder", "Accelerometer recording dropped 3 samples"):
            clean.write_text(message + "\n", encoding="utf-8")
            try:
                scan_native(clean)
            except cleanup.HilError:
                pass
            else:
                raise AssertionError("native actionable error was accepted")
    print("run_extended_collection_hil.py self-test: PASS (no hardware access)")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-test", help="run hardware-free parser/scanner tests")
    run = sub.add_parser("run", help="run the destructive extended campaign")
    for name in ("tools-dir", "elf", "drive", "output", "nrfutil"):
        run.add_argument(f"--{name}", type=Path, required=True)
    for name in ("probe", "port", "usb-serial", "command-port", "relay-port",
                 "central-jlink-serial", "peer-name"):
        run.add_argument(f"--{name}", required=True)
    run.add_argument("--destructive-confirmation", choices=(CONFIRMATION,), required=True)
    run.add_argument("--preflight-log", type=Path, nargs="+", required=True)
    wake = run.add_mutually_exclusive_group(required=True)
    wake.add_argument("--wake-first", action="store_true")
    wake.add_argument("--already-awake", action="store_true")
    run.add_argument("--nm", type=Path)
    run.add_argument("--allow-port-change", action="store_true")
    run.add_argument("--relay-rtscts", action="store_true")
    run.add_argument("--campaign-hours", type=float, default=3.5)
    run.add_argument("--mixed-minutes", type=float, default=30)
    run.add_argument("--cycle-minutes", type=float, default=10)
    run.add_argument("--cycle-count", type=int, default=3)
    run.add_argument("--post-capture-seconds", type=float, default=60)
    run.add_argument("--dropout-seconds", type=float, default=120)
    run.add_argument("--stream-timeout", type=float, default=240)
    run.add_argument("--connect-timeout", type=float, default=30)
    run.add_argument("--central-reopen-timeout", type=float, default=45)
    run.add_argument("--wake-delay", type=float, default=5)
    run.add_argument("--capture-start-delay", type=float, default=1)
    run.add_argument("--remount-timeout", type=float, default=60)
    run.add_argument("--settle-seconds", type=float, default=2)
    args = parser.parse_args(argv)
    if args.command == "run":
        if not 3 <= args.campaign_hours <= 4:
            parser.error("--campaign-hours must be between 3 and 4")
        durations = (args.mixed_minutes, args.cycle_minutes, args.post_capture_seconds,
                     args.dropout_seconds, args.stream_timeout, args.connect_timeout,
                     args.central_reopen_timeout, args.wake_delay, args.capture_start_delay,
                     args.remount_timeout, args.settle_seconds)
        if args.cycle_count < 2 or min(durations) <= 0:
            parser.error("durations must be positive and at least two cycles are required")
        sessions = 2 + args.cycle_count
        allocated = (args.mixed_minutes + args.cycle_count * args.cycle_minutes) * 60
        soak = args.campaign_hours * 3600 - allocated - sessions * args.post_capture_seconds
        if soak < 90 * 60:
            parser.error("campaign allocation leaves less than 90 minutes for collection-only soak")
        args.soak_seconds = soak
        args.capture_tool = args.tools_dir / "capture_uart.py"
        args.button_tool = args.tools_dir / "short_button.py"
        args.validator_tool = args.tools_dir / "validate_new_files.py"
        args.baud = 115200
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.command == "self-test":
        self_test()
        return 0
    summary = {"test": "ECG extended collection HIL", "result": "FAIL",
               "started_utc": cleanup.utc_now(), "campaign_hours": args.campaign_hours,
               "confirmation": args.destructive_confirmation, "sessions": [], "streams": []}
    created = False
    central = None
    try:
        cleanup.create_output_directory(args.output)
        created = True
        required = (args.elf, args.nrfutil, args.capture_tool, args.button_tool,
                    args.validator_tool)
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            raise cleanup.HilError("required input/tool absent: " + ", ".join(missing))
        summary.update({"elf": str(args.elf), "elf_sha256": cleanup.sha256(args.elf),
                        "probe": args.probe, "native_port": args.port,
                        "native_usb_serial": args.usb_serial, "drive": str(args.drive),
                        "central_jlink_serial": args.central_jlink_serial})
        button = cleanup.load_module(args.button_tool, "ecg_extended_button")
        validator = cleanup.load_module(args.validator_tool, "ecg_extended_validator")
        if args.wake_first:
            summary["wake"] = cleanup.press_button(button, "wake", args.probe, args.elf,
                                                     args.nm, args.nrfutil)
            time.sleep(args.wake_delay)
        baseline = cleanup.wait_for_snapshot(args.drive, args.remount_timeout,
                                             args.settle_seconds)
        if {name.lower() for name in baseline} != {"uuid.txt"}:
            raise cleanup.HilError("post-format media is ambiguous; expected only uuid.txt")
        (args.output / "baseline.json").write_text(json.dumps(baseline, indent=2) + "\n",
                                                    encoding="utf-8")
        summary["preflight"] = cleanup.collect_preflight(args.preflight_log,
                                                           args.output / "preflight")
        summary["preflight_irregularity_scans"] = [
            {"file": str(path), "scan": scan_native(path)}
            for path in sorted((args.output / "preflight").iterdir())]
        before, hashes = baseline, cleanup.content_snapshot(args.drive)
        central = Central(args, args.output)
        summary["peer"] = central.initial_peer()
        session_id = int(time.time()) & 0xffff_ffff or 1
        recording_ids = []

        def run_case(index: int, name: str, duration: float, action) -> None:
            nonlocal before, hashes
            args.acquisition_seconds = duration
            args.capture_timeout = duration + args.post_capture_seconds
            started = time.monotonic()
            def timed_action():
                action(time.monotonic() + duration)
            before, hashes, result = cleanup.run_session(
                args, index, before, hashes, button, validator, args.output, timed_action)
            result["name"] = name
            result["planned_acquisition_seconds"] = duration
            result["elapsed_seconds"] = round(time.monotonic() - started, 3)
            uart_path = args.output / f"session-{index}" / "uart.txt"
            result["native_irregularity_scan"] = scan_native(uart_path)
            marker = native_collection_start(
                uart_path.read_text(encoding="utf-8", errors="replace"))
            if marker is None:
                raise cleanup.HilError(
                    f"session {index} lacks complete native collection-start evidence")
            result["native_collection_start"] = marker
            copied = [args.output / f"session-{index}" / "new-files" / path
                      for path in result["new_files"]]
            result["log_irregularity_scans"] = [
                {"file": str(path), "scan": scan_native(path)}
                for path in copied if cleanup.LOG_NAME.fullmatch(path.name)]
            result["rollover_continuity"] = validate_rollovers(copied)
            ids = {entry["recording_id"] for entry in result["validation"]["ecg"]}
            if len(ids) != 1 or next(iter(ids)) in recording_ids:
                raise cleanup.HilError(f"session {index} has ambiguous or reused ECG recording ID")
            recording_ids.append(next(iter(ids)))
            if name == "collection-only-soak" and (
                    result["rollover_continuity"]["ecg_files"] < 2 or
                    result["rollover_continuity"]["accel_files"] < 2):
                raise cleanup.HilError("collection-only soak did not exercise both media rollovers")
            summary["sessions"].append(result)
            (args.output / f"checkpoint-{index}.json").write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8")

        def mixed(deadline):
            nonlocal session_id
            uart = args.output / "session-1" / "uart.txt"
            watch_native(uart, min(deadline, time.monotonic() + 30))
            summary["streams"].append(central.finite(session_id)); session_id += 1
            summary["streams"].append(central.disconnect_dropout(session_id)); session_id += 1
            summary["streams"].append(central.finite(session_id)); session_id += 1
            summary["streams"].append(central.central_restart(session_id, 1)); session_id += 1
            summary["streams"].append(central.finite(session_id)); session_id += 1
            remaining = deadline - time.monotonic()
            if remaining < 0:
                raise cleanup.HilError("mixed fault sequence exceeded its collection window")
            watch_native(uart, deadline)

        run_case(1, "mixed-stream-and-fault-recovery", args.mixed_minutes * 60, mixed)
        for cycle in range(args.cycle_count):
            def cycle_action(deadline, _cycle=cycle):
                nonlocal session_id
                ready = wait_for_collection_ready(
                    central.send, central.wait,
                    min(deadline, time.monotonic() + COLLECTION_START_TIMEOUT))
                stream = central.finite(session_id)
                stream["collection_ready"] = ready
                summary["streams"].append(stream); session_id += 1
                remaining = deadline - time.monotonic()
                if remaining < 0:
                    raise cleanup.HilError(f"collection cycle {_cycle + 1} exceeded its window")
                watch_native(args.output / f"session-{2 + _cycle}" / "uart.txt", deadline)
            run_case(2 + cycle, f"collection-cycle-{cycle + 1}",
                     args.cycle_minutes * 60, cycle_action)

        def soak(deadline):
            central.disconnect()
            watch_native(args.output / f"session-{2 + args.cycle_count}" / "uart.txt", deadline)
        run_case(2 + args.cycle_count, "collection-only-soak", args.soak_seconds, soak)
        summary["final_media_preservation"] = cleanup.require_preserved(args.drive, hashes)
        all_copied = [path for path in args.output.glob("session-*/new-files/*") if path.is_file()]
        summary["campaign_rollover_continuity"] = validate_rollovers(all_copied)
        summary["result"] = "PASS"
        code = 0
    except Exception as error:
        summary["error"] = f"{type(error).__name__}: {error}"
        code = 1
    finally:
        if central is not None:
            central.close()
        if created:
            summary["ended_utc"] = cleanup.utc_now()
            manifest = evidence_manifest(args.output)
            (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                                       encoding="utf-8")
            cleanup.write_summary(args.output, summary, True)
    print(json.dumps(summary, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
