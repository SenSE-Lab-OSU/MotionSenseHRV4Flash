#!/usr/bin/env python3
"""Run a non-destructive 2--3 hour PPG streaming endurance campaign."""

from __future__ import annotations

import argparse
import codecs
import json
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Offline self-test does not need pyserial.
    serial = None
    list_ports = None

import run_production_hil as production


FINITE_BYTES = 131072
PPG_RECORD_BYTES = 16
HISTORY_SECONDS = 8.25
MRLY_HEADER_BYTES = 12
MAX_MRLY_PAYLOAD = 512
COMMON_HEADER_BYTES = 12
MSG_START_ACK = 0x81
MSG_DATA = 0x82
MSG_END = 0x83
MSG_RESULT = 0x84
STATUS_SUCCESS = 0
STATUS_STOPPED = 8
PEER_READY_COMPLETE_PATTERN = (
    r"PEER_READY nus=1 smp=\d+ legacy=\d+ "
    r"peer_name=(\S+) peer_addr=(\S+) peer_addr_type=(\d+)(?:\r?\n|$)"
)


class HilError(production.HilError):
    pass


def stamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def disconnect_abort_pattern(session_id: int) -> str:
    """Require every authoritative abort milestone without imposing event order."""
    return (
        rf"(?=[\s\S]*\bDISCONNECT_SENT\b)"
        rf"(?=[\s\S]*(?:^|\r?\n)DISCONNECTED(?:[^\r\n]*)(?:\r?\n|$))"
        rf"(?=[\s\S]*\bRELAY_IDLE\b)"
        rf"(?=[\s\S]*\bPPG_PREFIX id={session_id}\b)"
        rf"(?=[\s\S]*\bSTREAM_END status=DISCONNECTED\b)"
    )


def terminal_and_relay_idle_pattern(terminal_pattern: str) -> str:
    """Require both segment milestones without imposing callback/relay order."""
    return (
        rf"(?=[\s\S]*(?:{terminal_pattern}))"
        rf"(?=[\s\S]*\bRELAY_IDLE\b)"
    )


def reset_peer_ready_pattern(code: int, peer: dict[str, str]) -> str:
    identity = (
        rf"PEER_READY nus=1 smp=\d+ legacy=\d+ "
        rf"peer_name={re.escape(peer['name'])} peer_addr={re.escape(peer['address'])} "
        rf"peer_addr_type={re.escape(peer['address_type'])}(?:\r?\n|$)"
    )
    return rf"RESET_REDISCOVERED code={code}[^\r\n]*[\s\S]*{identity}"


def validate_ppg_records(sensor: bytes) -> dict[str, object]:
    if not sensor or len(sensor) % PPG_RECORD_BYTES:
        raise HilError("PPG data is empty or not 16-byte record aligned")
    ticks = []
    for offset in range(0, len(sensor), PPG_RECORD_BYTES):
        record = sensor[offset:offset + PPG_RECORD_BYTES]
        for channel in range(4):
            value = int.from_bytes(record[channel * 3:channel * 3 + 3], "little")
            if value > 0x7FFFF:
                raise HilError(f"PPG channel {channel} exceeds 19 bits at record {offset // 16}")
        ticks.append(struct.unpack_from("<I", record, 12)[0])
    deltas = [((b - a) & 0xFFFFFFFF) for a, b in zip(ticks, ticks[1:])]
    if any(delta >= 0x80000000 for delta in deltas):
        raise HilError("PPG global tick moved backward")
    return {
        "tick_first": ticks[0], "tick_last": ticks[-1],
        "tick_repeats": sum(delta == 0 for delta in deltas),
        "tick_max_forward_delta": max(deltas, default=0),
    }


class EventLog:
    def __init__(self, root: Path):
        self.started = time.monotonic()
        self.path = root / "events.jsonl"
        self.output = self.path.open("x", encoding="utf-8", buffering=1)

    def add(self, kind: str, **fields) -> None:
        record = {"utc": stamp(), "elapsed_s": round(time.monotonic() - self.started, 3),
                  "kind": kind, **fields}
        self.output.write(json.dumps(record, sort_keys=True) + "\n")
        print(json.dumps(record, sort_keys=True), flush=True)

    def close(self) -> None:
        if not self.output.closed:
            self.output.close()


class RelayAudit:
    """Strictly audit MRLY/NUS framing and contiguous PPG DATA offsets."""

    def __init__(self, session_id: int, infinity: bool):
        self.session_id = session_id
        self.infinity = infinity
        self.pending = bytearray()
        self.sensor = bytearray()
        self.frames = 0
        self.data_messages = 0
        self.first_sequence = None
        self.last_sequence = None
        self.start_ack = False
        self.end_status = None
        self.expected_offset = 0

    def feed(self, raw: bytes) -> None:
        self.pending.extend(raw)
        while len(self.pending) >= MRLY_HEADER_BYTES:
            if self.pending[:4] != b"MRLY":
                raise HilError("relay lost MRLY synchronization")
            version, kind = self.pending[4], self.pending[5]
            length, sequence = struct.unpack_from("<HI", self.pending, 6)
            if (version, kind) != (1, 1) or not 0 < length <= MAX_MRLY_PAYLOAD:
                raise HilError(f"invalid MRLY header version/type/length {version}/{kind}/{length}")
            end = MRLY_HEADER_BYTES + length
            if len(self.pending) < end:
                return
            payload = bytes(self.pending[MRLY_HEADER_BYTES:end])
            del self.pending[:end]
            if self.last_sequence is not None and sequence != ((self.last_sequence + 1) & 0xFFFFFFFF):
                raise HilError(f"MRLY sequence gap: {self.last_sequence} -> {sequence}")
            self.first_sequence = sequence if self.first_sequence is None else self.first_sequence
            self.last_sequence = sequence
            self.frames += 1
            self._nus(payload)

    def _nus(self, frame: bytes) -> None:
        if len(frame) < COMMON_HEADER_BYTES or frame[:2] != b"MS":
            raise HilError("invalid NUS common header")
        version, kind = frame[2], frame[3]
        session_id, length, flags = struct.unpack_from("<IHH", frame, 4)
        if version != 0 or flags != 0 or len(frame) != COMMON_HEADER_BYTES + length:
            raise HilError("invalid NUS v0 length/version/flags")
        if session_id != self.session_id:
            raise HilError(f"NUS session mismatch: expected {self.session_id}, got {session_id}")
        payload = frame[COMMON_HEADER_BYTES:]
        if kind == MSG_START_ACK:
            if self.start_ack or len(payload) != 16:
                raise HilError("invalid or duplicate START_ACK")
            if bool(payload[0]) != self.infinity or payload[1] != 8:
                raise HilError("START_ACK mode/history mismatch")
            total = struct.unpack_from("<Q", payload, 8)[0]
            if (not self.infinity and total != FINITE_BYTES) or (self.infinity and total != 0):
                raise HilError(f"START_ACK total byte mismatch: {total}")
            self.start_ack = True
        elif kind == MSG_DATA:
            if not self.start_ack or len(payload) <= 8:
                raise HilError("DATA before START_ACK or with empty payload")
            offset = struct.unpack_from("<Q", payload)[0]
            data = payload[8:]
            if offset != self.expected_offset:
                raise HilError(f"PPG DATA offset gap: expected {self.expected_offset}, got {offset}")
            self.sensor.extend(data)
            self.expected_offset += len(data)
            self.data_messages += 1
        elif kind == MSG_END:
            if len(payload) != 2 or self.end_status is not None:
                raise HilError("invalid or duplicate END")
            self.end_status = struct.unpack_from("<H", payload)[0]
        elif kind == MSG_RESULT:
            status = struct.unpack_from("<H", payload)[0] if len(payload) == 2 else None
            raise HilError(f"stream RESULT status={status}")
        else:
            raise HilError(f"unknown NUS message type {kind:#x}")

    def finish(self, expectation: str) -> dict[str, object]:
        truncated = len(self.pending)
        if truncated and expectation != "aborted":
            raise HilError(f"truncated MRLY tail ({truncated} bytes)")
        if not self.start_ack or not self.sensor:
            raise HilError("stream produced no START_ACK or PPG DATA")
        complete = len(self.sensor) // PPG_RECORD_BYTES * PPG_RECORD_BYTES
        record_result = validate_ppg_records(bytes(self.sensor[:complete]))
        if expectation == "finite":
            if self.end_status != STATUS_SUCCESS or len(self.sensor) != FINITE_BYTES:
                raise HilError(f"finite stream incomplete: status={self.end_status} bytes={len(self.sensor)}")
        elif expectation == "stopped":
            if self.end_status != STATUS_STOPPED or len(self.sensor) % PPG_RECORD_BYTES:
                raise HilError(f"stopped stream was not a clean record boundary: {self.end_status}")
        elif expectation == "aborted":
            if self.end_status not in (None, 0x000D):
                raise HilError(f"aborted stream had unexpected END status {self.end_status}")
        else:
            raise AssertionError(expectation)
        return {
            "session_id": self.session_id, "mode": "INFINITY" if self.infinity else "FINITE",
            "expectation": expectation, "frames": self.frames,
            "first_relay_sequence": self.first_sequence, "last_relay_sequence": self.last_sequence,
            "data_messages": self.data_messages, "sensor_bytes": len(self.sensor),
            "complete_records": complete // PPG_RECORD_BYTES,
            "end_status": self.end_status, "truncated_relay_tail": truncated, **record_result,
        }


class Segment:
    def __init__(self, root: Path, session_id: int, infinity: bool):
        root.mkdir(parents=True, exist_ok=False)
        self.root = root
        self.audit = RelayAudit(session_id, infinity)
        self.command_rx = (root / "command.rx.raw").open("xb")
        self.command_tx = (root / "command.tx.raw").open("xb")
        self.relay = (root / "relay.mrly.raw").open("xb")

    def rx(self, raw: bytes) -> None:
        self.command_rx.write(raw); self.command_rx.flush()

    def tx(self, raw: bytes) -> None:
        self.command_tx.write(raw); self.command_tx.flush()

    def relay_rx(self, raw: bytes) -> None:
        self.relay.write(raw); self.relay.flush(); self.audit.feed(raw)

    def finish(self, expectation: str) -> dict[str, object]:
        for output in (self.command_rx, self.command_tx, self.relay):
            output.close()
        raw = (self.root / "command.rx.raw").read_bytes()
        (self.root / "command.decoded.txt").write_text(
            raw.decode("utf-8", errors="replace"), encoding="utf-8")
        result = self.audit.finish(expectation)
        (self.root / "summary.json").write_text(json.dumps(result, indent=2) + "\n",
                                                 encoding="utf-8")
        return result

    def abandon(self) -> None:
        for output in (self.command_rx, self.command_tx, self.relay):
            if not output.closed:
                output.close()


class Central:
    def __init__(self, args, events: EventLog, root: Path):
        self.args, self.events = args, events
        self.command = self.relay = None
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
        self.text = ""
        self.global_rx = (root / "central-command.rx.raw").open("xb")
        self.global_tx = (root / "central-command.tx.raw").open("xb")
        self.global_relay = (root / "central-relay.mrly.raw").open("xb")
        self.global_text = (root / "central-command.txt").open("x", encoding="utf-8", newline="")
        self.segment: Segment | None = None
        self.active_session: int | None = None
        self.stream_mark = 0

    def open(self, timeout: float = 60.0) -> None:
        if serial is None:
            raise HilError("pyserial is required for a hardware run")
        deadline, last = time.monotonic() + timeout, None
        while time.monotonic() < deadline:
            try:
                self.command = serial.Serial(self.args.command_port, 115200, timeout=0,
                                             write_timeout=2, rtscts=False, dsrdtr=False)
                self.relay = serial.Serial(self.args.relay_port, 1_000_000, timeout=0,
                                           write_timeout=2, rtscts=True, dsrdtr=False)
                self.command.dtr = self.relay.dtr = True
                self.command.reset_input_buffer(); self.relay.reset_input_buffer()
                self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
                self.events.add("central_ports_opened", command=self.args.command_port,
                                relay=self.args.relay_port)
                return
            except Exception as error:
                last = error
                self.close_ports()
                time.sleep(1)
        raise HilError(f"Central ports did not open: {last}")

    def close_ports(self) -> None:
        for port in (self.relay, self.command):
            if port is not None:
                try: port.close()
                except Exception: pass
        self.command = self.relay = None

    def close(self) -> None:
        self.close_ports()
        for output in (self.global_rx, self.global_tx, self.global_relay, self.global_text):
            if not output.closed: output.close()

    def pump(self) -> None:
        if self.command is None or self.relay is None:
            return
        raw = self.command.read(self.command.in_waiting or 0)
        if raw:
            self.global_rx.write(raw); self.global_rx.flush()
            if self.segment: self.segment.rx(raw)
            text = self.decoder.decode(raw)
            self.text += text
            self.global_text.write(text); self.global_text.flush()
            print(text, end="", flush=True)
        raw = self.relay.read(self.relay.in_waiting or 0)
        if raw:
            self.global_relay.write(raw); self.global_relay.flush()
            if self.segment: self.segment.relay_rx(raw)

    def send(self, command: str) -> int:
        if self.command is None:
            raise HilError("Central command port is closed")
        raw = (command + "\r\n").encode("ascii")
        mark = len(self.text)
        self.global_tx.write(raw); self.global_tx.flush()
        if self.segment: self.segment.tx(raw)
        self.events.add("central_command", command=command)
        self.command.write(raw); self.command.flush()
        return mark

    def wait(self, mark: int, label: str, pattern: str, timeout: float,
             reject: str | None = None) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            current = self.text[mark:]
            if reject and re.search(reject, current):
                raise HilError(f"Central error while waiting for {label}: {current[-1000:]}")
            if re.search(pattern, current):
                self.events.add("milestone", label=label)
                return current
            time.sleep(0.01)
        raise HilError(f"timed out waiting for {label}; tail={self.text[mark:][-1000:]}")

    def delay(self, seconds: float, label: str, status_interval: float = 60.0) -> None:
        self.events.add("delay_started", label=label, seconds=round(seconds, 3))
        deadline, next_status = time.monotonic() + seconds, time.monotonic() + status_interval
        while time.monotonic() < deadline:
            self.pump()
            if self.command is not None and self.active_session and time.monotonic() >= next_status:
                self.send("status")
                next_status += status_interval
            time.sleep(0.01)
        self.events.add("delay_ended", label=label)

    def identify(self) -> None:
        mark = self.send("help")
        self.wait(mark, "Central help", r"COMMANDS:", 8)

    def connect_same(self, expected: dict[str, str] | None = None) -> dict[str, str]:
        for attempt in range(1, 13):
            mark = self.send("connect ppg")
            text = self.wait(mark, "complete PPG PEER_READY", PEER_READY_COMPLETE_PATTERN, 35,
                             r"NUS_UNAVAILABLE|COLLECT_UNAVAILABLE|\bERROR\b")
            matches = list(re.finditer(PEER_READY_COMPLETE_PATTERN, text))
            if matches:
                found = {"name": matches[-1].group(1), "address": matches[-1].group(2),
                         "address_type": matches[-1].group(3)}
                correct = found["name"] == self.args.peer_name and (
                    expected is None or found == expected)
                if correct and "nus=1" in matches[-1].group(0):
                    self.events.add("peer_verified", **found)
                    return found
            self.events.add("peer_mismatch", attempt=attempt,
                            response=text[-1000:])
            mark = self.send("disconnect")
            self.wait(mark, "mismatched peer disconnect", r"DISCONNECTED|ERR no active", 12)
            time.sleep(2)
        raise HilError(f"could not connect to exact peer {self.args.peer_name}")

    def collect(self, enabled: bool) -> None:
        value = "on" if enabled else "off"
        mark = self.send(f"collect {value}")
        pattern = rf"COLLECT_RESULT[^\r\n]*enabled={int(enabled)}[^\r\n]*status=success"
        self.wait(mark, f"collect {value}", pattern, 30, r"\bERR(?:OR)?\b")

    def start_segment(self, root: Path, session_id: int, infinity: bool) -> Segment:
        if self.segment is not None:
            raise HilError("a stream segment is already active")
        if self.relay is None:
            raise HilError("relay port is closed")
        self.relay.reset_input_buffer()
        self.segment = Segment(root, session_id, infinity)
        self.active_session = session_id
        command = f"start infinity {session_id}" if infinity else f"start {session_id}"
        mark = self.send(command)
        self.stream_mark = mark
        mode = "INFINITY" if infinity else "FINITE"
        self.wait(mark, f"START_ACK {session_id}",
                  rf"START_ACK[^\r\n]*mode={mode}[^\r\n]*", 20,
                  r"START_RESULT|PROTOCOL_ERROR|ERROR relay|ERROR NUS")
        return self.segment

    def finish_segment(self, expectation: str, command_text_pattern: str,
                       timeout: float = 300.0) -> dict[str, object]:
        if self.segment is None:
            raise HilError("no active stream segment")
        mark = self.stream_mark
        if command_text_pattern:
            terminal_and_idle = terminal_and_relay_idle_pattern(command_text_pattern)
            self.wait(mark, f"stream terminal and RELAY_IDLE {self.active_session}", terminal_and_idle,
                      timeout, r"PROTOCOL_ERROR|ERROR relay|ERROR NUS|BUFFER_OVERFLOW|STORAGE_ERROR")
        else:
            self.wait(mark, "RELAY_IDLE", r"RELAY_IDLE", timeout)
        self.delay(0.25, "relay host drain", status_interval=9999)
        segment, self.segment = self.segment, None
        self.active_session = None
        return segment.finish(expectation)

    def stop_segment(self) -> dict[str, object]:
        if self.active_session is None:
            raise HilError("no stream to stop")
        session_id = self.active_session
        mark = self.send(f"stop {session_id}")
        self.wait(mark, f"STOP_SENT {session_id}", rf"STOP_SENT id={session_id}", 15,
                  r"\bERR(?:OR)?\b")
        return self.finish_segment("stopped", r"STREAM_END status=STOPPED", 90)

    def abort_by_disconnect(self) -> dict[str, object]:
        session_id = self.active_session
        if session_id is None:
            raise HilError("no stream to disconnect")
        mark = self.send("disconnect")
        self.wait(mark, "intentional stream disconnect",
                  disconnect_abort_pattern(session_id), 20,
                  r"PROTOCOL_ERROR|ERROR relay|ERROR NUS|BUFFER_OVERFLOW|STORAGE_ERROR")
        self.delay(0.25, "relay host drain", status_interval=9999)
        segment, self.segment = self.segment, None
        self.active_session = None
        return segment.finish("aborted")

    def abandon_for_central_reset(self) -> dict[str, object]:
        if self.segment is None:
            raise HilError("no segment active before Central reset")
        self.pump()
        segment, self.segment = self.segment, None
        self.active_session = None
        self.close_ports()
        return segment.finish("aborted")

    def cleanup(self) -> list[str]:
        errors = []
        if self.command is None:
            return errors
        if self.active_session is not None:
            try: self.stop_segment()
            except Exception as error:
                errors.append(f"stop: {error}")
                if self.segment is not None:
                    self.segment.abandon()
                    self.segment = None
                self.active_session = None
        for command, pattern in (("collect off", r"COLLECT_RESULT.*enabled=0.*status=success|ERR"),
                                 ("disconnect", r"DISCONNECTED|ERR no active")):
            try:
                mark = self.send(command); self.wait(mark, f"cleanup {command}", pattern, 15)
            except Exception as error: errors.append(f"{command}: {error}")
        return errors


class NativeCapture(threading.Thread):
    """Continuous identity-checked PPG CDC capture that survives re-enumeration."""

    def __init__(self, args, events: EventLog):
        super().__init__(daemon=True)
        self.args, self.events = args, events
        self.stop_requested = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.reconnects = 0
        self.connected_at_end = False
        self.raw_bytes = 0

    def _port(self) -> str | None:
        matches = [item for item in list_ports.comports()
                   if (item.serial_number or "").upper() == self.args.ppg_usb_serial.upper()]
        if len(matches) != 1:
            return None
        actual = matches[0].device
        if actual.upper() != self.args.ppg_port.upper() and not self.args.allow_port_change:
            self.error = f"PPG USB serial moved from {self.args.ppg_port} to {actual}"
            return None
        return actual

    def run(self) -> None:
        raw_path = self.args.output / "native-uart.raw"
        text_path = self.args.output / "native-uart.txt"
        chunks_path = self.args.output / "native-uart.chunks.jsonl"
        try:
            with raw_path.open("xb") as raw, text_path.open("x", encoding="utf-8", newline="") as text, \
                    chunks_path.open("x", encoding="utf-8", buffering=1) as chunks:
                port = None
                while not self.stop_requested.is_set():
                    if port is None:
                        name = self._port()
                        if self.error:
                            return
                        if name is None:
                            time.sleep(0.25); continue
                        try:
                            port = serial.Serial(name, self.args.ppg_baud, timeout=0.1,
                                                 write_timeout=1, rtscts=False, dsrdtr=False)
                            port.dtr = True
                            self.reconnects += int(self.ready.is_set())
                            self.ready.set()
                            self.events.add("native_uart_connected", port=name,
                                            reconnects=self.reconnects)
                        except Exception:
                            port = None; time.sleep(0.25); continue
                    try:
                        payload = port.read(port.in_waiting or 1)
                        if payload:
                            decoded = payload.decode("utf-8", errors="replace")
                            raw.write(payload); raw.flush(); text.write(decoded); text.flush()
                            chunks.write(json.dumps({"utc": stamp(), "offset": self.raw_bytes,
                                                     "length": len(payload)}) + "\n")
                            self.raw_bytes += len(payload)
                    except Exception as error:
                        self.events.add("native_uart_disconnected", error=str(error))
                        try: port.close()
                        except Exception: pass
                        port = None
                self.connected_at_end = port is not None
                if port is not None: port.close()
        except Exception as error:
            self.error = str(error)

    def stop(self) -> dict[str, object]:
        self.stop_requested.set(); self.join(timeout=5)
        if self.is_alive():
            raise HilError("native UART capture thread did not stop")
        if self.error:
            raise HilError(f"native UART capture failed: {self.error}")
        return {"result": "PASS", "raw_bytes": self.raw_bytes,
                "reconnects": self.reconnects, "connected_at_end": self.connected_at_end,
                "storage_scan": production.scan_storage_errors(
                    (self.args.output / "native-uart.txt").read_text(
                        encoding="utf-8", errors="replace"))}


def wait_native_ready(capture: NativeCapture, timeout: float = 15) -> None:
    if not capture.ready.wait(timeout):
        raise HilError(capture.error or "PPG native UART identity did not become available")


def recording_checkpoint(args, case_root: Path, before, prior_hashes) -> tuple[dict, dict, dict]:
    after = production.wait_for_snapshot(args.drive, args.remount_timeout, args.settle_seconds)
    names = production.new_recording_files(before, after)
    kinds = {"ppg" if production.PPG_NAME.fullmatch(name) else
             "accel" if production.ACCEL_NAME.fullmatch(name) else "log" for name in names}
    if kinds != {"ppg", "accel", "log"}:
        raise HilError(f"new recording set incomplete: {names}")
    preservation = production.require_preserved(args.drive, prior_hashes)
    media = case_root / "media"
    media.mkdir(exist_ok=False)
    validation = []
    graceful_logs = 0
    for name in names:
        target = media / name
        shutil.copy2(args.drive / name, target)
        checked = production.validate_recording_file(sys.modules[__name__], target)
        if checked["kind"] == "log":
            production.scan_storage_errors(checked["text"])
            positions = [checked["text"].find(marker) for marker in production.SHUTDOWN_MARKERS]
            if all(position >= 0 for position in positions) and positions == sorted(positions):
                graceful_logs += 1
            checked["close"] = "graceful" if all(position >= 0 for position in positions) else "interrupted"
            checked.pop("text", None)
        validation.append(checked)
    if graceful_logs == 0:
        raise HilError("case produced no gracefully closed log")
    (case_root / "post-media.json").write_text(json.dumps(after, indent=2) + "\n",
                                                encoding="utf-8")
    result = {"result": "PASS", "new_files": names, "validated": validation,
              "graceful_logs": graceful_logs, "prior_files": preservation}
    return after, production.content_snapshot(args.drive), result


def reset_central(args, events: EventLog, case_root: Path) -> dict[str, object]:
    command = [str(args.nrfutil), "--log-output", "stdout", "--log-level", "error",
               "device", "reset", "--reset-kind", "RESET_SYSTEM",
               "--serial-number", args.central_jlink_serial]
    events.add("central_reset_started", kind_name="RESET_SYSTEM")
    completed = subprocess.run(command, text=True, capture_output=True, timeout=120)
    (case_root / "central-reset.stdout.txt").write_text(completed.stdout, encoding="utf-8")
    (case_root / "central-reset.stderr.txt").write_text(completed.stderr, encoding="utf-8")
    if completed.returncode:
        raise HilError(f"nrfutil Central RESET_SYSTEM failed ({completed.returncode})")
    events.add("central_reset_completed", returncode=completed.returncode)
    return {"result": "PASS", "returncode": completed.returncode,
            "stdout": "central-reset.stdout.txt", "stderr": "central-reset.stderr.txt"}


class Campaign:
    def __init__(self, args):
        self.args = args
        self.summary = {"test": "PPG extended streaming HIL", "result": "FAIL",
                        "started_utc": stamp(), "configuration": vars(args).copy(), "cases": []}
        self.summary["configuration"] = {key: str(value) if isinstance(value, Path) else value
                                         for key, value in self.summary["configuration"].items()}
        self.events = EventLog(args.output)
        self.central = Central(args, self.events, args.output)
        self.native = NativeCapture(args, self.events)
        self.session_id = args.session_id_base
        self.before = self.prior_hashes = None
        self.peer = None
        self.campaign_start = time.monotonic()
        self.checkpoint_index = 0

    def next_id(self) -> int:
        value = self.session_id
        self.session_id += 1
        if self.session_id > 0xFFFFFFFF:
            raise HilError("session ID range exhausted")
        return value

    def case(self, name: str) -> tuple[Path, dict[str, object]]:
        root = self.args.output / f"case-{len(self.summary['cases']) + 1:02d}-{name}"
        root.mkdir(exist_ok=False)
        result = {"name": name, "result": "FAIL", "started_utc": stamp(), "segments": []}
        self.summary["cases"].append(result)
        self.events.add("case_started", name=name)
        return root, result

    def complete(self, root: Path, result: dict[str, object], media: bool = True) -> None:
        if media:
            self.before, self.prior_hashes, result["media"] = recording_checkpoint(
                self.args, root, self.before, self.prior_hashes)
        result["result"] = "PASS"; result["ended_utc"] = stamp()
        self.checkpoint_index += 1
        checkpoint = self.args.output / f"checkpoint-{self.checkpoint_index:02d}.json"
        checkpoint.write_text(json.dumps(self.summary, indent=2) + "\n", encoding="utf-8")
        self.events.add("case_completed", name=result["name"])

    def finite(self, root: Path, label: str) -> dict[str, object]:
        session_id = self.next_id()
        self.central.start_segment(root / f"segment-{label}-{session_id}", session_id, False)
        return self.central.finish_segment("finite", rf"STREAM_OK id={session_id}",
                                           self.args.stream_timeout)

    def infinity(self, root: Path, label: str, seconds: float) -> dict[str, object]:
        session_id = self.next_id()
        self.central.start_segment(root / f"segment-{label}-{session_id}", session_id, True)
        self.central.delay(seconds, label)
        return self.central.stop_segment()

    def collect_start(self) -> None:
        self.central.collect(True)
        self.central.delay(HISTORY_SECONDS, "history fill", status_interval=9999)

    def run(self) -> None:
        self.before = production.wait_for_snapshot(
            self.args.drive, self.args.remount_timeout, self.args.settle_seconds)
        (self.args.output / "baseline-media.json").write_text(
            json.dumps(self.before, indent=2) + "\n", encoding="utf-8")
        self.prior_hashes = production.content_snapshot(self.args.drive)
        self.summary["baseline_files"] = sorted(self.prior_hashes)
        self.native.start(); wait_native_ready(self.native)
        self.central.open(); self.central.identify()
        mark = self.central.send("disconnect")
        self.central.wait(mark, "initial idle", r"DISCONNECTED|ERR no active connection", 15)
        self.peer = self.central.connect_same()
        self.summary["peer"] = self.peer

        root, case = self.case("baseline")
        self.collect_start(); case["segments"].append(
            self.infinity(root, "baseline", self.args.baseline_seconds))
        self.central.collect(False); self.complete(root, case)

        root, case = self.case("intentional-disconnect")
        self.collect_start()
        sid = self.next_id(); self.central.start_segment(root / f"segment-abort-{sid}", sid, True)
        self.central.delay(self.args.fault_pre_seconds, "pre-disconnect streaming")
        aborted = self.central.abort_by_disconnect()
        self.central.delay(self.args.dropout_seconds, "intentional BLE outage", status_interval=9999)
        self.central.connect_same(self.peer)
        recovery = self.finite(root, "recovery")
        aborted["fault_classification"] = "EXPECTED_AFTER_RECOVERY"
        case["segments"].extend((aborted, recovery))
        self.central.collect(False); self.complete(root, case)

        root, case = self.case("central-reset-system")
        self.collect_start()
        sid = self.next_id(); self.central.start_segment(root / f"segment-abort-{sid}", sid, True)
        self.central.delay(self.args.fault_pre_seconds, "pre-Central-reset streaming")
        aborted = self.central.abandon_for_central_reset()
        case["reset"] = reset_central(self.args, self.events, root)
        self.central.open(self.args.central_restart_timeout); self.central.identify()
        self.central.connect_same(self.peer)
        recovery = self.finite(root, "recovery")
        aborted["fault_classification"] = "EXPECTED_AFTER_RECOVERY"
        case["segments"].extend((aborted, recovery))
        self.central.collect(False); self.complete(root, case)

        root, case = self.case("ppg-reset-121")
        self.collect_start(); case["segments"].append(
            self.infinity(root, "pre-reset", self.args.fault_pre_seconds))
        mark = self.central.send("reset 121")
        reset_text = self.central.wait(mark, "PPG reset 121 ready rediscovery",
            reset_peer_ready_pattern(121, self.peer), self.args.ppg_reset_timeout,
            r"RESET_RECONNECT_TIMEOUT|RESET_DISCONNECT_TIMEOUT|ERR reset")
        for milestone in ("RESET_DISCONNECTED code=121", "RESET_ADVERTISING code=121",
                          "RESET_RECONNECTED code=121", "RESET_REDISCOVERED code=121"):
            if milestone not in reset_text:
                raise HilError(f"PPG reset omitted milestone: {milestone}")
        self.collect_start(); recovery = self.finite(root, "recovery")
        case["segments"].append(recovery)
        self.central.collect(False); self.complete(root, case)

        for cycle in range(1, self.args.cycle_count + 1):
            root, case = self.case(f"cycle-{cycle:02d}")
            self.collect_start()
            if cycle % 2:
                case["segments"].append(self.finite(root, "finite"))
            else:
                case["segments"].append(self.infinity(
                    root, "short-infinity", self.args.short_infinity_seconds))
            self.central.collect(False); self.complete(root, case)
            if cycle % 2 == 0:
                mark = self.central.send("disconnect")
                self.central.wait(mark, "cycle reconnect disconnect", r"DISCONNECTED", 15)
                self.central.connect_same(self.peer)

        root, case = self.case("final-soak")
        reserve = self.args.final_validation_seconds
        remaining = self.args.campaign_minutes * 60 - (time.monotonic() - self.campaign_start) - reserve
        soak = max(self.args.minimum_final_soak_seconds, remaining)
        case["planned_seconds"] = round(soak, 3)
        self.collect_start(); case["segments"].append(self.infinity(root, "soak", soak))
        self.central.collect(False); self.complete(root, case)

        final = production.require_preserved(self.args.drive, self.prior_hashes)
        mark = self.central.send("status")
        final_status = self.central.wait(mark, "final STATUS", r"STATUS[^\r\n]*", 8)
        if self.peer["name"] not in final_status or "binary_mode=0" not in final_status:
            raise HilError("final Central status is not idle on the expected peer")
        mark = self.central.send("disconnect")
        self.central.wait(mark, "final disconnect", r"DISCONNECTED", 15)
        self.summary["final_preservation"] = final
        self.summary["result"] = "PASS"


def self_test() -> None:
    record = bytes((1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0)) + struct.pack("<I", 42)
    sid = 7001
    def nus(kind, payload):
        return b"MS" + bytes((0, kind)) + struct.pack("<IHH", sid, len(payload), 0) + payload
    def mrly(seq, payload):
        return b"MRLY\x01\x01" + struct.pack("<HI", len(payload), seq) + payload
    ack = bytes((0, 8)) + b"\0" * 6 + struct.pack("<Q", FINITE_BYTES)
    audit = RelayAudit(sid, False)
    frames = [mrly(10, nus(MSG_START_ACK, ack))]
    sequence = 11
    offset = 0
    chunk = record * 20
    while offset < FINITE_BYTES:
        payload = chunk[:min(len(chunk), FINITE_BYTES - offset)]
        frames.append(mrly(sequence, nus(MSG_DATA, struct.pack("<Q", offset) + payload)))
        sequence += 1
        offset += len(payload)
    frames.append(mrly(sequence, nus(MSG_END, struct.pack("<H", STATUS_SUCCESS))))
    raw = b"".join(frames)
    # Feed in awkward host-UART boundaries.
    audit.feed(raw[:17]); audit.feed(raw[17:])
    result = audit.finish("finite")
    assert result["sensor_bytes"] == FINITE_BYTES and result["complete_records"] == 8192
    aborted = RelayAudit(sid, True)
    infinity_ack = bytes((1, 8)) + b"\0" * 14
    aborted.feed(mrly(2, nus(MSG_START_ACK, infinity_ack)))
    aborted.feed(mrly(3, nus(MSG_DATA, struct.pack("<Q", 0) + record)) + b"MR")
    assert aborted.finish("aborted")["truncated_relay_tail"] == 2
    broken = RelayAudit(sid, True)
    try:
        broken.feed(mrly(1, nus(MSG_START_ACK, infinity_ack)) +
                    mrly(3, nus(MSG_DATA, struct.pack("<Q", 0) + record)))
    except HilError:
        pass
    else:
        raise AssertionError("MRLY sequence gap was accepted")
    transcript = "\n".join((
        "DISCONNECT_SENT", "NUS subscription removed", "RELAY_IDLE",
        "DISCONNECTED reason=0x16", f"PPG_PREFIX id={sid} complete_records=100",
        "STREAM_END status=DISCONNECTED(0x000d) bytes=1600 data=10",
    ))
    pattern = disconnect_abort_pattern(sid)
    assert re.search(pattern, transcript)
    assert not re.search(pattern, transcript.replace(f"id={sid}", f"id={sid + 1}"))
    assert not re.search(pattern, transcript.replace("RELAY_IDLE", ""))
    assert not re.search(pattern, transcript.replace("DISCONNECTED reason=0x16\n", ""))
    stopped = "STREAM_END status=STOPPED(0x0008)"
    terminal_pattern = terminal_and_relay_idle_pattern(r"STREAM_END status=STOPPED")
    assert re.search(terminal_pattern, f"RELAY_IDLE\n{stopped}")
    assert re.search(terminal_pattern, f"{stopped}\nRELAY_IDLE")
    assert not re.search(terminal_pattern, stopped)
    peer = {"name": "MSense4PPG-TEST", "address": "AA:BB:CC:DD:EE:FF",
            "address_type": "0"}
    partial_ready = (
        "PEER_READY nus=1 smp=1 legacy=1 peer_name=MSense4PPG-TEST "
        "peer_addr=AA:BB:CC"
    )
    complete_ready = (
        "PEER_READY nus=1 smp=1 legacy=1 peer_name=MSense4PPG-TEST "
        "peer_addr=AA:BB:CC:DD:EE:FF peer_addr_type=0\n"
    )
    assert not re.search(PEER_READY_COMPLETE_PATTERN, partial_ready)
    assert re.search(PEER_READY_COMPLETE_PATTERN, complete_ready)
    assert re.search(PEER_READY_COMPLETE_PATTERN, complete_ready.rstrip("\n"))
    assert re.search(PEER_READY_COMPLETE_PATTERN,
                     partial_ready + ":DD:EE:FF peer_addr_type=0")
    reset_pattern = reset_peer_ready_pattern(121, peer)
    assert not re.search(reset_pattern, "RESET_REDISCOVERED code=121\n" + partial_ready)
    assert re.search(reset_pattern, "RESET_REDISCOVERED code=121\n" + complete_ready)
    assert not re.search(reset_pattern, complete_ready + "RESET_REDISCOVERED code=121\n")
    print("run_extended_streaming_hil.py self-test: PASS (no hardware access)")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-test", help="run parser checks without hardware")
    run = sub.add_parser("run", help="run the non-destructive endurance campaign")
    run.add_argument("--command-port", required=True)
    run.add_argument("--relay-port", required=True)
    run.add_argument("--central-jlink-serial", required=True)
    run.add_argument("--nrfutil", type=Path, required=True)
    run.add_argument("--ppg-port", required=True)
    run.add_argument("--ppg-usb-serial", required=True)
    run.add_argument("--peer-name", required=True)
    run.add_argument("--drive", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--session-id-base", type=int, required=True)
    run.add_argument("--campaign-minutes", type=float, default=150)
    run.add_argument("--baseline-seconds", type=float, default=900)
    run.add_argument("--fault-pre-seconds", type=float, default=300)
    run.add_argument("--dropout-seconds", type=float, default=180)
    run.add_argument("--short-infinity-seconds", type=float, default=60)
    run.add_argument("--cycle-count", type=int, default=8)
    run.add_argument("--minimum-final-soak-seconds", type=float, default=900)
    run.add_argument("--final-validation-seconds", type=float, default=300)
    run.add_argument("--stream-timeout", type=float, default=300)
    run.add_argument("--central-restart-timeout", type=float, default=90)
    run.add_argument("--ppg-reset-timeout", type=float, default=330)
    run.add_argument("--remount-timeout", type=float, default=90)
    run.add_argument("--settle-seconds", type=float, default=3)
    run.add_argument("--ppg-baud", type=int, default=115200)
    run.add_argument("--allow-port-change", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "run":
        durations = (args.baseline_seconds, args.fault_pre_seconds, args.dropout_seconds,
                     args.short_infinity_seconds, args.minimum_final_soak_seconds,
                     args.final_validation_seconds, args.stream_timeout,
                     args.central_restart_timeout, args.ppg_reset_timeout,
                     args.remount_timeout, args.settle_seconds)
        if not 120 <= args.campaign_minutes <= 180:
            parser.error("--campaign-minutes must be from 120 through 180")
        if any(value <= 0 for value in durations):
            parser.error("durations and timeouts must be positive")
        if args.cycle_count < 2 or args.cycle_count > 32:
            parser.error("--cycle-count must be 2 through 32")
        needed_ids = 8 + args.cycle_count
        if not 0 < args.session_id_base <= 0xFFFFFFFF - needed_ids:
            parser.error("--session-id-base does not leave enough nonzero uint32 IDs")
        if not args.peer_name.startswith("MSense4PPG-"):
            parser.error("--peer-name must be an exact MSense4PPG-... name")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.command == "self-test":
        self_test(); return 0
    summary = {"test": "PPG extended streaming HIL", "result": "FAIL",
               "started_utc": stamp()}
    created = False
    campaign = None
    try:
        production.create_output_directory(args.output); created = True
        if not args.nrfutil.is_file():
            raise HilError(f"nrfutil is absent: {args.nrfutil}")
        campaign = Campaign(args)
        campaign.run()
        summary = campaign.summary
        code = 0
    except Exception as error:
        if campaign:
            summary = campaign.summary
        summary["error"] = str(error)
        code = 1
    finally:
        if campaign:
            try: summary["cleanup"] = campaign.central.cleanup()
            except Exception as error: summary["cleanup"] = [str(error)]
            try: summary["native_uart"] = campaign.native.stop()
            except Exception as error:
                summary["native_uart"] = {"result": "FAIL", "error": str(error)}
                summary["result"] = "FAIL"; code = 1
            try:
                if campaign.central.text:
                    production.scan_storage_errors(campaign.central.text)
                summary["central_storage_scan"] = {"result": "PASS"}
            except Exception as error:
                summary["central_storage_scan"] = {"result": "FAIL", "error": str(error)}
                summary["result"] = "FAIL"; code = 1
            campaign.central.close(); campaign.events.close()
        summary["ended_utc"] = stamp()
        if created:
            (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                                       encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
