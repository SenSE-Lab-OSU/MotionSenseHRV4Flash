#!/usr/bin/env python3
"""Format, DFU, and run two production PPG collection/validation sessions."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path


FILE_BYTES = 4 * 1024 * 1024
RECORD_BYTES = 16
PPG_NAME = re.compile(r"^(?:\d+)?ppg\d+\.bin$", re.IGNORECASE)
ACCEL_NAME = re.compile(r"^(?:\d+)?ac\d+(?:_\d{4})?\.bin$", re.IGNORECASE)
LOG_NAME = re.compile(r"^(?:\d+)?log\d+\.txt$", re.IGNORECASE)
RECORDING_NAME = re.compile(
    r"^(?:\d+)?(?:ppg\d+\.bin|ac\d+(?:_\d{4})?\.bin|log\d+\.txt)$",
    re.IGNORECASE,
)
SHUTDOWN_MARKERS = (
    "Leaving PPG collection mode",
    "Closing storage log",
    "Storage log end",
)
STORAGE_CONTEXT = re.compile(
    r"\b(?:nand|spi_nand|nand_disk|nor|spi_nor|fatfs|filesystem|zephyrfilesystem|dhara|disk|fs)\b|"
    r"\b(?:spi4|spim4|nrfx_spim|spi[-_ ]controller|storage[-_ ]bus)\b|"
    r"\bstorage(?:\b|_)|\b(?:double|duplicate)[-_ ]program\b|"
    r"\b(?:duplicate|reused?)[-_ ](?:logical|physical)?[-_ ]?page\b",
    re.IGNORECASE,
)
ERROR_WORD = re.compile(
    r"fail(?:ed|ure|s)?|errors?|fault|corrupt(?:ed|ion)?|uncorrectable|"
    r"\bEIO\b|\berr\b|\bret\b|duplicate|mismatch|does not match|unable|"
    r"couldn'?t|not ready|reject(?:ed|ing)?|too big|"
    r"returned status|lfm_start|timeout|timed out|ETIMEDOUT|"
    r"(?:P_FAIL|E_FAIL)\s*(?:=|:)?\s*1\b|<err>",
    re.IGNORECASE,
)
BAD_BLOCK = re.compile(r"bad[ _-]?blocks?", re.IGNORECASE)


class HilError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot(root: Path) -> dict[str, dict[str, int]]:
    if not root.is_dir():
        raise HilError(f"MSC drive is not readable: {root}")
    result = {}
    for path in sorted(candidate for candidate in root.iterdir() if candidate.is_file()):
        stat = path.stat()
        result[path.name] = {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}
    return result


def content_snapshot(root: Path) -> dict[str, str]:
    if not root.is_dir():
        raise HilError(f"MSC drive is not readable: {root}")
    return {path.relative_to(root).as_posix(): sha256(path)
            for path in sorted(candidate for candidate in root.iterdir()
                               if candidate.is_file())}


def require_preserved(root: Path, expected: dict[str, str]) -> dict[str, object]:
    for name, digest in expected.items():
        path = root / name
        if not path.is_file():
            raise HilError(f"prior file was deleted: {name}")
        if sha256(path) != digest:
            raise HilError(f"prior file content changed: {name}")
    return {"result": "PASS", "files": sorted(expected)}


def create_output_directory(output: Path) -> None:
    if output.exists():
        raise HilError(f"output directory already exists: {output}")
    output.mkdir(parents=True)


def write_summary(output: Path, summary: dict[str, object], created: bool) -> None:
    if created:
        (output / "summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8")


def wait_for_snapshot(root: Path, timeout: float, settle: float):
    deadline = time.monotonic() + timeout
    previous = None
    stable_since = None
    while time.monotonic() < deadline:
        try:
            current = snapshot(root)
            if current == previous:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= settle:
                    return current
            else:
                previous = current
                stable_since = time.monotonic()
        except (HilError, OSError):
            previous = stable_since = None
        time.sleep(0.5)
    raise HilError("MSC did not return with a stable readable directory")


def new_recording_files(before, after) -> list[str]:
    return sorted(name for name in after if name not in before and RECORDING_NAME.fullmatch(name))


def scan_storage_errors(text: str) -> dict[str, object]:
    failures, bad_blocks = [], []
    records = re.split(r"\r?\n|(?=\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\])", text)
    for line in records:
        if BAD_BLOCK.search(line):
            bad_blocks.append(line)
            continue
        counters = re.findall(r"(?:duplicates?|fails?|errors?|corrections?)\s+(-?\d+)",
                              line, re.IGNORECASE)
        if counters and "tot " in line.lower() and all(int(value) == 0 for value in counters):
            continue
        evaluated = re.sub(r"(?:P_FAIL|E_FAIL)\s*(?:=|:)?\s*0\b", "", line,
                           flags=re.IGNORECASE)
        if STORAGE_CONTEXT.search(line) and ERROR_WORD.search(evaluated):
            failures.append(line)
    if failures:
        raise HilError("NAND/storage errors reported: " + " | ".join(failures))
    return {"result": "PASS", "bad_block_diagnostics": bad_blocks}


def revalidate_evidence(paths: list[Path], summary_path: Path | None) -> int:
    summary = {"test": "PPG storage-log evidence revalidation", "result": "PASS",
               "files": []}
    for path in paths:
        entry = {"file": str(path), "result": "FAIL"}
        text = ""
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
            entry.update(scan_storage_errors(text))
        except Exception as error:
            entry["error"] = str(error)
            if text:
                entry["bad_block_diagnostics"] = [line for line in text.splitlines()
                                                   if BAD_BLOCK.search(line)]
            summary["result"] = "FAIL"
        summary["files"].append(entry)
    if summary_path:
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["result"] == "PASS" else 1


def collect_preflight(paths: list[Path] | None, destination: Path) -> dict[str, object]:
    if not paths:
        return {"result": "NOT_PROVIDED",
                "observability": "no complete native boot/preflight log supplied"}
    destination.mkdir()
    files = []
    for index, source in enumerate(paths, 1):
        if not source.is_file():
            raise HilError(f"preflight log is absent: {source}")
        target = destination / f"{index:02d}-{source.name}"
        shutil.copy2(source, target)
        scan = scan_storage_errors(target.read_text(encoding="utf-8", errors="replace"))
        files.append({"source": str(source), "evidence": str(target),
                      "sha256": sha256(target), "storage_scan": scan})
    return {"result": "PASS", "files": files}


def require_ordered_markers(text: str, repetitions: int) -> None:
    position = 0
    for _ in range(repetitions):
        for marker in SHUTDOWN_MARKERS:
            position = text.find(marker, position)
            if position < 0:
                raise HilError("shutdown markers are missing or out of order")
            position += len(marker)


def load_module(path: Path, name: str):
    if not path.is_file():
        raise HilError(f"required tool is absent: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise HilError(f"cannot load tool: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_process(command: list[str], prefix: Path, label: str) -> dict[str, object]:
    completed = subprocess.run(command, text=True, capture_output=True)
    prefix.with_suffix(".stdout.txt").write_text(completed.stdout, encoding="utf-8")
    prefix.with_suffix(".stderr.txt").write_text(completed.stderr, encoding="utf-8")
    if completed.returncode != 0:
        raise HilError(f"{label} failed with code {completed.returncode}")
    return {"result": "PASS", "exit_code": completed.returncode,
            "stdout": str(prefix.with_suffix('.stdout.txt')),
            "stderr": str(prefix.with_suffix('.stderr.txt'))}


def run_argv_file(path: Path, prefix: Path, label: str) -> dict[str, object]:
    command = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(command, list) or not command or not all(isinstance(x, str) for x in command):
        raise HilError(f"{label} argv JSON must be a non-empty array of strings")
    result = run_process(command, prefix, label)
    result["argv_json"] = str(path)
    result["storage_scan"] = scan_storage_errors("\n".join([
        Path(result["stdout"]).read_text(encoding="utf-8", errors="replace"),
        Path(result["stderr"]).read_text(encoding="utf-8", errors="replace"),
    ]))
    return result


def run_central_format(args) -> dict[str, object]:
    try:
        import serial
    except ImportError as error:
        raise HilError(f"pyserial is required for Central format control: {error}") from error

    transcript_path = args.output / "format-central.txt"
    received = ""

    with transcript_path.open("w", encoding="utf-8", newline="") as transcript, \
            serial.Serial(args.command_port, 115200, timeout=0.1, write_timeout=2,
                          rtscts=False, dsrdtr=False) as uart:
        uart.dtr = True
        uart.reset_input_buffer()

        def send(command: str) -> int:
            nonlocal received
            mark = len(received)
            transcript.write(f">>> {command}\n")
            transcript.flush()
            uart.write((command + "\r\n").encode("ascii"))
            uart.flush()
            return mark

        def wait_for(mark: int, label: str, predicate, timeout: float) -> str:
            nonlocal received
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                payload = uart.read(uart.in_waiting or 1)
                if payload:
                    decoded = payload.decode("utf-8", errors="replace")
                    received += decoded
                    transcript.write(decoded)
                    transcript.flush()
                    current = received[mark:]
                    if "RESET_RECONNECT_TIMEOUT" in current or \
                            "RESET_DISCONNECT_TIMEOUT" in current:
                        raise HilError("Central reported reset timeout")
                    if predicate(current):
                        return current
            raise HilError(f"timed out waiting for Central {label}")

        mark = send("help")
        wait_for(mark, "reset-68 capability",
                 lambda text: "reset 68|120|121|132" in text, 8.0)
        mark = send("disconnect")
        wait_for(mark, "initial disconnect",
                 lambda text: "DISCONNECTED" in text or "ERR no active connection" in text,
                 15.0)
        mark = send("connect ppg")
        ready = wait_for(mark, "PPG reset-characteristic discovery",
                         lambda text: "LEGACY_RESET_READY" in text and
                         "PEER_READY" in text and
                         "peer_name=MSense4PPG-" in text, args.format_timeout)
        if "LEGACY_RESET_UNAVAILABLE" in ready or re.search(r"\b(?:ERR|ERROR)\b", ready):
            raise HilError("Central PPG discovery reported an error")
        mark = send("reset 68")
        formatted = wait_for(mark, "format/reboot rediscovery",
                             lambda text: "RESET_DISCONNECTED code=68" in text and
                             "RESET_REDISCOVERED code=68" in text,
                             args.format_timeout)
        if "ERR reset 68" in formatted:
            raise HilError("Central rejected reset 68")

    return {"result": "PASS", "target": "ppg", "code": 68,
            "transcript": str(transcript_path),
            "storage_scan": scan_storage_errors(received)}


def start_native_capture(args):
    command = [
        sys.executable, str(args.capture_tool), "--output", str(args.output / "native-uart"),
        "--port", args.ppg_port, "--usb-serial", args.ppg_usb_serial,
        "--baud", str(args.ppg_baud), "--duration", str(args.native_capture_seconds),
    ]
    if args.allow_port_change:
        command.append("--allow-port-change")
    stdout = (args.output / "native-capture.stdout.txt").open("w", encoding="utf-8")
    stderr = (args.output / "native-capture.stderr.txt").open("w", encoding="utf-8")
    process = subprocess.Popen(command, stdout=stdout, stderr=stderr, text=True)
    time.sleep(args.capture_start_delay)
    if process.poll() is not None:
        stdout.close()
        stderr.close()
        raise HilError(f"native UART capture exited early ({process.returncode})")
    return process, stdout, stderr


def finish_native_capture(process, stdout, stderr, args):
    try:
        process.wait(timeout=args.native_capture_seconds + 5)
    except subprocess.TimeoutExpired as error:
        process.terminate()
        process.wait(timeout=5)
        raise HilError("native UART capture did not finish") from error
    finally:
        stdout.close()
        stderr.close()
    if process.returncode != 0:
        raise HilError(f"native UART capture failed with code {process.returncode}")
    metadata = json.loads((args.output / "native-uart.meta.json").read_text(encoding="utf-8"))
    if metadata.get("reason") != "duration":
        raise HilError(f"native UART capture ended unexpectedly ({metadata.get('reason')})")
    text = (args.output / "native-uart.txt").read_text(encoding="utf-8", errors="replace")
    scan = scan_storage_errors(text)
    require_ordered_markers(text, 2)
    return {"capture": metadata, "storage_scan": scan,
            "shutdown_marker_sequences": 2}


def stop_native_capture(process, stdout, stderr) -> None:
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
    if stdout is not None and not stdout.closed:
        stdout.close()
    if stderr is not None and not stderr.closed:
        stderr.close()


def file_prefix(raw: bytes, record_bytes: int, name: str) -> bytes:
    last_data = next((index for index in range(len(raw) - 1, -1, -1)
                      if raw[index] != 0xFF), -1)
    if last_data < 0:
        raise HilError(f"{name}: file contains no data")
    end = ((last_data + 1 + record_bytes - 1) // record_bytes) * record_bytes
    if end > len(raw) or raw[end:] != b"\xff" * (len(raw) - end):
        raise HilError(f"{name}: invalid record boundary or erased suffix")
    return raw[:end]


def validate_recording_file(stream_module, path: Path) -> dict[str, object]:
    raw = path.read_bytes()
    if PPG_NAME.fullmatch(path.name):
        record_bytes, kind = RECORD_BYTES, "ppg"
    elif ACCEL_NAME.fullmatch(path.name):
        record_bytes, kind = 26, "accel"
    elif LOG_NAME.fullmatch(path.name):
        record_bytes, kind = 1, "log"
    else:
        raise HilError(f"unexpected recording filename: {path.name}")
    if kind == "log":
        if not raw or len(raw) >= FILE_BYTES:
            raise HilError(f"{path.name}: expected a nonempty exact-length log below {FILE_BYTES} bytes")
        if raw.endswith(b"\xff"):
            raise HilError(f"{path.name}: exposed erased 0xFF tail")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as error:
            raise HilError(f"{path.name}: log is not UTF-8 text") from error
        return {"file": str(path), "kind": kind, "valid_bytes": len(raw),
                "records": len(raw), "termination": "exact EOF",
                "sha256": sha256(path), "text": text}
    if len(raw) != FILE_BYTES:
        raise HilError(f"{path.name}: size {len(raw)}, expected {FILE_BYTES}")
    prefix = file_prefix(raw, record_bytes, path.name)
    if len(prefix) == FILE_BYTES:
        raise HilError(f"{path.name}: short session left no erased suffix")
    if record_bytes > 1 and any(
            prefix[offset:offset + record_bytes] == b"\xff" * record_bytes
            for offset in range(0, len(prefix), record_bytes)):
        raise HilError(f"{path.name}: erased record appears inside data prefix")
    result = {"file": str(path), "kind": kind, "valid_bytes": len(prefix),
              "records": len(prefix) // record_bytes,
              "termination": "0xFF suffix",
              "sha256": sha256(path)}
    if kind == "ppg":
        try:
            result.update(stream_module.validate_ppg_records(prefix))
        except Exception as error:
            raise HilError(f"{path.name}: PPG record validation failed: {error}") from error
    return result


def wait_for_session_files(args, before):
    deadline = time.monotonic() + args.remount_timeout
    stable_since = None
    previous = None
    last = "MSC unavailable"
    while time.monotonic() < deadline:
        try:
            current = snapshot(args.drive)
            names = new_recording_files(before, current)
            signature = [(name, current[name]["size"], current[name]["mtime_ns"])
                         for name in names]
            kinds = {"ppg" if PPG_NAME.fullmatch(name) else
                     "accel" if ACCEL_NAME.fullmatch(name) else "log"
                     for name in names}
            if kinds == {"ppg", "accel", "log"}:
                if signature == previous:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= args.settle_seconds:
                        return current, names
                else:
                    previous, stable_since = signature, time.monotonic()
            last = "new PPG/accelerometer/log set is incomplete"
        except (HilError, OSError) as error:
            last = str(error)
        time.sleep(0.5)
    raise HilError(f"PPG MSC remount/file settle timed out: {last}")


def run_stream_session(args, index: int, session_id: int, before, prior_hashes,
                       stream_module):
    root = args.output / f"session-{index}"
    root.mkdir()
    stream_dir = root / "stream"
    command = [
        sys.executable, str(args.stream_tool), "--command-port", args.command_port,
        "--relay-port", args.relay_port, "--session-id", str(session_id),
        "--stream-timeout", str(args.stream_timeout), "--output-dir", str(stream_dir),
    ]
    process = run_process(command, root / "stream-process", f"PPG session {index}")
    stream_summary = json.loads((stream_dir / "summary.json").read_text(encoding="utf-8"))
    if not stream_summary.get("passed") or stream_summary.get("sensor_bytes") != 131072 or \
            stream_summary.get("complete_records") != 8192:
        raise HilError(f"PPG session {index} finite stream summary is incomplete")
    central_text = "\n".join([
        (stream_dir / "command.decoded.txt").read_text(encoding="utf-8", errors="replace"),
        Path(process["stdout"]).read_text(encoding="utf-8", errors="replace"),
        Path(process["stderr"]).read_text(encoding="utf-8", errors="replace"),
    ])
    central_scan = scan_storage_errors(central_text)
    after, names = wait_for_session_files(args, before)
    preservation = require_preserved(args.drive, prior_hashes)
    current_hashes = content_snapshot(args.drive)
    (root / "post.json").write_text(json.dumps(after, indent=2) + "\n", encoding="utf-8")
    copied = []
    for name in names:
        target = root / "new-files" / name
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(args.drive / name, target)
        copied.append(target)
    validation = [validate_recording_file(stream_module, path) for path in copied]
    log_text = "\n".join(entry["text"] for entry in validation if entry["kind"] == "log")
    positions = [log_text.find(marker) for marker in SHUTDOWN_MARKERS]
    if any(position < 0 for position in positions) or positions != sorted(positions):
        raise HilError(f"PPG session {index} log shutdown markers are missing or out of order")
    for entry in validation:
        entry.pop("text", None)
    return after, current_hashes, {
                   "session": index, "session_id": session_id, "result": "PASS",
                   "stream": stream_summary, "central_storage_scan": central_scan,
                   "new_files": names, "prior_files": preservation,
                   "shutdown_markers": "ordered",
                   "on_media": validation}


def verify_persistence(args, sessions, prior_hashes):
    run_argv_file(args.reset_argv_json, args.output / "reset", "reset")
    wait_for_snapshot(args.drive, args.remount_timeout, args.settle_seconds)
    preservation = require_preserved(args.drive, prior_hashes)
    checked = []
    for session in sessions:
        for name in session["new_files"]:
            saved = args.output / f"session-{session['session']}" / "new-files" / name
            live = args.drive / name
            if not live.is_file() or sha256(saved) != sha256(live):
                raise HilError(f"reset persistence mismatch: {name}")
            checked.append(name)
    return {"result": "PASS", "files": checked, "prior_files": preservation}


def self_test() -> None:
    with tempfile.TemporaryDirectory(prefix="ppg-production-hil-") as directory:
        root = Path(directory)
        before = snapshot(root)
        (root / "ppg100.bin").write_bytes(b"x")
        (root / "ac100.bin").write_bytes(b"x")
        (root / "log1.txt").write_bytes(b"x")
        after = snapshot(root)
        assert new_recording_files(before, after) == ["ac100.bin", "log1.txt", "ppg100.bin"]
        assert PPG_NAME.fullmatch("7ppg100.bin")
        assert ACCEL_NAME.fullmatch("7ac100_0001.bin")
        class Validator:
            @staticmethod
            def validate_ppg_records(sensor):
                assert len(sensor) == RECORD_BYTES
                return {"tick_first": 1, "tick_last": 1}

        media = root / "ppg101.bin"
        media.write_bytes(b"\0" * 12 + b"\1\0\0\0" +
                          b"\xff" * (FILE_BYTES - RECORD_BYTES))
        checked = validate_recording_file(Validator, media)
        assert checked["valid_bytes"] == RECORD_BYTES
        accel = root / "ac101.bin"
        accel.write_bytes(bytes(range(26)) + b"\xff" * (FILE_BYTES - 26))
        assert validate_recording_file(Validator, accel)["records"] == 1
        log = root / "log2.txt"
        log_bytes = "\n".join(SHUTDOWN_MARKERS).encode()
        log.write_bytes(log_bytes)
        checked = validate_recording_file(Validator, log)
        assert checked["valid_bytes"] == len(log_bytes)
        assert checked["termination"] == "exact EOF"
        for invalid_log in (b"", log_bytes + b"\xff", b"x" * FILE_BYTES):
            log.write_bytes(invalid_log)
            try:
                validate_recording_file(Validator, log)
            except HilError:
                pass
            else:
                raise AssertionError("invalid exact-length log was accepted")
        require_ordered_markers("\n".join(SHUTDOWN_MARKERS * 2), 2)
        assert scan_storage_errors("nand_disk: tot duplicates 0, tot verify fails 0, tot ECC errors 0")["result"] == "PASS"
        assert scan_storage_errors("bad-block diagnostic 4")["bad_block_diagnostics"]
        assert scan_storage_errors("<err> bq274xx: Failed to write into control register")["result"] == "PASS"
        assert scan_storage_errors("NUS stream disconnected: reason 0x13")["result"] == "PASS"
        for failure in ("NAND program failed: EIO",
                        "<err> spi_nand: NAND ECC uncorrectable",
                        "<wrn> spi_nand: page write returned status 8",
                        "<wrn> spi_nand: err block erase 3: -5",
                        "nand_disk initialization failed: -5",
                        "fatfs mount timed out: -110 ETIMEDOUT",
                        "spi4 controller fault while accessing storage bus",
                        "spi_nand status E_FAIL: 1",
                        "spi_nor: JEDEC ID read failed: -5"):
            try:
                scan_storage_errors(failure)
            except HilError:
                pass
            else:
                raise AssertionError("storage error was accepted")
        assert scan_storage_errors("spi_nand status P_FAIL=0 E_FAIL=0")["result"] == "PASS"

        preserved = content_snapshot(root)
        victim = root / "ppg100.bin"
        original = victim.read_bytes()
        victim.unlink()
        try:
            require_preserved(root, preserved)
        except HilError:
            pass
        else:
            raise AssertionError("deleted prior file was accepted")
        victim.write_bytes(original)
        victim.write_bytes(b"y" * len(original))
        try:
            require_preserved(root, preserved)
        except HilError:
            pass
        else:
            raise AssertionError("same-size prior-file corruption was accepted")
        victim.write_bytes(original)
        assert require_preserved(root, preserved)["result"] == "PASS"

        existing = root / "existing-output"
        existing.mkdir()
        (existing / "summary.json").write_text("sentinel", encoding="utf-8")
        (existing / "evidence.bin").write_bytes(b"evidence")
        output_before = content_snapshot(existing)
        created = False
        try:
            create_output_directory(existing)
        except HilError:
            pass
        else:
            raise AssertionError("existing output directory was accepted")
        finally:
            write_summary(existing, {"result": "FAIL"}, created)
        assert content_snapshot(existing) == output_before

        boot = root / "boot.txt"
        boot.write_text("spi_nand status P_FAIL=0 E_FAIL=0", encoding="utf-8")
        preflight = collect_preflight([boot], root / "preflight")
        assert preflight["result"] == "PASS" and preflight["files"][0]["sha256"]
    print("run_production_hil.py self-test: PASS (no hardware access)")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-test", help="run offline checks")
    scan = subparsers.add_parser("scan-evidence", help="re-scan preserved text evidence")
    scan.add_argument("--input", type=Path, nargs="+", required=True)
    scan.add_argument("--summary", type=Path)
    run = subparsers.add_parser("run", help="run the destructive two-session campaign")
    for name in ("dfu-tool", "stream-tool", "capture-tool", "image", "drive", "output"):
        run.add_argument(f"--{name}", type=Path, required=True)
    run.add_argument("--command-port", required=True)
    run.add_argument("--relay-port", required=True)
    run.add_argument("--central-jlink-serial", required=True)
    run.add_argument("--ppg-port", required=True)
    run.add_argument("--ppg-usb-serial", required=True)
    run.add_argument("--session-ids", type=int, nargs=2, required=True)
    run.add_argument("--dfu-transaction", type=lambda value: int(value, 0), required=True)
    run.add_argument("--dfu-timeout", type=float, default=90.0)
    run.add_argument("--format-timeout", type=float, default=330.0)
    run.add_argument("--format-confirmation", required=True,
                     choices=("FORMAT_PPG_FATFS_CODE_68",))
    run.add_argument("--stream-timeout", type=float, default=240.0)
    run.add_argument("--native-capture-seconds", type=float, default=120.0)
    run.add_argument("--capture-start-delay", type=float, default=1.0)
    run.add_argument("--remount-timeout", type=float, default=60.0)
    run.add_argument("--settle-seconds", type=float, default=2.0)
    run.add_argument("--ppg-baud", type=int, default=115200)
    run.add_argument("--allow-port-change", action="store_true")
    run.add_argument("--reset-argv-json", type=Path)
    run.add_argument("--preflight-log", type=Path, nargs="+",
                     help="complete native boot/preflight logs to preserve and scan")
    args = parser.parse_args(argv)
    if args.command == "run":
        if len(set(args.session_ids)) != 2 or any(not 0 < value <= 0xFFFFFFFF
                                                  for value in args.session_ids):
            parser.error("--session-ids must be two distinct nonzero uint32 values")
        if not 0 < args.dfu_transaction <= 0xFFFFFFFF:
            parser.error("--dfu-transaction must be a nonzero uint32")
        if any(value <= 0 for value in (args.dfu_timeout, args.format_timeout,
                                        args.stream_timeout,
                                        args.native_capture_seconds, args.capture_start_delay,
                                        args.remount_timeout, args.settle_seconds)):
            parser.error("timeouts and durations must be positive")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.command == "self-test":
        self_test()
        return 0
    if args.command == "scan-evidence":
        return revalidate_evidence(args.input, args.summary)
    summary = {"test": "PPG production NAND functional HIL", "result": "FAIL",
               "started_utc": utc_now(), "image": str(args.image),
               "central": {"serial": args.central_jlink_serial,
                           "command_port": args.command_port, "relay_port": args.relay_port},
               "ppg": {"usb_serial": args.ppg_usb_serial, "port": args.ppg_port},
               "drive": str(args.drive), "sessions": []}
    native_process = native_stdout = native_stderr = None
    output_created = False
    try:
        create_output_directory(args.output)
        output_created = True
        for path in (args.dfu_tool, args.stream_tool, args.capture_tool, args.image):
            if not path.is_file():
                raise HilError(f"required input is absent: {path}")
        summary["image_sha256"] = sha256(args.image)
        stream_module = load_module(args.stream_tool, "ppg_stream_validator")
        summary["format"] = run_central_format(args)
        dfu_command = [
            sys.executable, str(args.dfu_tool), "--image", str(args.image), "--target", "ppg",
            "--command-port", args.command_port, "--data-port", args.relay_port,
            "--jlink-serial", args.central_jlink_serial,
            "--transaction", str(args.dfu_transaction), "--timeout", str(args.dfu_timeout),
            "--json",
        ]
        summary["dfu"] = run_process(dfu_command, args.output / "dfu", "PPG DFU")
        summary["dfu"]["storage_scan"] = scan_storage_errors("\n".join([
            Path(summary["dfu"]["stdout"]).read_text(encoding="utf-8", errors="replace"),
            Path(summary["dfu"]["stderr"]).read_text(encoding="utf-8", errors="replace"),
        ]))
        baseline = wait_for_snapshot(args.drive, args.remount_timeout, args.settle_seconds)
        stale = [name for name in baseline if RECORDING_NAME.fullmatch(name)]
        if stale:
            raise HilError("post-format baseline still contains recordings: " +
                           ", ".join(stale))
        (args.output / "baseline.json").write_text(
            json.dumps(baseline, indent=2) + "\n", encoding="utf-8")
        summary["preflight"] = collect_preflight(args.preflight_log,
                                                  args.output / "preflight")
        prior_hashes = content_snapshot(args.drive)
        native_process, native_stdout, native_stderr = start_native_capture(args)
        for index, session_id in enumerate(args.session_ids, 1):
            baseline, prior_hashes, session = run_stream_session(
                args, index, session_id, baseline, prior_hashes, stream_module)
            summary["sessions"].append(session)
            if native_process.poll() is not None:
                raise HilError("native UART capture ended before both sessions completed")
        summary["native_uart"] = finish_native_capture(
            native_process, native_stdout, native_stderr, args)
        native_process = native_stdout = native_stderr = None
        if args.reset_argv_json:
            summary["reset_persistence"] = verify_persistence(
                args, summary["sessions"], prior_hashes)
        summary["result"] = "PASS"
        code = 0
    except Exception as error:
        summary["error"] = str(error)
        code = 1
    finally:
        stop_native_capture(native_process, native_stdout, native_stderr)
        summary["ended_utc"] = utc_now()
        write_summary(args.output, summary, output_created)
    print(json.dumps(summary, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
