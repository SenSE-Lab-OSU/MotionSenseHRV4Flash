#!/usr/bin/env python3
"""Run two production ECG collection sessions and validate their new files."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path


SHUTDOWN_MARKERS = (
    "Leaving ECG collection mode",
    "Closing storage log",
    "Storage log end",
)
ECG_NAME = re.compile(r"^(?:\d+)?ecg\d+_\d{4}\.bin$", re.IGNORECASE)
ACCEL_NAME = re.compile(r"^(?:\d+)?ac\d+_\d{4}\.bin$", re.IGNORECASE)
LOG_NAME = re.compile(r"^(?:\d+)?log\d+\.txt$", re.IGNORECASE)
FILE_BYTES = 4 * 1024 * 1024
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
        result[path.relative_to(root).as_posix()] = {
            "size": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
        }
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


def wait_for_snapshot(root: Path, timeout: float, settle: float) -> dict[str, dict[str, int]]:
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
    raise HilError("MSC did not return with a stable readable directory after format")


def is_recording_name(name: str) -> bool:
    base = Path(name).name
    return bool(ECG_NAME.fullmatch(base) or ACCEL_NAME.fullmatch(base) or
                LOG_NAME.fullmatch(base))


def new_recordings(before: dict[str, dict[str, int]],
                   after: dict[str, dict[str, int]]) -> list[str]:
    return sorted(name for name in after if name not in before and is_recording_name(name))


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
        raise HilError("UART reported NAND/storage errors: " + " | ".join(failures))
    return {"result": "PASS", "bad_block_diagnostics": bad_blocks}


def revalidate_evidence(paths: list[Path], summary_path: Path | None) -> int:
    summary = {"test": "ECG storage-log evidence revalidation", "result": "PASS",
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


def run_argv_file(path: Path, label: str, output: Path) -> dict[str, object]:
    command = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(command, list) or not command or not all(isinstance(x, str) for x in command):
        raise HilError(f"{label} argv JSON must be a non-empty array of strings")
    completed = subprocess.run(command, text=True, capture_output=True)
    output.with_suffix(".stdout.txt").write_text(completed.stdout, encoding="utf-8")
    output.with_suffix(".stderr.txt").write_text(completed.stderr, encoding="utf-8")
    if completed.returncode != 0:
        raise HilError(f"{label} command failed with code {completed.returncode}")
    return {"result": "PASS", "argv_json": str(path), "exit_code": completed.returncode,
            "storage_scan": scan_storage_errors(completed.stdout + "\n" + completed.stderr)}


def load_module(path: Path, name: str):
    if not path.is_file():
        raise HilError(f"required tool is absent: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise HilError(f"cannot load tool: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def press_button(button, action: str, probe: str, elf: Path,
                 nm: Path | None, nrfutil: Path | None) -> dict[str, object]:
    button.PROBE_SERIAL = probe
    button.ELF = elf
    if nm is not None:
        button.NM_CANDIDATES = (nm,)
    if nrfutil is not None:
        button.nrfutil = lambda: str(nrfutil)
    output = io.StringIO()
    try:
        with contextlib.redirect_stdout(output):
            button.release()
    except Exception as error:
        raise HilError(f"{action} button injection failed: {error}") from error
    try:
        detail = json.loads(output.getvalue())
    except json.JSONDecodeError as error:
        raise HilError(f"{action} button tool emitted invalid JSON") from error
    detail["action"] = action
    detail["utc"] = utc_now()
    return detail


def start_capture(args, session_dir: Path) -> tuple[subprocess.Popen, object, object]:
    prefix = session_dir / "uart"
    command = [
        sys.executable, str(args.capture_tool), "--output", str(prefix),
        "--port", args.port, "--usb-serial", args.usb_serial,
        "--baud", str(args.baud), "--duration", str(args.capture_timeout),
    ]
    if args.allow_port_change:
        command.append("--allow-port-change")
    stdout = (session_dir / "capture.stdout.txt").open("w", encoding="utf-8")
    stderr = (session_dir / "capture.stderr.txt").open("w", encoding="utf-8")
    process = subprocess.Popen(command, stdout=stdout, stderr=stderr, text=True)
    time.sleep(args.capture_start_delay)
    if process.poll() is not None:
        stdout.close()
        stderr.close()
        raise HilError(f"UART capture exited before collection (code {process.returncode})")
    return process, stdout, stderr


def finish_capture(process: subprocess.Popen, stdout, stderr,
                   session_dir: Path, timeout: float) -> dict[str, object]:
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        raise HilError("UART capture did not finish") from error
    finally:
        stdout.close()
        stderr.close()
    if process.returncode != 0:
        raise HilError(f"UART capture failed with code {process.returncode}")
    metadata = json.loads((session_dir / "uart.meta.json").read_text(encoding="utf-8"))
    if metadata.get("reason") != "duration":
        raise HilError(f"UART capture ended unexpectedly ({metadata.get('reason')})")
    uart_text = (session_dir / "uart.txt").read_text(encoding="utf-8", errors="replace")
    positions = [uart_text.find(marker) for marker in SHUTDOWN_MARKERS]
    if any(position < 0 for position in positions) or positions != sorted(positions):
        raise HilError("UART shutdown markers are missing or out of order")
    metadata["shutdown_markers"] = "ordered"
    return metadata


def stop_capture(process: subprocess.Popen | None, stdout, stderr) -> None:
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


def wait_for_files(drive: Path, before: dict[str, dict[str, int]],
                   timeout: float, settle: float) -> tuple[dict[str, dict[str, int]], list[str]]:
    deadline = time.monotonic() + timeout
    stable_since = None
    previous = None
    last_error = "MSC drive unavailable"
    while time.monotonic() < deadline:
        try:
            current = snapshot(drive)
            selected = new_recordings(before, current)
            signature = [(name, current[name]["size"], current[name]["mtime_ns"])
                         for name in selected]
            if any(ECG_NAME.fullmatch(Path(name).name) for name in selected) and \
                    any(ACCEL_NAME.fullmatch(Path(name).name) for name in selected) and \
                    any(LOG_NAME.fullmatch(Path(name).name) for name in selected):
                if signature == previous:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= settle:
                        return current, selected
                else:
                    previous = signature
                    stable_since = time.monotonic()
            last_error = "new ECG/AC/log set is incomplete"
        except (HilError, OSError) as error:
            last_error = str(error)
        time.sleep(0.5)
    raise HilError(f"MSC remount/file settle timed out: {last_error}")


def copy_selected(drive: Path, names: list[str], destination: Path) -> list[Path]:
    copied = []
    for name in names:
        source = drive / Path(name)
        target = destination / Path(name)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        copied.append(target)
    return copied


def validate_files(validator, paths: list[Path]) -> dict[str, object]:
    ecg, accel, logs = [], [], []
    try:
        for path in paths:
            if ECG_NAME.fullmatch(path.name):
                ecg.append(validator.validate_ecg(path))
            elif ACCEL_NAME.fullmatch(path.name):
                accel.append(validator.validate_accel(path))
            elif LOG_NAME.fullmatch(path.name):
                logs.append(path)
    except Exception as error:
        raise HilError(f"recording validation failed: {error}") from error
    if not ecg or not accel or not logs:
        raise HilError("copied file set does not contain ECG, accelerometer, and log files")
    if any(int(entry["valid_blocks"]) <= 0 for entry in ecg):
        raise HilError("ECG file has no completed ECB2 block")
    if any(int(entry["samples"]) <= 0 for entry in accel):
        raise HilError("accelerometer file has no ACB1 samples")
    combined = "\n".join(path.read_text(encoding="utf-8", errors="replace")
                           for path in sorted(logs))
    positions = [combined.find(marker) for marker in SHUTDOWN_MARKERS]
    if any(position < 0 for position in positions) or positions != sorted(positions):
        raise HilError("filesystem shutdown markers are missing or out of order")
    log_lengths = []
    for path in logs:
        raw = path.read_bytes()
        if not raw or len(raw) >= FILE_BYTES or b"\xff" in raw:
            raise HilError(f"logger file is not exact-length text: {path}")
        log_lengths.append({"file": str(path), "bytes": len(raw),
                            "exposed_erased_tail": False})
    return {
        "validation": "PASS",
        "ecg": ecg,
        "accel": accel,
        "logs": [{"file": str(path), "sha256": sha256(path)} for path in logs],
        "logger_lengths": log_lengths,
        "artifacts": [{"file": str(path), "bytes": path.stat().st_size,
                       "sha256": sha256(path)} for path in paths],
        "shutdown_markers": "ordered",
    }


def run_session(args, index: int, before, prior_hashes, button, validator, output: Path,
                during_acquisition=None):
    session_dir = output / f"session-{index}"
    session_dir.mkdir(parents=True, exist_ok=False)
    process = stdout = stderr = None
    buttons = []
    collection_started = recovery_stop_required = False
    try:
        process, stdout, stderr = start_capture(args, session_dir)
        buttons.append(press_button(button, "start", args.probe, args.elf,
                                    args.nm, args.nrfutil))
        collection_started = True
        recovery_stop_required = True
        # Deliberately no probe/debug operation in this acquisition interval.
        if during_acquisition:
            during_acquisition()
        else:
            time.sleep(args.acquisition_seconds)
        buttons.append(press_button(button, "stop", args.probe, args.elf,
                                    args.nm, args.nrfutil))
        recovery_stop_required = False
        capture = finish_capture(process, stdout, stderr, session_dir,
                                 args.capture_timeout + 5)
        process = stdout = stderr = None
        uart_scan = scan_storage_errors(
            (session_dir / "uart.txt").read_text(encoding="utf-8", errors="replace"))
        after, selected = wait_for_files(args.drive, before, args.remount_timeout,
                                         args.settle_seconds)
        preservation = require_preserved(args.drive, prior_hashes)
        current_hashes = content_snapshot(args.drive)
        (session_dir / "post.json").write_text(
            json.dumps(after, indent=2) + "\n", encoding="utf-8")
        copied = copy_selected(args.drive, selected, session_dir / "new-files")
        validation = validate_files(validator, copied)
        for path in copied:
            relative = path.relative_to(session_dir / "new-files").as_posix()
            if sha256(path) != current_hashes[relative]:
                raise HilError(f"copied evidence does not match on-media file: {relative}")
        return after, current_hashes, {
            "session": index, "result": "PASS", "buttons": buttons,
            "uart": capture, "uart_storage_scan": uart_scan,
            "new_files": selected, "prior_files": preservation,
            "validation": validation,
        }
    finally:
        if collection_started and recovery_stop_required:
            try:
                recovery = press_button(button, "failure-stop", args.probe, args.elf,
                                        args.nm, args.nrfutil)
            except Exception as error:
                recovery = {"action": "failure-stop", "result": "FAIL", "error": str(error)}
            (session_dir / "failure-stop.json").write_text(
                json.dumps(recovery, indent=2) + "\n", encoding="utf-8")
            time.sleep(min(10, args.post_capture_seconds
                           if hasattr(args, "post_capture_seconds") else 10))
        stop_capture(process, stdout, stderr)


def reset_and_verify(args, sessions: list[dict[str, object]], prior_hashes) -> dict[str, object]:
    control = run_argv_file(args.reset_argv_json, "reset", args.output / "reset")
    deadline = time.monotonic() + args.remount_timeout
    expected = []
    for session in sessions:
        session_dir = args.output / f"session-{session['session']}" / "new-files"
        expected.extend((session_dir / name, args.drive / name)
                        for name in session["new_files"])
    while time.monotonic() < deadline:
        try:
            preservation = require_preserved(args.drive, prior_hashes)
            for saved, live in expected:
                if not live.is_file() or sha256(live) != sha256(saved):
                    raise OSError(f"persistence mismatch: {live}")
            validation = validate_files(load_module(args.validator_tool, "ecg_validator_reset"),
                                        [live for _, live in expected])
            return {"result": "PASS", "files": len(expected),
                    "control": control, "prior_files": preservation,
                    "validation": validation}
        except (OSError, HilError):
            time.sleep(0.5)
    raise HilError("reset/remount persistence validation timed out")


def self_test() -> None:
    with tempfile.TemporaryDirectory(prefix="ecg-cleanup-hil-") as directory:
        root = Path(directory)
        (root / "uuid.txt").write_text("create once", encoding="utf-8")
        before = snapshot(root)
        (root / "ecg1_0000.bin").write_bytes(b"ECF2")
        (root / "ac1_0000.bin").write_bytes(b"ACF3")
        (root / "log1.txt").write_text("\n".join(SHUTDOWN_MARKERS), encoding="utf-8")
        after = snapshot(root)
        assert new_recordings(before, after) == ["ac1_0000.bin", "ecg1_0000.bin", "log1.txt"]
        assert not is_recording_name("uuid.txt")
        assert is_recording_name("17log42.txt")
        class Validator:
            @staticmethod
            def validate_ecg(_path):
                return {"valid_blocks": 1, "recording_id": 1}

            @staticmethod
            def validate_accel(_path):
                return {"samples": 1}

        checked = validate_files(Validator, [root / name for name in new_recordings(before, after)])
        assert checked["logger_lengths"][0]["bytes"] < FILE_BYTES
        assert scan_storage_errors("nand_disk: tot duplicates 0, tot verify fails 0, tot ECC errors 0")["result"] == "PASS"
        assert scan_storage_errors("bad block diagnostic: block 7")["bad_block_diagnostics"]
        assert scan_storage_errors("<err> bq274xx: Failed to write into control register")["result"] == "PASS"
        assert scan_storage_errors("NUS stream disconnected: reason 0x13")["result"] == "PASS"
        for failure in ("filesystem write failed: -5 EIO",
                        "<err> spi_nand: NAND ECC uncorrectable",
                        "<wrn> spi_nand: page write returned status 8",
                        "<wrn> spi_nand: err block erase 3: -5",
                        "nand_disk initialization failed: -5",
                        "fatfs mount timed out: -110 ETIMEDOUT",
                        "spi4 controller fault while accessing storage bus",
                        "spi_nand status P_FAIL=1",
                        "spi_nor: JEDEC ID read failed: -5"):
            try:
                scan_storage_errors(failure)
            except HilError:
                pass
            else:
                raise AssertionError("storage failure was accepted")
        assert scan_storage_errors("spi_nand status P_FAIL=0 E_FAIL=0")["result"] == "PASS"

        preserved = content_snapshot(root)
        victim = root / "uuid.txt"
        original = victim.read_bytes()
        victim.unlink()
        try:
            require_preserved(root, preserved)
        except HilError:
            pass
        else:
            raise AssertionError("deleted prior file was accepted")
        victim.write_bytes(original)
        victim.write_bytes(b"X" * len(original))
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
    print("run_cleanup_hil.py self-test: PASS (no hardware access)")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-test", help="run offline helper checks")
    scan = subparsers.add_parser("scan-evidence", help="re-scan preserved text evidence")
    scan.add_argument("--input", type=Path, nargs="+", required=True)
    scan.add_argument("--summary", type=Path)
    run = subparsers.add_parser("run", help="run two ECG collection sessions")
    run.add_argument("--tools-dir", type=Path, required=True)
    run.add_argument("--probe", required=True)
    run.add_argument("--elf", type=Path, required=True)
    run.add_argument("--port", required=True)
    run.add_argument("--usb-serial", required=True)
    run.add_argument("--drive", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--nm", type=Path)
    run.add_argument("--nrfutil", type=Path)
    run.add_argument("--baud", type=int, default=115200)
    run.add_argument("--acquisition-seconds", type=float, default=30.0)
    run.add_argument("--capture-timeout", type=float, default=90.0)
    run.add_argument("--capture-start-delay", type=float, default=1.0)
    run.add_argument("--remount-timeout", type=float, default=45.0)
    run.add_argument("--settle-seconds", type=float, default=2.0)
    wake = run.add_mutually_exclusive_group(required=True)
    wake.add_argument("--wake-first", action="store_true")
    wake.add_argument("--already-awake", action="store_true")
    run.add_argument("--wake-delay", type=float, default=5.0)
    run.add_argument("--allow-port-change", action="store_true")
    run.add_argument("--reset-argv-json", type=Path,
                     help="optional JSON argv array for reset/remount persistence check")
    run.add_argument("--preflight-log", type=Path, nargs="+",
                     help="complete native boot/preflight logs to preserve and scan")
    clean = run.add_mutually_exclusive_group(required=True)
    clean.add_argument("--format-argv-json", type=Path,
                       help="JSON argv array for an explicitly reviewed normal FatFS format command")
    clean.add_argument("--manual-format-confirmation",
                       choices=("I_FORMATTED_ECG_FATFS",),
                       help="confirm a separately completed normal FatFS format")
    args = parser.parse_args(argv)
    if args.command == "run":
        args.capture_tool = args.tools_dir / "capture_uart.py"
        args.button_tool = args.tools_dir / "short_button.py"
        args.validator_tool = args.tools_dir / "validate_new_files.py"
        for value in (args.acquisition_seconds, args.capture_timeout,
                      args.capture_start_delay, args.remount_timeout,
                      args.settle_seconds, args.wake_delay):
            if value <= 0:
                parser.error("all timeout/duration values must be positive")
        if args.capture_timeout <= args.acquisition_seconds:
            parser.error("--capture-timeout must extend beyond --acquisition-seconds")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.command == "self-test":
        self_test()
        return 0
    if args.command == "scan-evidence":
        return revalidate_evidence(args.input, args.summary)
    summary = {
        "test": "ECG legacy-filesystem cleanup HIL", "started_utc": utc_now(),
        "result": "FAIL", "probe": args.probe, "elf": str(args.elf),
        "usb_serial": args.usb_serial, "port": args.port, "drive": str(args.drive),
        "sessions": [],
    }
    output_created = False
    try:
        create_output_directory(args.output)
        output_created = True
        if not args.elf.is_file():
            raise HilError(f"ELF is absent: {args.elf}")
        summary["elf_sha256"] = sha256(args.elf)
        button = load_module(args.button_tool, "ecg_short_button")
        validator = load_module(args.validator_tool, "ecg_file_validator")
        if args.wake_first:
            summary["wake"] = press_button(button, "wake", args.probe, args.elf,
                                             args.nm, args.nrfutil)
            time.sleep(args.wake_delay)
        if args.format_argv_json:
            summary["format"] = run_argv_file(args.format_argv_json, "format",
                                               args.output / "format")
        else:
            summary["format"] = {"result": "PASS", "mode": "manual",
                                 "confirmation": args.manual_format_confirmation}
        baseline = wait_for_snapshot(args.drive, args.remount_timeout,
                                     args.settle_seconds)
        stale = [name for name in baseline if is_recording_name(name)]
        if stale:
            raise HilError("post-format baseline still contains recordings: " +
                           ", ".join(stale))
        (args.output / "baseline.json").write_text(
            json.dumps(baseline, indent=2) + "\n", encoding="utf-8")
        summary["preflight"] = collect_preflight(args.preflight_log,
                                                  args.output / "preflight")
        prior_hashes = content_snapshot(args.drive)
        recording_ids = []
        for index in (1, 2):
            baseline, prior_hashes, session = run_session(
                args, index, baseline, prior_hashes, button, validator, args.output)
            summary["sessions"].append(session)
            session_ids = {entry["recording_id"]
                           for entry in session["validation"]["ecg"]}
            if len(session_ids) != 1:
                raise HilError(f"session {index} has inconsistent ECG recording IDs")
            recording_ids.append(next(iter(session_ids)))
        if len(set(recording_ids)) != 2:
            raise HilError("two sessions did not produce two unique ECG recording IDs")
        if args.reset_argv_json:
            summary["reset_persistence"] = reset_and_verify(
                args, summary["sessions"], prior_hashes)
        summary["result"] = "PASS"
        return_code = 0
    except Exception as error:
        summary["error"] = str(error)
        return_code = 1
    finally:
        summary["ended_utc"] = utc_now()
        write_summary(args.output, summary, output_created)
    print(json.dumps(summary, indent=2))
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
