"""Run spaced ECG FINITE streams through central_nus_test for several hours."""

import argparse
import datetime
import json
import pathlib
import random
import re
import subprocess
import sys
import threading
import time

import serial


TARGET_NAME = "MSense4ECG-EX4BT"
TARGET_TERMINAL = re.compile(r"START_RESULT status=NOT_RECORDING|STREAM_END status=NOT_RECORDING")
ANY_TERMINAL = re.compile(r"STREAM_OK|STREAM_END|START_RESULT|PROTOCOL_ERROR")


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat(timespec="milliseconds")


class RunLog:
    def __init__(self, root):
        self.root = root
        self.root.mkdir(parents=True, exist_ok=False)
        self.events = (root / "events.jsonl").open("a", encoding="utf-8", buffering=1)
        self.control = (root / "central-control.log").open("a", encoding="utf-8", buffering=1)
        self.ecg = (root / "ecg-console.log").open("a", encoding="utf-8", buffering=1)
        self.observations = root.parent / "ECG_LONG_RUN_OBSERVATIONS.md"

    def event(self, kind, **fields):
        record = {"time": stamp(), "kind": kind, **fields}
        self.events.write(json.dumps(record, sort_keys=True) + "\n")
        print(json.dumps(record, sort_keys=True), flush=True)

    def observation(self, title, detail):
        first = not self.observations.exists()
        with self.observations.open("a", encoding="utf-8") as output:
            if first:
                output.write("# ECG Long-Run Unexpected Observations\n\n")
            output.write(f"## {stamp()} — {title}\n\n{detail}\n\n")

    def close(self):
        self.events.close()
        self.control.close()
        self.ecg.close()


class TextPort:
    def __init__(self, port, baud, output):
        self.serial = serial.Serial(port, baud, timeout=0.05, write_timeout=2,
                                    rtscts=False, dsrdtr=False)
        self.serial.dtr = True
        self.output = output
        self.buffer = ""

    def read(self):
        data = self.serial.read(self.serial.in_waiting or 1)
        if not data:
            return ""
        text = data.decode("utf-8", errors="replace")
        self.output.write(text)
        self.buffer += text
        return text

    def send(self, command):
        self.output.write(f"\n[{stamp()}] COMMAND {command}\n")
        self.serial.write((command + "\r\n").encode("ascii"))
        self.serial.flush()

    def collect(self, timeout, pattern=None):
        start = len(self.buffer)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read()
            result = self.buffer[start:]
            if pattern is not None and re.search(pattern, result):
                return result
            time.sleep(0.02)
        return self.buffer[start:]

    def close(self):
        self.serial.close()


class EcgConsoleThread(threading.Thread):
    def __init__(self, port, output, run_log):
        super().__init__(daemon=True)
        self.port = port
        self.output = output
        self.run_log = run_log
        self.stop_requested = threading.Event()
        self.error = None

    def run(self):
        try:
            with serial.Serial(self.port, 115200, timeout=0.2, rtscts=False,
                               dsrdtr=False) as console:
                console.dtr = True
                while not self.stop_requested.is_set():
                    data = console.read(console.in_waiting or 1)
                    if data:
                        self.output.write(data.decode("utf-8", errors="replace"))
        except serial.SerialException as error:
            self.error = str(error)
            self.run_log.event("ecg_console_error", error=self.error)


def wait_with_heartbeat(seconds, run_log, reason):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        remaining = max(0, round(deadline - time.monotonic()))
        run_log.event("waiting", reason=reason, remaining_seconds=remaining)
        time.sleep(min(60, max(1, remaining)))


def exit_ship_mode(script, reset_kind, run_log):
    command = [sys.executable, str(script), "--reset-kind", reset_kind]
    run_log.event("ship_mode_exit_start", reset_kind=reset_kind)
    result = subprocess.run(command, text=True, capture_output=True, timeout=300)
    run_log.event("ship_mode_exit_result", reset_kind=reset_kind,
                  returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"ship-mode exit failed: {result.stderr or result.stdout}")


def connect_target(command, run_log):
    for attempt in range(1, 13):
        command.send("status")
        status = command.collect(3, r"STATUS[^\r\n]*")
        if re.search(r"state=(READY|COMPLETE)", status):
            if f"peer_name={TARGET_NAME}" in status:
                return
            command.send("disconnect")
            command.collect(6, r"DISCONNECTED|ERR")
        elif "state=IDLE" not in status:
            command.send("disconnect")
            command.collect(6, r"DISCONNECTED|ERR")

        command.send("connect ecg")
        response = command.collect(15, r"PEER_READY|ERROR|DISCONNECTED")
        if TARGET_NAME in response and "PEER_READY nus=1" in response:
            command.send("status")
            status = command.collect(3, r"STATUS[^\r\n]*")
            if f"peer_name={TARGET_NAME}" in status:
                return
        run_log.event("connect_retry", attempt=attempt, response=response[-2000:])
        if "CONNECTED" in response or "PEER_READY" in response:
            command.send("disconnect")
            command.collect(6, r"DISCONNECTED|ERR")
        time.sleep(5)
    raise RuntimeError(f"could not connect specifically to {TARGET_NAME}")


def enable_collection(command):
    command.send("collect on")
    response = command.collect(15, r"COLLECT_RESULT|ERR")
    if "COLLECT_RESULT type=ECG enabled=1 status=success" not in response:
        raise RuntimeError(f"ECG collection enable failed: {response[-1000:]}")


def finite_cycle(command, relay, capture_path, session_id, run_log):
    relay.reset_input_buffer()
    command.send(f"start {session_id}")
    deadline = time.monotonic() + 210
    response = ""
    relay_data = bytearray()
    terminal_seen = False
    relay_idle_seen = False
    while time.monotonic() < deadline:
        chunk = command.read()
        response += chunk
        if relay.in_waiting:
            relay_data.extend(relay.read(relay.in_waiting))
        terminal_seen = terminal_seen or ANY_TERMINAL.search(response) is not None
        relay_idle_seen = relay_idle_seen or "RELAY_IDLE" in response
        if terminal_seen and relay_idle_seen and not relay.in_waiting:
            break
        time.sleep(0.01)
    capture_path.write_bytes(relay_data)
    result = {
        "session_id": session_id,
        "terminal_seen": terminal_seen,
        "relay_idle_seen": relay_idle_seen,
        "relay_bytes": len(relay_data),
        "stream_ok": re.search(r"STREAM_OK[^\r\n]*bytes=131072", response) is not None,
        "target_condition": TARGET_TERMINAL.search(response) is not None,
        "response_tail": response[-4000:],
    }
    run_log.event("finite_result", **result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-port", default="COM23")
    parser.add_argument("--relay-port", default="COM22")
    parser.add_argument("--ecg-console-port", default="COM52")
    parser.add_argument("--duration-hours", type=float, default=8.0)
    parser.add_argument("--idle-min-seconds", type=int, default=90)
    parser.add_argument("--idle-max-seconds", type=int, default=150)
    parser.add_argument("--reset-min-minutes", type=int, default=45)
    parser.add_argument("--reset-max-minutes", type=int, default=75)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--skip-initial-ship-exit", action="store_true")
    parser.add_argument("--output-root", type=pathlib.Path,
                        default=pathlib.Path(__file__).parent / "captures")
    args = parser.parse_args()
    if args.idle_min_seconds > args.idle_max_seconds:
        parser.error("idle minimum exceeds maximum")
    if args.reset_min_minutes > args.reset_max_minutes:
        parser.error("reset minimum exceeds maximum")

    seed = args.seed if args.seed is not None else time.time_ns()
    rng = random.Random(seed)
    run_name = datetime.datetime.now().strftime("ex4bt_%Y%m%d_%H%M%S")
    run_log = RunLog(args.output_root / run_name)
    command = None
    relay = None
    console_thread = EcgConsoleThread(args.ecg_console_port, run_log.ecg, run_log)
    helper = pathlib.Path(__file__).with_name("exit_ecg_ship_mode.py")
    consecutive_failures = 0
    session_id = 1
    reset_count = 0
    deadline = time.monotonic() + args.duration_hours * 3600
    next_reset = time.monotonic() + rng.randint(args.reset_min_minutes * 60,
                                                args.reset_max_minutes * 60)
    run_log.event("run_start", seed=seed, target=TARGET_NAME,
                  duration_hours=args.duration_hours)
    console_thread.start()
    try:
        if not args.skip_initial_ship_exit:
            exit_ship_mode(helper, "RESET_PIN", run_log)
        command = TextPort(args.command_port, 115200, run_log.control)
        relay = serial.Serial(args.relay_port, 1_000_000, timeout=0.02,
                              write_timeout=2, rtscts=False, dsrdtr=False)
        relay.dtr = True
        command.send("help")
        if "COMMANDS:" not in command.collect(3, r"COMMANDS:"):
            raise RuntimeError("COM23 did not identify as the command port")
        connect_target(command, run_log)
        enable_collection(command)
        wait_with_heartbeat(120, run_log, "initial_history_fill")

        while time.monotonic() < deadline:
            if time.monotonic() >= next_reset:
                command.send("disconnect")
                command.collect(8, r"DISCONNECTED|ERR")
                reset_count += 1
                reset_kind = "RESET_SYSTEM" if reset_count % 2 else "RESET_PIN"
                exit_ship_mode(helper, reset_kind, run_log)
                connect_target(command, run_log)
                enable_collection(command)
                wait_with_heartbeat(120, run_log, "post_reset_history_fill")
                next_reset = time.monotonic() + rng.randint(args.reset_min_minutes * 60,
                                                            args.reset_max_minutes * 60)

            result = finite_cycle(command, relay, run_log.root / f"session_{session_id}.mrly",
                                  session_id, run_log)
            session_id += 1
            if result["target_condition"]:
                run_log.observation("Target NOT_RECORDING condition",
                                    result["response_tail"])
                run_log.event("diagnostic_handoff", reason="target_condition")
                return 42
            if result["stream_ok"] and result["relay_idle_seen"]:
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                run_log.observation("FINITE stream anomaly", result["response_tail"])
                if consecutive_failures >= 3:
                    run_log.event("diagnostic_handoff", reason="three_consecutive_failures")
                    return 43
                command.send("disconnect")
                command.collect(8, r"DISCONNECTED|ERR")
                connect_target(command, run_log)
            if session_id > 1 and (session_id - 1) % 10 == 0:
                run_log.event("scheduled_reconnect", completed_sessions=session_id - 1)
                command.send("disconnect")
                command.collect(8, r"DISCONNECTED|ERR")
                connect_target(command, run_log)
            idle_seconds = rng.randint(args.idle_min_seconds, args.idle_max_seconds)
            wait_with_heartbeat(idle_seconds, run_log, "history_refill")

        run_log.event("run_complete", sessions=session_id - 1, resets=reset_count)
        return 0
    except (OSError, RuntimeError, serial.SerialException,
            subprocess.SubprocessError) as error:
        run_log.observation("Harness stopped on an unexpected error", str(error))
        run_log.event("run_error", error=type(error).__name__, detail=str(error))
        raise
    finally:
        console_thread.stop_requested.set()
        console_thread.join(timeout=2)
        if relay is not None:
            relay.close()
        if command is not None:
            command.close()
        run_log.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, serial.SerialException, subprocess.SubprocessError) as error:
        print(f"LONG_RUN_FAILED {error}", file=sys.stderr)
        sys.exit(2)
