#!/usr/bin/env python3
"""Unattended host client for the MotionSense Central MDFU v1 protocol.

The host retains an MCUboot application ``.bin`` and only sends a frame after the
Central issues a matching ``DFU_CREDIT``. It deliberately does not own BLE or
MCUmgr sequencing; the Central does that work and reports terminal events.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import re
import secrets
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path
from typing import Callable, Iterable, Iterator, Mapping, Sequence

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Unit tests exercise parser code without pyserial installed.
    serial = None
    list_ports = None


EXIT_SUCCESS = 0
EXIT_USAGE = 2
EXIT_IMAGE = 3
EXIT_PORT = 4
EXIT_PROTOCOL = 5
EXIT_DFU_FAILURE = 6
EXIT_INTERRUPTED = 130

MCUBOOT_IMAGE_MAGIC = 0x96F3B83D
MCUBOOT_TLV_INFO_MAGIC = 0x6907
MCUBOOT_TLV_PROT_INFO_MAGIC = 0x6908
MCUBOOT_TLV_SHA256 = 0x10
MCUBOOT_SIGNATURE_TLVS = frozenset(range(0x20, 0x26))

MDFU_MAGIC = b"MDFU"
MDFU_VERSION = 1
MDFU_DATA_TYPE = 1
MDFU_HEADER_BYTES = 24
MDFU_MAX_PAYLOAD = 384


class DfuError(RuntimeError):
    """A predictable user-facing MDFU error."""


class ImageValidationError(DfuError):
    """The supplied file is not a usable MCUboot image binary."""


@dataclasses.dataclass(frozen=True)
class McubootImage:
    path: Path
    data: bytes
    header_size: int
    image_size: int
    version: tuple[int, int, int, int]
    tlv_sha256: bytes
    whole_file_sha256: bytes
    signature_tlvs: tuple[int, ...]

    @property
    def tlv_sha256_hex(self) -> str:
        return self.tlv_sha256.hex()

    @property
    def whole_file_sha256_hex(self) -> str:
        return self.whole_file_sha256.hex()


@dataclasses.dataclass(frozen=True)
class MdfuFrame:
    transaction: int
    offset: int
    payload: bytes


@dataclasses.dataclass(frozen=True)
class PortRecord:
    device: str
    description: str
    hwid: str
    serial_number: str | None
    location: str | None
    interface: str | None

    @property
    def group_key(self) -> str:
        if self.serial_number:
            return f"serial:{self.serial_number}"
        if self.location:
            parent = re.sub(r"[.:_-]?\d+$", "", self.location)
            return f"location:{parent or self.location}"
        return f"port:{self.device}"


def _le16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _le32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _parse_tlv_area(data: bytes, offset: int, expected_magic: int | None) -> tuple[int, list[tuple[int, bytes]]]:
    if offset + 4 > len(data):
        raise ImageValidationError("missing MCUboot TLV info header")
    magic = _le16(data, offset)
    total_size = _le16(data, offset + 2)
    if expected_magic is not None and magic != expected_magic:
        raise ImageValidationError(f"unexpected TLV magic 0x{magic:04x}")
    if magic not in (MCUBOOT_TLV_INFO_MAGIC, MCUBOOT_TLV_PROT_INFO_MAGIC):
        raise ImageValidationError(f"invalid MCUboot TLV magic 0x{magic:04x}")
    if total_size < 4 or offset + total_size > len(data):
        raise ImageValidationError("MCUboot TLV area exceeds the binary")

    entries: list[tuple[int, bytes]] = []
    cursor = offset + 4
    end = offset + total_size
    while cursor < end:
        if cursor + 4 > end:
            raise ImageValidationError("truncated MCUboot TLV entry")
        tlv_type = _le16(data, cursor)
        length = _le16(data, cursor + 2)
        cursor += 4
        if cursor + length > end:
            raise ImageValidationError("MCUboot TLV entry exceeds its area")
        entries.append((tlv_type, data[cursor : cursor + length]))
        cursor += length
    if cursor != end:
        raise ImageValidationError("misaligned MCUboot TLV area")
    return total_size, entries


def parse_mcuboot_bin(path: str | Path) -> McubootImage:
    """Read an MCUboot application ``.bin`` and extract identity metadata.

    The parser intentionally validates image structure and hashes only. It
    reports whether a signature TLV exists but never requests a signing key or
    attempts signature verification.
    """

    image_path = Path(path)
    if image_path.suffix.lower() != ".bin":
        raise ImageValidationError("only MCUboot application .bin images are accepted")
    try:
        data = image_path.read_bytes()
    except OSError as exc:
        raise ImageValidationError(f"cannot read image: {exc}") from exc
    if len(data) < 32:
        raise ImageValidationError("file is too short for an MCUboot header")
    if _le32(data, 0) != MCUBOOT_IMAGE_MAGIC:
        raise ImageValidationError("MCUboot image magic is missing")

    header_size = _le16(data, 8)
    protected_tlv_size = _le16(data, 10)
    image_size = _le32(data, 12)
    if header_size < 32 or header_size > len(data):
        raise ImageValidationError("invalid MCUboot header size")
    image_end = header_size + image_size
    if image_end < header_size or image_end > len(data):
        raise ImageValidationError("MCUboot image body exceeds the binary")

    tlv_entries: list[tuple[int, bytes]] = []
    cursor = image_end
    if protected_tlv_size:
        actual_size, entries = _parse_tlv_area(data, cursor, MCUBOOT_TLV_PROT_INFO_MAGIC)
        if actual_size != protected_tlv_size:
            raise ImageValidationError("protected TLV size disagrees with image header")
        tlv_entries.extend(entries)
        cursor += actual_size
    if cursor + 4 <= len(data):
        magic = _le16(data, cursor)
        if magic == MCUBOOT_TLV_INFO_MAGIC:
            _, entries = _parse_tlv_area(data, cursor, MCUBOOT_TLV_INFO_MAGIC)
            tlv_entries.extend(entries)

    sha_values = [value for tlv_type, value in tlv_entries if tlv_type == MCUBOOT_TLV_SHA256]
    if len(sha_values) != 1 or len(sha_values[0]) != 32:
        raise ImageValidationError("MCUboot SHA256 TLV is missing or invalid")
    signature_tlvs = tuple(tlv_type for tlv_type, _ in tlv_entries if tlv_type in MCUBOOT_SIGNATURE_TLVS)

    return McubootImage(
        path=image_path,
        data=data,
        header_size=header_size,
        image_size=image_size,
        version=(data[20], data[21], _le16(data, 22), _le32(data, 24)),
        tlv_sha256=sha_values[0],
        whole_file_sha256=hashlib.sha256(data).digest(),
        signature_tlvs=signature_tlvs,
    )


def encode_mdfu_frame(transaction: int, offset: int, payload: bytes) -> bytes:
    if not 0 < len(payload) <= MDFU_MAX_PAYLOAD:
        raise ValueError("MDFU payload must contain 1..384 bytes")
    if not 0 < transaction <= 0xFFFFFFFF:
        raise ValueError("MDFU transaction must be nonzero uint32")
    if not 0 <= offset <= 0xFFFFFFFF:
        raise ValueError("MDFU offset must be uint32")
    prefix = struct.pack(
        "<4sBBHIIHH",
        MDFU_MAGIC,
        MDFU_VERSION,
        MDFU_DATA_TYPE,
        MDFU_HEADER_BYTES,
        transaction,
        offset,
        len(payload),
        0,
    )
    crc = zlib.crc32(prefix[4:] + payload) & 0xFFFFFFFF
    return prefix + struct.pack("<I", crc) + payload


class MdfuFrameParser:
    """Incremental, safe-resynchronizing MDFU v1 parser."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Iterator[MdfuFrame]:
        self._buffer.extend(data)
        while True:
            magic_at = self._buffer.find(MDFU_MAGIC)
            if magic_at < 0:
                del self._buffer[:-3]
                return
            if magic_at:
                del self._buffer[:magic_at]
            if len(self._buffer) < MDFU_HEADER_BYTES:
                return
            version, frame_type, header_size = struct.unpack_from("<BBH", self._buffer, 4)
            transaction, offset, payload_length, reserved, expected_crc = struct.unpack_from(
                "<IIHHI", self._buffer, 8
            )
            if (
                version != MDFU_VERSION
                or frame_type != MDFU_DATA_TYPE
                or header_size != MDFU_HEADER_BYTES
                or not 0 < payload_length <= MDFU_MAX_PAYLOAD
                or reserved != 0
            ):
                del self._buffer[0]
                continue
            frame_length = MDFU_HEADER_BYTES + payload_length
            if len(self._buffer) < frame_length:
                return
            payload = bytes(self._buffer[MDFU_HEADER_BYTES:frame_length])
            actual_crc = zlib.crc32(bytes(self._buffer[4:20]) + payload) & 0xFFFFFFFF
            if actual_crc != expected_crc:
                del self._buffer[0]
                continue
            del self._buffer[:frame_length]
            yield MdfuFrame(transaction=transaction, offset=offset, payload=payload)


def parse_event(line: str) -> dict[str, object]:
    line = line.strip()
    if not line:
        return {"event": "", "raw": ""}
    tokens = line.split()
    fields: dict[str, str] = {}
    for token in tokens[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value
    return {"event": tokens[0], "fields": fields, "raw": line}


def classify_exit(event: str) -> int:
    if event == "DFU_SUCCESS":
        return EXIT_SUCCESS
    if event == "DFU_FAIL":
        return EXIT_DFU_FAILURE
    return EXIT_PROTOCOL


def list_port_records() -> list[PortRecord]:
    if list_ports is None:
        raise DfuError("pyserial is required; install it with: python -m pip install pyserial")
    return [
        PortRecord(
            device=port.device,
            description=port.description or "",
            hwid=port.hwid or "",
            serial_number=getattr(port, "serial_number", None),
            location=getattr(port, "location", None),
            interface=getattr(port, "interface", None),
        )
        for port in list_ports.comports()
    ]


def group_ports(records: Iterable[PortRecord]) -> dict[str, list[PortRecord]]:
    groups: dict[str, list[PortRecord]] = {}
    for record in records:
        groups.setdefault(record.group_key, []).append(record)
    for group in groups.values():
        group.sort(key=lambda record: record.device)
    return groups


def _serial_matches(requested: str, actual: str | None) -> bool:
    if actual is None:
        return False
    return requested == actual or requested.lstrip("0") == actual.lstrip("0")


def parse_nrfutil_vcom_pair(payload: Mapping[str, object], jlink_serial: str | None) -> tuple[str, str]:
    """Select exactly one J-Link VCOM0/VCOM1 pair from nrfutil JSON."""

    devices = payload.get("devices")
    if not isinstance(devices, list):
        raise DfuError("nrfutil device list did not contain a devices array")

    candidates: list[tuple[str, str]] = []
    for device in devices:
        if not isinstance(device, dict):
            continue
        traits = device.get("traits")
        if not isinstance(traits, dict) or not traits.get("jlink"):
            continue
        serial_number = str(device.get("serialNumber", ""))
        if jlink_serial and not _serial_matches(jlink_serial, serial_number):
            continue
        ports = device.get("serialPorts")
        if not isinstance(ports, list):
            continue
        command_ports: list[str] = []
        data_ports: list[str] = []
        for port in ports:
            if not isinstance(port, dict):
                continue
            try:
                vcom = int(port.get("vcom"))
            except (TypeError, ValueError):
                continue
            com_name = port.get("comName")
            if not isinstance(com_name, str) or not com_name:
                continue
            if vcom == 0:
                command_ports.append(com_name)
            elif vcom == 1:
                data_ports.append(com_name)
        if len(command_ports) == 1 and len(data_ports) == 1:
            candidates.append((command_ports[0], data_ports[0]))

    if len(candidates) != 1:
        qualifier = f" for J-Link serial {jlink_serial}" if jlink_serial else ""
        raise DfuError(
            f"nrfutil could not identify exactly one J-Link VCOM0/VCOM1 pair{qualifier}; "
            "use --command-port and --data-port"
        )
    return candidates[0]


def nrfutil_device_list() -> Mapping[str, object]:
    try:
        result = subprocess.run(
            ["nrfutil", "device", "list", "--json", "--skip-overhead", "--log-output=stdout"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise DfuError(
            "nrfutil is unavailable for automatic VCOM identification; "
            "use --command-port and --data-port"
        ) from exc
    if result.returncode != 0:
        raise DfuError(
            "nrfutil device list failed; use --command-port and --data-port"
        )
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise DfuError(
            "nrfutil device list returned invalid JSON; use --command-port and --data-port"
        ) from exc
    if not isinstance(payload, dict):
        raise DfuError("nrfutil device list returned invalid JSON; use --command-port and --data-port")
    return payload


def select_ports(
    records: Sequence[PortRecord],
    command_port: str | None,
    data_port: str | None,
    jlink_serial: str | None = None,
    nrfutil_loader: Callable[[], Mapping[str, object]] | None = None,
) -> tuple[str, str]:
    if bool(command_port) != bool(data_port):
        raise DfuError("provide both --command-port and --data-port, or neither")
    if command_port and data_port:
        if command_port == data_port:
            raise DfuError("command and data ports must differ")
        return command_port, data_port

    groups = group_ports(records)
    if jlink_serial:
        groups = {
            key: value
            for key, value in groups.items()
            if any(_serial_matches(jlink_serial, record.serial_number) for record in value)
        }
    candidates: list[tuple[PortRecord, PortRecord]] = []
    for group in groups.values():
        command_candidates = [
            record
            for record in group
            if re.search(r"vcom0|serial\s*port\s*0|interface\s*0", f"{record.interface} {record.description}", re.I)
        ]
        data_candidates = [
            record
            for record in group
            if re.search(r"vcom2|serial\s*port\s*1|interface\s*1", f"{record.interface} {record.description}", re.I)
        ]
        if len(command_candidates) == 1 and len(data_candidates) == 1:
            candidates.append((command_candidates[0], data_candidates[0]))
    if len(candidates) != 1:
        loader = nrfutil_loader or nrfutil_device_list
        return parse_nrfutil_vcom_pair(loader(), jlink_serial)
    return candidates[0][0].device, candidates[0][1].device


class CommandChannel:
    def __init__(self, port: object) -> None:
        self.port = port
        self.buffer = bytearray()

    def send(self, command: str) -> None:
        self.port.write((command + "\n").encode("ascii"))
        self.port.flush()

    def events_until(self, deadline: float) -> Iterator[dict[str, object]]:
        while time.monotonic() < deadline:
            chunk = self.port.read(256)
            if not chunk:
                continue
            self.buffer.extend(chunk)
            while b"\n" in self.buffer:
                line, _, remainder = self.buffer.partition(b"\n")
                self.buffer[:] = remainder
                yield parse_event(line.rstrip(b"\r").decode("utf-8", errors="replace"))


def _open_serial(port: str, baudrate: int, rtscts: bool = False) -> object:
    if serial is None:
        raise DfuError("pyserial is required; install it with: python -m pip install pyserial")
    try:
        handle = serial.Serial(port, baudrate=baudrate, timeout=0.1, write_timeout=5, rtscts=rtscts)
        handle.dtr = True
        handle.reset_input_buffer()
        return handle
    except Exception as exc:
        raise DfuError(f"cannot open {port} at {baudrate}: {exc}") from exc


def _event_tx(event: Mapping[str, object]) -> int | None:
    value = event.get("fields", {}).get("tx") if isinstance(event.get("fields"), dict) else None
    try:
        return int(value, 0) if value is not None else None
    except ValueError:
        return None


def _emit(event: Mapping[str, object], json_lines: bool) -> None:
    if json_lines:
        print(json.dumps(event, sort_keys=True), flush=True)
    else:
        print(event["raw"], flush=True)


def _progress(message: str, json_lines: bool) -> None:
    if json_lines:
        print(message, file=sys.stderr, flush=True)


def _wait_for(
    channel: CommandChannel,
    timeout: float,
    json_lines: bool,
    predicate,
) -> dict[str, object]:
    deadline = time.monotonic() + timeout
    for event in channel.events_until(deadline):
        _emit(event, json_lines)
        if predicate(event):
            return event
    raise DfuError("timed out waiting for Central command event")


def _event_fields(event: Mapping[str, object]) -> Mapping[str, str]:
    fields = event.get("fields")
    return fields if isinstance(fields, dict) else {}


def target_matches_peer(target: str, peer_name: str | None) -> bool:
    if peer_name is None:
        return False
    if target == "ppg":
        return peer_name.startswith("MSense4PPG")
    if target == "ecg":
        return peer_name.startswith("MSense4ECG")
    return target == "any" and peer_name.startswith("MSense")


def _require_target_peer(fields: Mapping[str, str], target: str) -> None:
    peer_name = fields.get("peer_name")
    if not target_matches_peer(target, peer_name):
        raise DfuError(
            f"peer identity mismatch: target={target} peer_name={peer_name or 'missing'} "
            f"peer_addr={fields.get('peer_addr', 'missing')}"
        )


def _wait_for_smp_ready(
    channel: CommandChannel, target: str, timeout: float, json_lines: bool
) -> None:
    deadline = time.monotonic() + timeout
    for event in channel.events_until(deadline):
        _emit(event, json_lines)
        name = str(event["event"])
        fields = _event_fields(event)
        if name == "PEER_READY":
            if fields.get("smp") == "1":
                _require_target_peer(fields, target)
                return
            raise DfuError("peer discovery completed without SMP support")
        if name in ("SMP_UNAVAILABLE", "ERR", "ERROR"):
            raise DfuError(f"Central connection failed: {event['raw']}")
    raise DfuError("timed out waiting for SMP peer discovery")


def _disconnect_and_wait(channel: CommandChannel, timeout: float, json_lines: bool) -> None:
    channel.send("disconnect")
    deadline = time.monotonic() + timeout
    for event in channel.events_until(deadline):
        _emit(event, json_lines)
        if event["event"] == "DISCONNECTED":
            return
        if event["event"] in ("ERR", "ERROR"):
            raise DfuError(f"Central disconnect failed: {event['raw']}")
    raise DfuError("timed out waiting for mismatched peer disconnect")


def ensure_smp_ready(channel: CommandChannel, target: str, timeout: float, json_lines: bool) -> None:
    """Keep an SMP-ready peer or connect and wait for a newly discovered one."""

    channel.send("status")
    status = _wait_for(
        channel,
        timeout,
        json_lines,
        lambda event: event["event"] in ("STATUS", "ERR", "ERROR"),
    )
    if status["event"] != "STATUS":
        raise DfuError(f"Central status failed: {status['raw']}")
    fields = _event_fields(status)
    if fields.get("smp") == "1":
        if target_matches_peer(target, fields.get("peer_name")):
            return
        _disconnect_and_wait(channel, timeout, json_lines)
        channel.send(f"connect {target}")
        _wait_for_smp_ready(channel, target, timeout, json_lines)
        return

    if fields.get("state") in {"SCANNING", "CONNECTING", "MTU_EXCHANGE", "DISCOVERING", "SUBSCRIBING"}:
        _wait_for_smp_ready(channel, target, timeout, json_lines)
        return
    channel.send(f"connect {target}")
    _wait_for_smp_ready(channel, target, timeout, json_lines)


def run_upgrade(args: argparse.Namespace) -> int:
    image = parse_mcuboot_bin(args.image)
    records = list_port_records()
    command_name, data_name = select_ports(records, args.command_port, args.data_port, args.jlink_serial)
    transaction = args.transaction or secrets.randbelow(0xFFFFFFFF - 1) + 1
    command_handle = _open_serial(command_name, 115200)
    data_handle = None
    try:
        channel = CommandChannel(command_handle)
        channel.send("dfu capabilities")
        caps = _wait_for(channel, args.timeout, args.json, lambda event: event["event"] == "DFU_CAPS")
        fields = _event_fields(caps)
        hwfc = fields.get("hwfc") == "1"
        if args.binary_rtscts == "on":
            hwfc = True
        elif args.binary_rtscts == "off":
            hwfc = False
        ensure_smp_ready(channel, args.target, args.timeout, args.json)
        data_handle = _open_serial(data_name, 1_000_000, rtscts=hwfc)

        _progress(f"preflight image list tx={transaction}", args.json)
        channel.send(f"dfu list {transaction}")
        preflight = _wait_for(
            channel,
            args.timeout,
            args.json,
            lambda event: _event_tx(event) == transaction and event["event"] in ("DFU_DONE", "DFU_FAIL"),
        )
        if preflight["event"] == "DFU_FAIL":
            return EXIT_DFU_FAILURE
        if args.preflight_only:
            return EXIT_SUCCESS

        allow_same = " allow-same" if args.allow_same else ""
        command = (
            f"dfu begin {transaction} {len(image.data)} {image.whole_file_sha256_hex} "
            f"{image.tlv_sha256_hex}{allow_same}"
        )
        _progress(f"starting tx={transaction} bytes={len(image.data)}", args.json)
        channel.send(command)
        deadline = time.monotonic() + args.timeout
        while True:
            for event in channel.events_until(deadline):
                _emit(event, args.json)
                event_name = event["event"]
                fields = event.get("fields", {})
                if not isinstance(fields, dict):
                    continue
                if _event_tx(event) != transaction:
                    continue
                if event_name == "DFU_CREDIT":
                    offset = int(fields["off"], 0)
                    maximum = int(fields["max"], 0)
                    if maximum <= 0 or maximum > MDFU_MAX_PAYLOAD or offset >= len(image.data):
                        raise DfuError("Central issued an invalid DFU credit")
                    payload = image.data[offset : min(offset + maximum, len(image.data))]
                    encoded = encode_mdfu_frame(transaction, offset, payload)
                    if data_handle.write(encoded) != len(encoded):
                        raise DfuError("short MDFU frame write")
                    data_handle.flush()
                    deadline = time.monotonic() + args.timeout
                    _progress(f"sent offset={offset} bytes={len(payload)}", args.json)
                elif event_name == "DFU_SUCCESS":
                    return EXIT_SUCCESS
                elif event_name == "DFU_FAIL":
                    return EXIT_DFU_FAILURE
            if time.monotonic() >= deadline:
                raise DfuError("timed out waiting for DFU credit or terminal event")
    except KeyboardInterrupt:
        try:
            CommandChannel(command_handle).send(f"dfu abort {transaction}")
        except Exception:
            pass
        return EXIT_INTERRUPTED
    finally:
        if data_handle is not None:
            data_handle.close()
        command_handle.close()


def _print_ports(records: Sequence[PortRecord]) -> None:
    for key, group in group_ports(records).items():
        print(f"{key}:")
        for record in group:
            print(
                f"  {record.device} interface={record.interface or '-'} "
                f"serial={record.serial_number or '-'} description={record.description}"
            )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="MCUboot application .bin image")
    parser.add_argument("--target", choices=("ppg", "ecg", "any"),
                        help="peer type to connect when no SMP-ready peer exists")
    parser.add_argument("--command-port", help="DK command VCOM at 115200")
    parser.add_argument("--data-port", help="DK binary VCOM at 1000000")
    parser.add_argument("--jlink-serial", help="restrict automatic VCOM pairing to this interface-MCU serial")
    parser.add_argument("--transaction", type=lambda value: int(value, 0), help="nonzero uint32 transaction ID")
    parser.add_argument("--timeout", type=float, default=45.0, help="per-stage timeout in seconds")
    parser.add_argument("--allow-same", action="store_true", help="allow an image matching active slot 0")
    parser.add_argument("--preflight-only", action="store_true", help="list target images without beginning upload")
    parser.add_argument("--binary-rtscts", choices=("auto", "on", "off"), default="auto")
    parser.add_argument("--json", action="store_true", help="emit command events as JSON Lines on stdout")
    parser.add_argument("--list-ports", action="store_true", help="print grouped serial ports and exit")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.list_ports:
            _print_ports(list_port_records())
            return EXIT_SUCCESS
        if args.image is None:
            raise DfuError("--image is required unless --list-ports is used")
        if args.target is None:
            raise DfuError("--target ppg|ecg|any is required for an unattended DFU run")
        if args.transaction is not None and not 0 < args.transaction <= 0xFFFFFFFF:
            raise DfuError("--transaction must be a nonzero uint32")
        return run_upgrade(args)
    except ImageValidationError as exc:
        print(f"image error: {exc}", file=sys.stderr)
        return EXIT_IMAGE
    except DfuError as exc:
        print(f"dfu error: {exc}", file=sys.stderr)
        return EXIT_PORT if "port" in str(exc).lower() or "pyserial" in str(exc).lower() else EXIT_PROTOCOL


if __name__ == "__main__":
    raise SystemExit(main())
