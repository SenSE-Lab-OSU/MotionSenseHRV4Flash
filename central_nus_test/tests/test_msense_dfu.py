"""Host-side regression tests for the Central MDFU v1 protocol."""

from __future__ import annotations

import hashlib
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import msense_dfu  # noqa: E402


def make_mcuboot_bin(signature: bool = True) -> bytes:
    """Make a small structurally-valid MCUboot binary for parser tests."""

    body = b"MotionSense DFU test image"
    header = bytearray(32)
    struct.pack_into("<I", header, 0, msense_dfu.MCUBOOT_IMAGE_MAGIC)
    struct.pack_into("<H", header, 8, len(header))
    struct.pack_into("<H", header, 10, 0)
    struct.pack_into("<I", header, 12, len(body))
    header[20:22] = b"\x01\x02"
    struct.pack_into("<H", header, 22, 3)
    struct.pack_into("<I", header, 24, 4)

    entries = [struct.pack("<HH", msense_dfu.MCUBOOT_TLV_SHA256, 32) + b"\x5a" * 32]
    if signature:
        entries.append(struct.pack("<HH", 0x20, 64) + b"\xa5" * 64)
    tlv_body = b"".join(entries)
    tlv = struct.pack("<HH", msense_dfu.MCUBOOT_TLV_INFO_MAGIC, len(tlv_body) + 4) + tlv_body
    return bytes(header) + body + tlv


class McubootParserTests(unittest.TestCase):
    def test_extracts_identity_and_signature_presence(self) -> None:
        blob = make_mcuboot_bin(signature=True)
        with tempfile.TemporaryDirectory() as directory:
            image_path = Path(directory) / "app_update.bin"
            image_path.write_bytes(blob)
            image = msense_dfu.parse_mcuboot_bin(image_path)

        self.assertEqual(image.header_size, 32)
        self.assertEqual(image.image_size, len(b"MotionSense DFU test image"))
        self.assertEqual(image.version, (1, 2, 3, 4))
        self.assertEqual(image.tlv_sha256, b"\x5a" * 32)
        self.assertEqual(image.whole_file_sha256, hashlib.sha256(blob).digest())
        self.assertEqual(image.signature_tlvs, (0x20,))

    def test_accepts_only_bin_extension(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            image_path = Path(directory) / "app_update.hex"
            image_path.write_bytes(make_mcuboot_bin())
            with self.assertRaises(msense_dfu.ImageValidationError):
                msense_dfu.parse_mcuboot_bin(image_path)


class MdfuWireTests(unittest.TestCase):
    def test_fragmented_frame_and_crc_resynchronization(self) -> None:
        payload = bytes(range(64))
        encoded = msense_dfu.encode_mdfu_frame(0x1234, 384, payload)
        parser = msense_dfu.MdfuFrameParser()

        self.assertEqual(list(parser.feed(encoded[:7])), [])
        self.assertEqual(list(parser.feed(encoded[7:31])), [])
        frames = list(parser.feed(encoded[31:]))
        self.assertEqual(frames, [msense_dfu.MdfuFrame(0x1234, 384, payload)])

        damaged = bytearray(encoded)
        damaged[-1] ^= 0x01
        frames = list(parser.feed(b"noise" + bytes(damaged) + encoded))
        self.assertEqual(frames, [msense_dfu.MdfuFrame(0x1234, 384, payload)])

    def test_credit_transcript_maps_offset_to_one_frame(self) -> None:
        credit = msense_dfu.parse_event("DFU_CREDIT tx=77 off=16 max=8")
        self.assertEqual(credit["event"], "DFU_CREDIT")
        fields = credit["fields"]
        self.assertEqual(fields, {"tx": "77", "off": "16", "max": "8"})
        frame = msense_dfu.encode_mdfu_frame(77, int(fields["off"]), b"ABCDEFGH")
        self.assertEqual(
            list(msense_dfu.MdfuFrameParser().feed(frame)),
            [msense_dfu.MdfuFrame(77, 16, b"ABCDEFGH")],
        )

    def test_wire_retry_transcript_reissues_the_same_credit_offset(self) -> None:
        transcript = (
            "DFU_CREDIT tx=77 off=16 max=8",
            "DFU_RETRY tx=77 off=16 reason=wire",
            "DFU_CREDIT tx=77 off=16 max=8",
        )
        frames = []
        for line in transcript:
            event = msense_dfu.parse_event(line)
            if event["event"] == "DFU_CREDIT":
                fields = event["fields"]
                frames.append(
                    msense_dfu.encode_mdfu_frame(
                        int(fields["tx"]), int(fields["off"]), b"ABCDEFGH"
                    )
                )
        decoded = [list(msense_dfu.MdfuFrameParser().feed(frame))[0] for frame in frames]
        self.assertEqual(decoded, [msense_dfu.MdfuFrame(77, 16, b"ABCDEFGH")] * 2)


class FakeCommandChannel:
    def __init__(self, batches: list[list[str]]) -> None:
        self.batches = batches
        self.sent: list[str] = []

    def send(self, command: str) -> None:
        self.sent.append(command)

    def events_until(self, deadline: float):
        del deadline
        if self.batches:
            for line in self.batches.pop(0):
                yield msense_dfu.parse_event(line)


class ConnectHandlingTests(unittest.TestCase):
    def test_keeps_existing_smp_ready_peer(self) -> None:
        channel = FakeCommandChannel(
            [["STATUS state=READY subscribed=1 smp=1 peer_name=MSense4PPG-1 peer_addr=AA:BB:CC:DD:EE:FF"]]
        )
        msense_dfu.ensure_smp_ready(channel, "ppg", 1.0, False)
        self.assertEqual(channel.sent, ["status"])

    def test_any_keeps_existing_smp_only_blinky_peer(self) -> None:
        channel = FakeCommandChannel(
            [["STATUS state=READY subscribed=0 smp=1 peer_name=MSenseBlinky peer_addr=AA:BB:CC:DD:EE:FF"]]
        )
        msense_dfu.ensure_smp_ready(channel, "any", 1.0, False)
        self.assertEqual(channel.sent, ["status"])

    def test_connects_and_waits_for_smp_peer_ready(self) -> None:
        channel = FakeCommandChannel(
            [
                ["STATUS state=IDLE subscribed=0 smp=0"],
                ["CONNECTED AA:BB", "PEER_READY nus=1 smp=1 peer_name=MSense4ECG-1 peer_addr=AA:BB:CC:DD:EE:FF"],
            ]
        )
        msense_dfu.ensure_smp_ready(channel, "ecg", 1.0, False)
        self.assertEqual(channel.sent, ["status", "connect ecg"])

    def test_rejects_peer_without_smp(self) -> None:
        channel = FakeCommandChannel(
            [["STATUS state=IDLE subscribed=0 smp=0"], ["PEER_READY nus=1 smp=0"]]
        )
        with self.assertRaises(msense_dfu.DfuError):
            msense_dfu.ensure_smp_ready(channel, "any", 1.0, False)

    def test_replaces_mismatched_smp_peer_before_connecting_target(self) -> None:
        channel = FakeCommandChannel(
            [
                ["STATUS state=READY subscribed=1 smp=1 peer_name=MSense4PPG-1 peer_addr=AA:BB:CC:DD:EE:FF"],
                ["DISCONNECT_SENT", "DISCONNECTED reason=0x13"],
                ["PEER_READY nus=1 smp=1 peer_name=MSense4ECG-1 peer_addr=11:22:33:44:55:66"],
            ]
        )
        msense_dfu.ensure_smp_ready(channel, "ecg", 1.0, False)
        self.assertEqual(channel.sent, ["status", "disconnect", "connect ecg"])

    def test_rejects_wrong_identity_after_target_connect(self) -> None:
        channel = FakeCommandChannel(
            [
                ["STATUS state=IDLE subscribed=0 smp=0"],
                ["PEER_READY nus=1 smp=1 peer_name=MSense4ECG-1 peer_addr=11:22:33:44:55:66"],
            ]
        )
        with self.assertRaises(msense_dfu.DfuError):
            msense_dfu.ensure_smp_ready(channel, "ppg", 1.0, False)


class PortAndExitTests(unittest.TestCase):
    @staticmethod
    def records(serial_number: str) -> list[msense_dfu.PortRecord]:
        return [
            msense_dfu.PortRecord("COM4", "J-Link CDC UART", "", serial_number, "1-2.0", "VCOM0"),
            msense_dfu.PortRecord("COM5", "J-Link CDC UART", "", serial_number, "1-2.1", "VCOM2"),
        ]

    def test_port_pairing_and_ambiguity(self) -> None:
        self.assertEqual(msense_dfu.select_ports(self.records("100"), None, None), ("COM4", "COM5"))
        with self.assertRaises(msense_dfu.DfuError):
            msense_dfu.select_ports(
                self.records("100") + self.records("200"),
                None,
                None,
                nrfutil_loader=lambda: {"devices": []},
            )
        self.assertEqual(
            msense_dfu.select_ports(self.records("100") + self.records("200"), "COM8", "COM9"),
            ("COM8", "COM9"),
        )

    @staticmethod
    def nrfutil_payload() -> dict[str, object]:
        return {
            "devices": [
                {
                    "serialNumber": "001050098667",
                    "traits": {"jlink": True},
                    "serialPorts": [
                        {"comName": "COM4", "vcom": 0},
                        {"comName": "COM5", "vcom": 1},
                    ],
                }
            ]
        }

    def test_nrfutil_vcom_fallback_handles_identical_pyserial_records(self) -> None:
        records = [
            msense_dfu.PortRecord("COM4", "JLink CDC UART Port", "", "001050098667", None, None),
            msense_dfu.PortRecord("COM5", "JLink CDC UART Port", "", "001050098667", None, None),
        ]
        self.assertEqual(
            msense_dfu.select_ports(
                records,
                None,
                None,
                "1050098667",
                nrfutil_loader=self.nrfutil_payload,
            ),
            ("COM4", "COM5"),
        )

    def test_nrfutil_vcom_fallback_rejects_ambiguous_pairs(self) -> None:
        payload = self.nrfutil_payload()
        devices = payload["devices"]
        assert isinstance(devices, list)
        devices.append(
            {
                "serialNumber": "001050098668",
                "traits": {"jlink": True},
                "serialPorts": [
                    {"comName": "COM6", "vcom": 0},
                    {"comName": "COM7", "vcom": 1},
                ],
            }
        )
        with self.assertRaises(msense_dfu.DfuError):
            msense_dfu.parse_nrfutil_vcom_pair(payload, None)

    def test_terminal_exit_mapping(self) -> None:
        self.assertEqual(msense_dfu.classify_exit("DFU_SUCCESS"), msense_dfu.EXIT_SUCCESS)
        self.assertEqual(msense_dfu.classify_exit("DFU_FAIL"), msense_dfu.EXIT_DFU_FAILURE)
        self.assertEqual(msense_dfu.classify_exit("DFU_CREDIT"), msense_dfu.EXIT_PROTOCOL)


if __name__ == "__main__":
    unittest.main()
