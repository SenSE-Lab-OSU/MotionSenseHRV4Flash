"""Focused source contracts for the legacy MSenseDevice HIL control path."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[2]
CENTRAL = ROOT / "central_nus_test/src/main.c"
PERIPHERAL = ROOT / "MSenseDevice/src/BLEService.h"


def function_body(source: str, name: str) -> str:
    match = re.search(rf"static\s+[^;\n]+\s+{name}\([^;]*?\)\s*\{{", source)
    if match is None:
        raise AssertionError(f"missing function {name}")
    brace = source.index("{", match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


class LegacyMsenseControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.central = CENTRAL.read_text()
        cls.peripheral = PERIPHERAL.read_text()

    def test_uuid_suffixes_match_peripheral_header(self):
        pairs = {
            "COLLECTION_CONTROL_SERVICE_UUID": "CONTROL_SERVICE_UUID",
            "COLLECTION_ENABLE_UUID": "WRITE_ENABLE_CHARACTERISTIC_UUID",
            "LEGACY_RESET_UUID": "WRITE_RESET_CHARACTERISTIC_UUID",
            "LEGACY_STATUS_SERVICE_UUID": "STATUS_SERVICE_UUID",
            "LEGACY_STATUS_REGISTER_UUID": "READ_STATUS_REGISTER_UUID",
        }
        for central_name, peripheral_name in pairs.items():
            central = re.search(
                rf"#define {central_name}.*?BT_UUID_128_ENCODE\(0x([0-9a-f]+),",
                self.central,
                re.DOTALL,
            )
            peripheral = re.search(
                rf"#define {peripheral_name}\s+.*?0x([0-9A-F]{{2}}),\s*0xC9,\s*0x39,\s*0xDA",
                self.peripheral,
                re.DOTALL,
            )
            self.assertIsNotNone(central, central_name)
            self.assertIsNotNone(peripheral, peripheral_name)
            # Header arrays are little-endian; byte 12 is the low byte of word 1.
            self.assertEqual(int(central.group(1), 16) & 0xFF,
                             int(peripheral.group(1), 16))

    def test_missing_nus_still_discovers_legacy_services(self):
        unavailable = function_body(self.central, "discovery_service_not_found")
        post_nus = function_body(self.central, "start_post_nus_discovery")
        self.assertIn("start_post_nus_discovery(conn)", unavailable)
        self.assertIn("start_collection_control_discovery(conn)", post_nus)
        self.assertIn("MSENSE_SENSOR_STREAM_DEVICE_PPG", post_nus)

    def test_control_discovery_includes_reset_and_chains_status(self):
        complete = function_body(self.central, "collection_control_discovery_complete")
        self.assertIn("collection_enable_uuid", complete)
        self.assertIn("legacy_reset_uuid", complete)
        self.assertIn("start_legacy_status_discovery(conn)", complete)
        status = function_body(self.central, "legacy_status_discovery_complete")
        self.assertIn("legacy_status_register_uuid", status)
        self.assertIn("legacy_discovery_finished(conn)", status)

    def test_reset_command_is_restricted_and_reconnect_is_bounded(self):
        handler = function_body(self.central, "handle_command")
        self.assertRegex(handler, r"value != 121U && value != 132U")
        reset = function_body(self.central, "issue_legacy_reset")
        self.assertIn("bt_gatt_write(connection, &reset_write_params)", reset)
        self.assertIn("legacy_reset_reconnect_timeout", reset)
        timeout = function_body(self.central, "legacy_reset_reconnect_timeout_handler")
        self.assertIn("RESET_RECONNECT_TIMEOUT", timeout)

    def test_machine_readable_reset_phases_are_present(self):
        for event in (
            "RESET_ATT", "RESET_DISCONNECTED", "RESET_ADVERTISING",
            "RESET_RECONNECTED", "RESET_REDISCOVERED", "REMOTE_STATUS",
        ):
            self.assertIn(f'"{event}', self.central)

    def test_att_error_before_disconnect_keeps_reset_tracking(self):
        callback = function_body(self.central, "reset_write_complete")
        error_path = callback[callback.index("if (current && err != 0U)"):]
        self.assertIn("reset_waiting_for_disconnect = true", error_path)
        self.assertIn("LEGACY_RESET_DISCONNECT_GRACE", error_path)
        self.assertNotIn("reset_reconnect_pending = false", error_path)

        disconnected = function_body(self.central, "disconnected")
        reset_path = disconnected[disconnected.index("if (reset_reconnect)"):]
        self.assertIn("LEGACY_RESET_RECONNECT_TIMEOUT", reset_path)
        self.assertLess(reset_path.index("LEGACY_RESET_RECONNECT_TIMEOUT"),
                        reset_path.index("start_scan()"))

    def test_unsolicited_nus_is_ignored_before_stream_ownership(self):
        callback = function_body(self.central, "nus_notification")
        guard = callback.index("atomic_get(&binary_port_mode) != BINARY_PORT_NUS_RELAY")
        parser = callback.index("stream_data = is_data_notification")
        relay = callback.index("relay_enqueue")
        self.assertLess(guard, parser)
        self.assertLess(guard, relay)
        guarded_path = callback[guard:parser]
        self.assertIn("NUS_UNSOLICITED_IGNORED", guarded_path)
        self.assertIn("return BT_GATT_ITER_CONTINUE", guarded_path)


if __name__ == "__main__":
    unittest.main()
