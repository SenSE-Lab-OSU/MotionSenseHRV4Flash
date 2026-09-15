"""Focused source contracts for the PPG BLE reset characteristic."""

from pathlib import Path
import re
import unittest


SOURCE = Path(__file__).resolve().parents[2] / "src/BLEService.c"


def function_body(source: str, name: str) -> str:
    match = re.search(rf"(?:static\s+)?[^;\n]+\s+{name}\([^;]*?\)\s*\{{", source)
    if match is None:
        raise AssertionError(f"missing function {name}")
    brace = source.index("{", match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def reset_branch(body: str, code: int, next_code: int | None) -> str:
    start = body.index(f"val == {code}")
    end = body.index(f"val == {next_code}", start) if next_code is not None else body.index(
        "} else {", start)
    return body[start:end]


class PpgBleResetControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text()
        cls.callback = function_body(cls.source, "bt_reset")

    def test_safe_reset_dispatches_one_storage_reboot_without_direct_reset(self):
        branch = reset_branch(self.callback, 120, 132)
        self.assertEqual(branch.count("request_ppg_storage_reboot()"), 1)
        self.assertNotIn("NVIC_SystemReset", branch)

    def test_existing_reset_codes_retain_their_dispatch(self):
        self.assertIn("request_ppg_storage_reset(false)", reset_branch(self.callback, 68, 120))
        self.assertIn("request_ppg_storage_reset(true)", reset_branch(self.callback, 132, 121))
        emergency = reset_branch(self.callback, 121, None)
        self.assertEqual(emergency.count("NVIC_SystemReset()"), 1)
        self.assertNotIn("request_ppg_storage_", emergency)

    def test_only_accepted_storage_requests_disconnect_and_stop_advertising(self):
        rejection = self.callback.index("if (ret != 0)")
        gatt_error = self.callback.index("BT_GATT_ERR(BT_ATT_ERR_UNLIKELY)", rejection)
        disconnect = self.callback.index("bt_conn_disconnect", rejection)
        advertising_stop = self.callback.index("bt_le_adv_stop", disconnect)
        self.assertLess(gatt_error, disconnect)
        self.assertLess(disconnect, advertising_stop)

    def test_storage_request_rejects_faulted_unready_busy_and_pending_states(self):
        request = function_body(self.source, "request_ppg_storage_action")
        self.assertRegex(request, r"(?s)ppg_collection_faulted\(\).*?return -EIO")
        self.assertRegex(request, r"(?s)ppg_collection_runtime_ready.*?return -EAGAIN")
        self.assertRegex(request, r"(?s)collecting_data \|\| host_wants_collection.*?return -EBUSY")
        self.assertRegex(request, r"(?s)!atomic_cas.*?return -EBUSY")
        self.assertNotIn("NVIC_SystemReset", request)


if __name__ == "__main__":
    unittest.main()
