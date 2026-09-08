"""Host regression for submitting version-zero commands to known peers."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
GCC = Path(r'C:\cygwin64\bin\gcc.exe')


class VersionZeroCommandTest(unittest.TestCase):
    def test_known_peers_send_zero_and_unknown_peer_is_rejected(self):
        source = (ROOT / 'central_nus_test/src/main.c').read_text()
        definition = re.search(r'static int issue_nus_command\([^;]*?\)\s*\{', source)
        self.assertIsNotNone(definition)
        start = definition.start()
        brace = source.index('{', start)
        end, depth = brace + 1, 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        production = source[start:end]
        protocol = (ROOT / 'shared/include/msense_sensor_stream_protocol.h').read_text()
        version = re.search(r'#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION (\d+)U', protocol)
        self.assertEqual(version.group(1), '0')
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include "msense_sensor_stream_protocol.h"
typedef int k_spinlock_key_t;
struct bt_conn { int unused; };
struct write_params {
 void (*func)(void); uint16_t handle, offset;
 const void *data; size_t length;
};
static struct {
 int lock; bool write_pending, command_pending, subscribed;
 uint8_t peer_device_type, last_command_opcode;
 uint32_t last_command_session_id;
 struct { int64_t request_start_ms; bool request_started; } statistics;
} tester;
static struct { struct { uint16_t rx; } handles; } nus_client;
static struct bt_conn conn;
static uint8_t command_write_data[MSENSE_SENSOR_STREAM_COMMAND_BYTES];
static struct write_params command_write_params;
static int writes, refs_released;
static struct bt_conn *connection_ref(void) { return &conn; }
static void bt_conn_unref(struct bt_conn *c) { (void)c; refs_released++; }
static k_spinlock_key_t k_spin_lock(int *lock) { (void)lock; return 0; }
static void k_spin_unlock(int *lock, k_spinlock_key_t key) { (void)lock; (void)key; }
static int64_t k_uptime_get(void) { return 123; }
static void command_write_complete(void) {}
static void sys_put_le32(uint32_t value, uint8_t *p) {
 for (unsigned i = 0; i < 4; i++) p[i] = value >> (8 * i);
}
static int bt_gatt_write(struct bt_conn *c, struct write_params *p) {
 (void)c;
 const uint8_t *bytes = p->data;
 assert(p->length == 8 && bytes[0] == 0x4d && bytes[1] == 0x53);
 assert(bytes[2] == 0 && bytes[3] == MSENSE_SENSOR_STREAM_OPCODE_START);
 assert(bytes[4] == 0x78 && bytes[5] == 0x56 && bytes[6] == 0x34 && bytes[7] == 0x12);
 writes++; return 0;
}
''' + production + r'''
int main(void) {
 const uint8_t peers[] = {MSENSE_SENSOR_STREAM_DEVICE_PPG, MSENSE_SENSOR_STREAM_DEVICE_ECG};
 nus_client.handles.rx = 1;
 for (unsigned i = 0; i < 2; i++) {
  memset(&tester, 0, sizeof(tester));
  tester.subscribed = true; tester.peer_device_type = peers[i];
  assert(issue_nus_command(MSENSE_SENSOR_STREAM_OPCODE_START, 0x12345678) == 0);
  assert(tester.write_pending && tester.command_pending);
 }
 memset(&tester, 0, sizeof(tester)); tester.subscribed = true;
 assert(issue_nus_command(MSENSE_SENSOR_STREAM_OPCODE_START, 1) == -EPROTONOSUPPORT);
 assert(!tester.write_pending && !tester.command_pending);
 assert(writes == 2 && refs_released == 3);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            generated = directory / 'command.c'
            executable = directory / 'command.exe'
            generated.write_text(harness)
            environment = os.environ.copy()
            environment['PATH'] = str(GCC.parent) + os.pathsep + environment.get('PATH', '')
            compiled = subprocess.run([str(GCC), '-std=c11', '-Wall', '-Wextra', '-Werror',
                                       '-I', str(ROOT / 'shared/include'), str(generated),
                                       '-o', str(executable)], env=environment,
                                      capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            completed = subprocess.run([str(executable)], env=environment,
                                       capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == '__main__':
    unittest.main()
