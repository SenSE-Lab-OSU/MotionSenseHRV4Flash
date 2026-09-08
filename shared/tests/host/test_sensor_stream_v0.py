import os
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
SOURCE = ROOT / "shared" / "sensor_stream.c"
KCONFIG = ROOT / "shared" / "Kconfig"
GCC = pathlib.Path(r"C:\cygwin64\bin\gcc.exe")


def extract_function(source: str, name: str) -> str:
    match = None
    for candidate in re.finditer(rf"\bstatic\s+[^{{;]+?\b{re.escape(name)}\s*\(", source):
        brace = source.find("{", candidate.start())
        semicolon = source.find(";", candidate.start())
        if brace >= 0 and (semicolon < 0 or brace < semicolon):
            match = candidate
            break
    if match is None:
        raise AssertionError(f"missing {name}")
    start = match.start()
    brace = source.find("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated {name}")


def run_harness(source: str, name: str) -> str:
    with tempfile.TemporaryDirectory() as directory:
        directory = pathlib.Path(directory)
        source_path = directory / f"{name}.c"
        executable = directory / f"{name}.exe"
        source_path.write_text(source, encoding="utf-8")
        environment = os.environ.copy()
        environment["PATH"] = str(GCC.parent) + os.pathsep + environment.get("PATH", "")
        compilation = subprocess.run(
            [str(GCC), "-std=c11", "-Wall", "-Werror", str(source_path), "-o", str(executable)],
            env=environment, capture_output=True, text=True,
        )
        if compilation.returncode:
            raise AssertionError(compilation.stderr)
        completed = subprocess.run(
            [str(executable)], env=environment, capture_output=True, text=True
        )
        if completed.returncode:
            raise AssertionError(completed.stderr)
        return completed.stdout


class SensorStreamV0HarnessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_zero_history_snapshot_and_live_wrap(self):
        names = (
            "stream_history_buffer", "stream_snapshot_buffer", "stream_live_buffer",
            "stream_reset_history", "stream_clear_live", "stream_append_history",
            "stream_snapshot_history", "stream_live_pending_bytes",
            "stream_copy_next_data", "stream_consume_data", "stream_append_live",
        )
        code = "\n\n".join(extract_function(self.source, name) for name in names)
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MSENSE_SENSOR_STREAM_HISTORY_BYTES 32U
#define MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES 128U
#define MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR 10U
#define MSENSE_SENSOR_STREAM_STATUS_BUFFER_OVERFLOW 14U
#define MIN(a,b) ((a)<(b)?(a):(b))
struct stream_runtime {
 uint64_t future_enqueued, data_byte_offset; uint32_t history_record_count,
 history_write_index, live_capacity, live_head, live_tail, live_count;
 uint16_t record_size, live_head_offset, terminal_status;
};
static struct stream_runtime stream;
static uint8_t stream_record_buffer[MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES];
static void stream_begin_terminal(uint16_t status) { stream.terminal_status = status; }
''' + code + r'''
int main(void) {
 uint8_t record[4], output[12]; unsigned int i;
 stream.record_size=4; stream.history_record_count=8; stream.live_capacity=16;
 stream_reset_history();
 for(i=1;i<=4;i++){memset(record,(int)i,4);stream_append_history(record);}
 stream_snapshot_history();
 for(i=0;i<16;i++) assert(stream_snapshot_buffer()[i]==0);
 for(i=0;i<16;i++) assert(stream_snapshot_buffer()[16+i]==i/4+1);
 for(i=5;i<=12;i++){memset(record,(int)i,4);stream_append_history(record);}
 stream_snapshot_history();
 for(i=0;i<8;i++) assert(stream_snapshot_buffer()[i*4]==i+5);
 stream_clear_live();
 memset(&stream_live_buffer()[56],41,4); memset(&stream_live_buffer()[60],42,4);
 memset(stream_live_buffer(),43,4); stream.live_head=14; stream.live_tail=1;
 stream.live_count=3; stream.data_byte_offset=32;
 assert(stream_copy_next_data(output,12));
 for(i=0;i<4;i++) assert(output[i]==41 && output[i+4]==42 && output[i+8]==43);
 stream_consume_data(6); assert(stream.live_head==15 && stream.live_head_offset==2);
 stream.live_count=stream.live_capacity; memset(record,77,4);
 assert(!stream_append_live(record)); assert(stream.terminal_status==14);
 puts("history/live passed"); return 0;
}'''
        self.assertIn("passed", run_harness(harness, "history_live"))

    def test_stale_ingress_is_discarded_and_first_terminal_wins(self):
        names = ("stream_begin_terminal", "stream_process_ingress")
        code = "\n\n".join(extract_function(self.source, name) for name in names)
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
struct stream_ingress_slot { uint8_t data[4]; uint32_t generation; uint8_t state; };
struct stream_runtime { uint64_t future_enqueued, future_quota_records; uint32_t acquisition_generation;
 uint16_t terminal_status; bool session_active, terminal_pending, terminal_submitted, infinity; };
static struct stream_runtime stream; static struct stream_ingress_slot slots[2]; static unsigned next_slot;
static int history_count, live_count, release_count;
static struct stream_ingress_slot *stream_take_ingress(void){return next_slot<2?&slots[next_slot++]:0;}
static void stream_release_ingress(struct stream_ingress_slot *slot){(void)slot;release_count++;}
static void stream_append_history(const uint8_t *record){(void)record;history_count++;}
static bool stream_append_live(const uint8_t *record){(void)record;live_count++;stream.future_enqueued++;return true;}
''' + code + r'''
int main(void){
 stream.acquisition_generation=4; stream.session_active=true; stream.infinity=true;
 slots[0].generation=3; slots[1].generation=4;
 assert(stream_process_ingress()); assert(history_count==0 && live_count==0);
 assert(stream_process_ingress()); assert(history_count==1 && live_count==1 && release_count==2);
 stream_begin_terminal(8); stream_begin_terminal(14);
 assert(stream.terminal_pending && stream.terminal_status==8);
 puts("generation/terminal passed"); return 0;
}'''
        self.assertIn("passed", run_harness(harness, "generation_terminal"))

    def test_single_owner_single_tx_contract_is_structural(self):
        self.assertEqual(len(re.findall(r"static struct stream_tx\s+tx\s*;", self.source)), 1)
        self.assertNotIn("stream_tx_slot", self.source)
        self.assertNotIn("frontier", self.source)
        self.assertNotIn("stream_thread", self.source)
        self.assertIn("K_WORK_DELAYABLE_DEFINE(stream_owner_work", self.source)
        self.assertIn("BT_GATT_CHRC_WRITE,", self.source)
        self.assertNotIn("BT_GATT_CHRC_WRITE_WITHOUT_RESP", self.source)
        kconfig = KCONFIG.read_text(encoding="utf-8")
        self.assertNotIn("MSENSE_SENSOR_STREAM_TX_SLOTS", kconfig)
        self.assertNotIn("MSENSE_SENSOR_STREAM_THREAD_STACK_SIZE", kconfig)

    def test_command_admission_releases_only_matching_answer(self):
        code = "\n\n".join(
            extract_function(self.source, name)
            for name in ("stream_command_answered", "stream_forget_command", "stream_rx_write")
        )
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef long ssize_t; typedef int k_spinlock_key_t;
struct k_spinlock { int unused; }; struct bt_conn { int id; }; struct bt_gatt_attr { int unused; };
#define STREAM_COMMAND_START 0U
#define STREAM_COMMAND_START_INFINITY 1U
#define STREAM_COMMAND_STOP 2U
#define STREAM_COMMAND_RESULT 3U
#define STREAM_RESPONSE_NONE 0U
#define STREAM_RESPONSE_ACK 1U
#define STREAM_RESPONSE_END 2U
#define STREAM_RESPONSE_RESULT 3U
#define MSENSE_SENSOR_STREAM_MAGIC0 0x4dU
#define MSENSE_SENSOR_STREAM_MAGIC1 0x53U
#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION 0U
#define MSENSE_SENSOR_STREAM_COMMAND_BYTES 8U
#define MSENSE_SENSOR_STREAM_OPCODE_START 1U
#define MSENSE_SENSOR_STREAM_OPCODE_START_INFINITY 3U
#define MSENSE_SENSOR_STREAM_OPCODE_STOP 2U
#define MSENSE_SENSOR_STREAM_STATUS_INVALID_COMMAND 6U
#define MSENSE_SENSOR_STREAM_STATUS_UNSUPPORTED_VERSION 7U
#define BT_ATT_ERR_INVALID_OFFSET 7
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN 13
#define BT_ATT_ERR_UNLIKELY 14
#define BT_ATT_ERR_INSUFFICIENT_RESOURCES 17
#define BT_GATT_ERR(x) (-(x))
#define ARG_UNUSED(x) ((void)(x))
struct stream_command { uint32_t session_id, connection_generation; uint16_t status; uint8_t kind; };
struct stream_publication { struct k_spinlock lock; struct bt_conn *conn; struct stream_command command;
 uint32_t connection_generation; bool command_pending, command_ready; };
struct stream_runtime { uint32_t response_connection_generation; uint8_t response_kind; };
static struct stream_publication publication; static struct stream_runtime stream; static int wakes;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock){(void)lock;return 0;}
static void k_spin_unlock(struct k_spinlock *lock,k_spinlock_key_t key){(void)lock;(void)key;}
static uint32_t sys_get_le32(const uint8_t *p){return p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void stream_wake(void){wakes++;}
''' + code + r'''
int main(void){
 struct bt_conn conn={0}; struct bt_gatt_attr attr={0};
 uint8_t start[8]={0x4d,0x53,0,1,7,0,0,0};
 publication.conn=&conn; publication.connection_generation=4;
 assert(stream_rx_write(&conn,&attr,start,8,0,0)==8);
 assert(publication.command_pending && publication.command_ready && wakes==1);
 assert(publication.command.kind==STREAM_COMMAND_START);
 assert(stream_rx_write(&conn,&attr,start,8,0,0)==-BT_ATT_ERR_INSUFFICIENT_RESOURCES);
 stream.response_kind=STREAM_RESPONSE_ACK; stream.response_connection_generation=3;
 stream_command_answered(STREAM_RESPONSE_ACK); assert(publication.command_pending);
 stream.response_kind=STREAM_RESPONSE_ACK; stream.response_connection_generation=4;
 stream_command_answered(STREAM_RESPONSE_ACK); assert(!publication.command_pending);
 assert(stream_rx_write(&conn,&attr,start,8,0,0)==8);
 stream_forget_command(3); assert(publication.command_pending);
 stream_forget_command(4); assert(!publication.command_pending);
 start[2]=3;
 assert(stream_rx_write(&conn,&attr,start,8,0,0)==8);
 assert(publication.command.kind==STREAM_COMMAND_RESULT);
 assert(publication.command.status==MSENSE_SENSOR_STREAM_STATUS_UNSUPPORTED_VERSION);
 puts("command admission passed"); return 0;
}'''
        self.assertIn("passed", run_harness(harness, "command_admission"))

    def test_single_slot_submit_completion_and_stale_token(self):
        code = "\n\n".join(
            extract_function(self.source, name)
            for name in (
                "stream_command_answered", "stream_invalidate_tx", "stream_note_busy",
                "stream_submit", "stream_notify_complete",
            )
        )
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int k_spinlock_key_t;
struct k_spinlock { int unused; }; struct bt_conn { int id; }; struct bt_gatt_attr { int unused; };
struct bt_gatt_notify_params { const struct bt_gatt_attr *attr; const void *data; uint16_t len;
 void (*func)(struct bt_conn *, void *); void *user_data; };
#define STREAM_TX_ACK 0U
#define STREAM_TX_DATA 1U
#define STREAM_TX_END 2U
#define STREAM_TX_RESULT 3U
#define STREAM_RESPONSE_NONE 0U
#define STREAM_RESPONSE_ACK 1U
#define STREAM_RESPONSE_END 2U
#define STREAM_RESPONSE_RESULT 3U
#define MSENSE_SENSOR_STREAM_FINITE_BYTES 128U
#define MSENSE_SENSOR_STREAM_STATUS_SUCCESS 0U
#define STREAM_RETRY_DELAY_MS 20U
#define ARG_UNUSED(x) ((void)(x))
struct stream_tx { struct bt_gatt_notify_params params; uint8_t data[64]; uintptr_t token;
 uint32_t session_generation, connection_generation; uint16_t sensor_bytes; uint8_t kind; bool busy; };
struct stream_publication { struct k_spinlock lock; uint32_t connection_generation; bool command_pending; };
struct stream_runtime { struct bt_conn *conn; uint64_t future_quota_records, future_enqueued, data_byte_offset;
 int64_t no_progress_since_ms, retry_after_ms; uint32_t connection_generation, session_generation;
 uint32_t response_connection_generation; uint8_t response_kind;
 bool infinity, ack_pending, ack_complete, terminal_pending, terminal_submitted, terminal_complete, result_pending; };
static struct stream_tx tx; static struct stream_publication publication; static struct stream_runtime stream;
static uintptr_t next_tx_token; static struct bt_gatt_attr attr; static const struct bt_gatt_attr *const STREAM_TX_ATTR=&attr;
static int64_t now_ms=100; static int notify_result; static int consumed, wake_count, disconnects, retires;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock){(void)lock;return 0;}
static void k_spin_unlock(struct k_spinlock *lock,k_spinlock_key_t key){(void)lock;(void)key;}
static int64_t k_uptime_get(void){return now_ms;}
static int bt_gatt_notify_cb(struct bt_conn *conn,struct bt_gatt_notify_params *params){(void)conn;(void)params;return notify_result;}
static void stream_consume_data(uint16_t n){consumed+=n;stream.data_byte_offset+=n;}
static void stream_begin_terminal(uint16_t status){(void)status;stream.terminal_pending=true;}
static void stream_request_disconnect(void){disconnects++;}
static void stream_session_retire(bool invalidate){(void)invalidate;retires++;}
static void stream_wake(void){wake_count++;}
static void stream_notify_complete(struct bt_conn *conn, void *user_data);
''' + code + r'''
int main(void){
 struct bt_conn conn={0}; uintptr_t stale;
 stream.conn=&conn; stream.connection_generation=publication.connection_generation=7;
 stream.session_generation=9; publication.command_pending=true;
 stream.response_kind=STREAM_RESPONSE_ACK; stream.response_connection_generation=7;
 stream.ack_pending=true;
 assert(stream_submit(STREAM_TX_ACK,12,0));
 assert(tx.busy && !stream.ack_pending && !publication.command_pending);
 stale=tx.token; stream_notify_complete(&conn,(void *)(stale+1));
 assert(tx.busy && !stream.ack_complete);
 stream_notify_complete(&conn,(void *)stale);
 assert(!tx.busy && stream.ack_complete && wake_count==1);
 stream.ack_complete=false; stream.ack_pending=true; publication.command_pending=true;
 stream.response_kind=STREAM_RESPONSE_NONE;
 assert(stream_submit(STREAM_TX_DATA,20,5));
 assert(tx.busy && consumed==5 && publication.command_pending);
 stale=tx.token; stream_invalidate_tx(); assert(!tx.busy);
 assert(stream_submit(STREAM_TX_ACK,12,0)); assert(tx.token!=stale && tx.busy);
 stream_notify_complete(&conn,(void *)stale); assert(tx.busy);
 stream_notify_complete(&conn,(void *)tx.token); assert(!tx.busy && stream.ack_complete);
 notify_result=-12; publication.command_pending=true; stream.response_kind=STREAM_RESPONSE_RESULT;
 stream.response_connection_generation=7; stream.result_pending=true;
 assert(!stream_submit(STREAM_TX_RESULT,14,0));
 assert(publication.command_pending && stream.result_pending && stream.retry_after_ms==now_ms+20);
 puts("single slot passed"); return 0;
}'''
        self.assertIn("passed", run_harness(harness, "single_slot"))


if __name__ == "__main__":
    unittest.main()
