import os
import pathlib
import re
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SOURCE = ROOT / "central_nus_test" / "src" / "main.c"
GCC = pathlib.Path(r"C:\cygwin64\bin\gcc.exe")


def extract_function(source: str, name: str, return_type: str) -> str:
    matches = re.finditer(
        rf"static\s+{re.escape(return_type)}\s+{re.escape(name)}\s*\(", source
    )
    for match in matches:
        start = match.start()
        brace = source.find("{", start)
        semicolon = source.find(";", start)
        if brace >= 0 and (semicolon < 0 or brace < semicolon):
            break
    else:
        raise AssertionError(f"missing {name}")
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated {name}")


class EcgV0ReceiverHarnessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_cross_boundary_and_pending_validation_paths(self):
        functions = "\n\n".join(
            [
                extract_function(self.source, "elapsed_ms_between", "uint64_t"),
                extract_function(self.source, "phase_statistics_record_data", "void"),
                extract_function(self.source, "stream_statistics_record_data", "void"),
                extract_function(self.source, "all_zero", "bool"),
                extract_function(self.source, "parse_start_ack", "bool"),
                extract_function(self.source, "ecg_validation_pending_locked", "bool"),
                extract_function(self.source, "complete_pending_end", "void"),
                extract_function(self.source, "finish_stream_locked", "void"),
                extract_function(self.source, "issue_deferred_stop", "void"),
                extract_function(self.source, "handle_start_ack", "void"),
                extract_function(self.source, "end_status_is_valid", "bool"),
                extract_function(self.source, "handle_end", "void"),
                extract_function(self.source, "result_status_is_valid", "bool"),
                extract_function(self.source, "handle_result", "void"),
                extract_function(self.source, "ecg_claim_rx_slot_locked", "uint8_t"),
                extract_function(self.source, "ecg_block_validate_work_handler", "void"),
                extract_function(self.source, "handle_data", "void"),
                extract_function(self.source, "stream_progress_work_handler", "void"),
                extract_function(self.source, "handle_subscription_removed", "void"),
                extract_function(self.source, "nus_notification", "uint8_t"),
            ]
        )
        harness = f"""
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ECG_RX_BLOCK_SLOT_COUNT 2U
#define ECG_RX_BLOCK_SLOT_NONE 0xffU
#define ECG_RX_BLOCK_FREE 0
#define ECG_RX_BLOCK_FILLING 1
#define ECG_RX_BLOCK_VALIDATING 2
#define TESTER_START_PENDING 0
#define TESTER_RECEIVING 1
#define TESTER_FINISHING 2
#define TESTER_FAILED 3
#define TESTER_COMPLETE 4
#define TESTER_IDLE 5
#define TESTER_READY 6
#define MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES 8U
#define MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE 4096U
#define MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS 8U
#define MSENSE_SENSOR_STREAM_DEVICE_ECG 2U
#define MSENSE_SENSOR_STREAM_START_ACK_BYTES 16U
#define MSENSE_SENSOR_STREAM_END_BYTES 2U
#define MSENSE_SENSOR_STREAM_HISTORY_UNITS 8U
#define MSENSE_SENSOR_STREAM_FINITE_BYTES 131072U
#define MSENSE_SENSOR_STREAM_STATUS_SUCCESS 0U
#define MSENSE_SENSOR_STREAM_STATUS_STOPPED 8U
#define MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING 1U
#define MSENSE_SENSOR_STREAM_STATUS_STORAGE_ERROR 9U
#define MSENSE_SENSOR_STREAM_STATUS_BUFFER_OVERFLOW 14U
#define MSENSE_SENSOR_STREAM_STATUS_UNSUPPORTED_VERSION 7U
#define MSENSE_SENSOR_STREAM_STATUS_NOT_INITIALIZED 11U
#define MSENSE_SENSOR_STREAM_STATUS_WRONG_SESSION 12U
#define MSENSE_SENSOR_STREAM_OPCODE_STOP 2U
#define MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR 0x000aU
#define MSENSE_SENSOR_STREAM_STATUS_DISCONNECTED 0x000dU
#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION_ECG 0U
#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION_PPG 0U
#define MSENSE_SENSOR_STREAM_DEVICE_PPG 1U
#define MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE 16U
#define MSENSE_SENSOR_STREAM_HISTORY_BYTES 32768U
#define CONFIG_MSENSE_CENTRAL_NO_PROGRESS_TIMEOUT_MS 15000U
#define ECG_DATA_OFFSET_OFFSET 0U
#define ECG_DATA_BYTES_OFFSET MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES
#define DATA_PHASE_HISTORY 0U
#define DATA_PHASE_FORWARD 1U
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#define ARG_UNUSED(value) ((void)(value))
#define BT_HCI_ERR_REMOTE_USER_TERM_CONN 0x13
#define BT_GATT_ITER_STOP 0U
#define BT_GATT_ITER_CONTINUE 1U

typedef int atomic_t;
typedef int k_spinlock_key_t;
struct k_spinlock {{ int unused; }};
struct k_work {{ int unused; }};
struct bt_conn {{ int id; }};
struct bt_gatt_subscribe_params {{ int unused; }};
struct msense_ecg_block_info {{ int unused; }};
struct phase_statistics {{
    int64_t first_data_ms, last_data_ms;
    uint64_t raw_nus_bytes, sensor_bytes, data_notifications;
    uint32_t max_gap_ms;
    bool has_data;
}};
struct stream_statistics {{
    struct phase_statistics total;
    struct phase_statistics history;
    struct phase_statistics forward;
    int64_t request_start_ms;
    bool request_started;
    bool history_reported;
}};
struct stream_metadata {{
    uint64_t total_sensor_bytes;
    uint64_t expected_byte_offset;
    uint64_t validated_records;
    uint64_t skipped_history_blocks;
    uint32_t received_data_messages;
    uint32_t session_id;
    uint16_t record_size;
    uint8_t device_type;
    uint8_t ecg_filling_slot;
    bool infinity;
    bool ecg_previous_block_valid;
    struct msense_ecg_block_info ecg_previous_block;
}};
struct pending_stream_end {{ uint16_t status; }};
struct end_completion_work {{
    struct k_work work;
    uint32_t session_id;
    uint32_t capture_generation;
}};
struct tester_context {{
    struct k_spinlock lock;
    struct bt_conn *conn;
    struct stream_metadata metadata;
    struct stream_statistics statistics;
    int state;
    uint32_t capture_generation;
    uint8_t peer_device_type;
    bool subscribed;
    bool start_infinity;
    bool write_pending;
    bool command_pending;
    bool stop_after_start_ack;
    bool reconnect_after_stream;
    uint16_t att_mtu;
    uint8_t last_command_opcode;
    uint32_t last_command_session_id;
    uint16_t end_status;
    int64_t last_progress_ms;
}};
struct ecg_rx_block_slot {{
    struct k_work work;
    uint8_t data[MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE];
    uint64_t byte_offset;
    uint32_t session_id;
    uint32_t capture_generation;
    atomic_t state;
}};

static struct tester_context tester;
static struct ecg_rx_block_slot ecg_rx_slots[ECG_RX_BLOCK_SLOT_COUNT];
static struct end_completion_work end_completion;
static struct k_work stream_progress_work;
static atomic_t relay_close_after_notification;
static int protocol_failures;
static int submit_calls;
static int validate_calls;
static int validate_result;
static int64_t now_ms;
static bool replace_session_on_unlock;
static struct bt_conn old_connection = {{ .id = 1 }};
static struct bt_conn new_connection = {{ .id = 2 }};
static struct bt_conn *disconnected_connection;
static bool relay_accept;
static bool notification_is_ecg_data;
static int relay_calls;
static int notification_calls;
static int notification_order[2];
static int notification_order_count;
static int command_calls;

static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock) {{ (void)lock; return 0; }}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key) {{
    (void)lock; (void)key;
    if (replace_session_on_unlock) {{
        replace_session_on_unlock = false;
        tester.state = TESTER_RECEIVING;
        tester.metadata.session_id = 99U;
        tester.capture_generation = 9U;
        tester.conn = &new_connection;
    }}
}}
static int atomic_get(atomic_t *value) {{ return *value; }}
static void atomic_set(atomic_t *value, int state) {{ *value = state; }}
static bool atomic_cas(atomic_t *value, int old, int next) {{
    if (*value != old) return false;
    *value = next;
    return true;
}}
static int k_work_submit(struct k_work *work) {{ (void)work; submit_calls++; return 2; }}
static int k_work_reschedule(void *work, int timeout) {{ (void)work; (void)timeout; return 1; }}
#define K_MSEC(value) ((int)(value))
static int64_t k_uptime_get(void) {{ return now_ms; }}
static struct bt_conn *bt_conn_ref(struct bt_conn *conn) {{ return conn; }}
static int bt_conn_disconnect(struct bt_conn *conn, int reason) {{
    (void)reason; disconnected_connection = conn; return 0;
}}
static void bt_conn_unref(struct bt_conn *conn) {{ (void)conn; }}
static uint64_t sys_get_le64(const uint8_t *p) {{
    uint64_t value = 0;
    for (unsigned int i = 0; i < 8; ++i) value |= (uint64_t)p[i] << (8U * i);
    return value;
}}
static void mark_protocol_failure(const char *reason) {{ (void)reason; protocol_failures++; }}
static void post_event(const char *format, ...) {{ (void)format; }}
static int issue_nus_command(uint8_t opcode, uint32_t session_id) {{
    assert(opcode == MSENSE_SENSOR_STREAM_OPCODE_STOP && session_id == 7U);
    command_calls++;
    tester.write_pending = true;
    tester.command_pending = true;
    return 0;
}}
static void post_stream_prefix(uint8_t device, uint32_t id, uint64_t records,
                               uint64_t skipped) {{
    (void)device; (void)id; (void)records; (void)skipped;
}}
static bool notification_for_current_connection(struct bt_conn *conn) {{ return tester.conn == conn; }}
static bool is_data_notification(const uint8_t *data, uint16_t length) {{
    (void)data; (void)length; return notification_is_ecg_data;
}}
static bool relay_enqueue(const uint8_t *data, uint16_t length) {{
    (void)data; (void)length;
    notification_order[notification_order_count++] = 1;
    relay_calls++;
    return relay_accept;
}}
static void handle_notification(const uint8_t *data, uint16_t length, int64_t when) {{
    (void)data; (void)length; (void)when;
    notification_order[notification_order_count++] = 2;
    notification_calls++;
}}
static void relay_request_idle(void) {{}}
static void post_throughput_history(uint32_t id, const struct phase_statistics *stats,
                                    int64_t start, bool started) {{
    (void)id; (void)stats; (void)start; (void)started;
}}
static void stream_progress_note(void) {{}}
static void stream_progress_stop(void) {{}}
static void ecg_release_filling_slot_locked(void) {{
    uint8_t slot = tester.metadata.ecg_filling_slot;
    if (slot < ECG_RX_BLOCK_SLOT_COUNT && ecg_rx_slots[slot].state == ECG_RX_BLOCK_FILLING)
        ecg_rx_slots[slot].state = ECG_RX_BLOCK_FREE;
    tester.metadata.ecg_filling_slot = ECG_RX_BLOCK_SLOT_NONE;
}}
static int msense_ecg_block_validate(const uint8_t *data, struct msense_ecg_block_info *info) {{
    (void)data; (void)info; validate_calls++; return validate_result;
}}
static int msense_ecg_block_validate_continuity(const struct msense_ecg_block_info *a,
                                                 const struct msense_ecg_block_info *b) {{
    (void)a; (void)b; return 0;
}}
static void post_throughput_forward(uint32_t id, const struct phase_statistics *stats) {{
    (void)id; (void)stats;
}}
static void post_throughput_summary(uint32_t id, const struct phase_statistics *stats) {{
    (void)id; (void)stats;
}}
static const char *status_name(uint16_t status) {{ (void)status; return "status"; }}
static void dfu_request_reconnect(void *context) {{ (void)context; }}
static uint16_t sys_get_le16(const uint8_t *p) {{ return p[0] | (uint16_t)p[1] << 8; }}

{functions}

static void reset_receiver(void) {{
    memset(&tester, 0, sizeof(tester));
    memset(ecg_rx_slots, 0, sizeof(ecg_rx_slots));
    tester.state = TESTER_RECEIVING;
    tester.metadata.device_type = MSENSE_SENSOR_STREAM_DEVICE_ECG;
    tester.metadata.record_size = MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE;
    tester.metadata.infinity = true;
    tester.metadata.session_id = 7U;
    tester.metadata.ecg_filling_slot = ECG_RX_BLOCK_SLOT_NONE;
    tester.capture_generation = 3U;
    tester.peer_device_type = MSENSE_SENSOR_STREAM_DEVICE_ECG;
    tester.subscribed = true;
    tester.conn = &old_connection;
    for (unsigned int i = 0; i < ECG_RX_BLOCK_SLOT_COUNT; ++i)
        ecg_rx_slots[i].state = ECG_RX_BLOCK_FREE;
    protocol_failures = submit_calls = validate_calls = 0;
    validate_result = 0;
    now_ms = 0;
    replace_session_on_unlock = false;
    disconnected_connection = NULL;
    relay_accept = true;
    notification_is_ecg_data = false;
    relay_calls = notification_calls = notification_order_count = 0;
    command_calls = 0;
    relay_close_after_notification = 0;
}}

static void put64(uint8_t *p, uint64_t value) {{
    for (unsigned int i = 0; i < 8; ++i) p[i] = (uint8_t)(value >> (8U * i));
}}

static void test_cross_boundary(void) {{
    uint8_t payload[20] = {{0}};
    reset_receiver();
    tester.metadata.expected_byte_offset = 4090U;
    tester.metadata.ecg_filling_slot = 0U;
    ecg_rx_slots[0].state = ECG_RX_BLOCK_FILLING;
    ecg_rx_slots[0].session_id = 7U;
    ecg_rx_slots[0].capture_generation = 3U;
    put64(payload, 4090U);
    for (unsigned int i = 0; i < 12; ++i) payload[8 + i] = (uint8_t)(0xa0U + i);
    handle_data(7U, payload, sizeof(payload), 32U, 1);
    assert(protocol_failures == 0);
    assert(submit_calls == 1);
    assert(ecg_rx_slots[0].state == ECG_RX_BLOCK_VALIDATING);
    assert(ecg_rx_slots[1].state == ECG_RX_BLOCK_FILLING);
    assert(tester.metadata.expected_byte_offset == 4102U);
    assert(memcmp(&ecg_rx_slots[0].data[4090], &payload[8], 6U) == 0);
    assert(memcmp(ecg_rx_slots[1].data, &payload[14], 6U) == 0);
}}

static void test_cross_boundary_backlog_keeps_completed_block_queued(void) {{
    uint8_t payload[20] = {{0}};
    reset_receiver();
    tester.metadata.expected_byte_offset = 4090U;
    tester.metadata.ecg_filling_slot = 0U;
    ecg_rx_slots[0].state = ECG_RX_BLOCK_FILLING;
    ecg_rx_slots[0].session_id = 7U;
    ecg_rx_slots[0].capture_generation = 3U;
    ecg_rx_slots[1].state = ECG_RX_BLOCK_VALIDATING;
    put64(payload, 4090U);
    handle_data(7U, payload, sizeof(payload), 32U, 1);
    assert(protocol_failures == 1);
    assert(submit_calls == 1);
    assert(ecg_rx_slots[0].state == ECG_RX_BLOCK_VALIDATING);
}}

static void test_offsets_and_finite_bounds(void) {{
    uint8_t one_byte[9] = {{0}};
    uint8_t two_bytes[10] = {{0}};
    uint8_t overrun[20] = {{0}};

    reset_receiver();
    tester.metadata.expected_byte_offset = (uint64_t)UINT32_MAX + 8U;
    put64(one_byte, tester.metadata.expected_byte_offset);
    handle_data(7U, one_byte, sizeof(one_byte), 21U, 1);
    assert(protocol_failures == 0);
    assert(tester.metadata.expected_byte_offset == (uint64_t)UINT32_MAX + 9U);

    reset_receiver();
    put64(one_byte, 1U);
    handle_data(7U, one_byte, sizeof(one_byte), 21U, 1);
    assert(protocol_failures == 1);

    reset_receiver();
    tester.metadata.expected_byte_offset = 1U;
    put64(one_byte, 0U);
    handle_data(7U, one_byte, sizeof(one_byte), 21U, 1);
    assert(protocol_failures == 1);

    reset_receiver();
    tester.metadata.infinity = false;
    tester.metadata.total_sensor_bytes = MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE;
    tester.metadata.expected_byte_offset = 4090U;
    put64(overrun, 4090U);
    handle_data(7U, overrun, sizeof(overrun), 32U, 1);
    assert(protocol_failures == 1);
    assert(tester.metadata.expected_byte_offset == 4090U);

    reset_receiver();
    tester.metadata.expected_byte_offset = UINT64_MAX - 1U;
    put64(two_bytes, UINT64_MAX - 1U);
    handle_data(7U, two_bytes, sizeof(two_bytes), 22U, 1);
    assert(protocol_failures == 1);
    assert(tester.metadata.expected_byte_offset == UINT64_MAX - 1U);
}}

static void test_stale_timeout_does_not_touch_new_session(void) {{
    reset_receiver();
    tester.last_progress_ms = 0;
    now_ms = CONFIG_MSENSE_CENTRAL_NO_PROGRESS_TIMEOUT_MS;
    replace_session_on_unlock = true;
    stream_progress_work_handler(NULL);
    assert(disconnected_connection == &old_connection);
    assert(tester.conn == &new_connection);
    assert(tester.state == TESTER_RECEIVING);
    assert(tester.metadata.session_id == 99U);
    assert(tester.capture_generation == 9U);
}}

static void test_finishing_and_validation_failure_prefix(void) {{
    reset_receiver();
    tester.state = TESTER_FINISHING;
    ecg_rx_slots[0].state = ECG_RX_BLOCK_VALIDATING;
    ecg_rx_slots[0].session_id = 7U;
    ecg_rx_slots[0].capture_generation = 3U;
	memset(ecg_rx_slots[0].data, 0x5a, sizeof(ecg_rx_slots[0].data));
    ecg_block_validate_work_handler(&ecg_rx_slots[0].work);
    assert(tester.metadata.validated_records == 1U);
    assert(tester.state == TESTER_FAILED); /* no matching successful byte total */

    reset_receiver();
    tester.state = TESTER_FINISHING;
    tester.metadata.validated_records = 1U;
    for (unsigned int i = 0; i < 2; ++i) {{
        ecg_rx_slots[i].state = ECG_RX_BLOCK_VALIDATING;
        ecg_rx_slots[i].session_id = 7U;
        ecg_rx_slots[i].capture_generation = 3U;
    }}
	memset(ecg_rx_slots[0].data, 0x5a, sizeof(ecg_rx_slots[0].data));
    validate_result = -5;
    ecg_block_validate_work_handler(&ecg_rx_slots[0].work);
    assert(tester.state == TESTER_FAILED);
    assert(tester.state != TESTER_FINISHING);
    assert(tester.metadata.validated_records == 1U);
	assert(disconnected_connection == &old_connection);
    validate_result = 0;
    ecg_block_validate_work_handler(&ecg_rx_slots[1].work);
    assert(tester.metadata.validated_records == 1U);
}}

static void test_zero_history_padding_and_partial_stop(void) {{
    uint8_t block[MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES +
                  MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE] = {{0}};
    uint8_t stopped[2] = {{8U, 0U}};
    uint8_t success[2] = {{0U, 0U}};

    reset_receiver();
    handle_data(7U, block, sizeof(block), sizeof(block), 1);
    assert(protocol_failures == 0 && submit_calls == 1);
    ecg_block_validate_work_handler(&ecg_rx_slots[0].work);
    assert(tester.metadata.skipped_history_blocks == 1U);
    assert(tester.metadata.validated_records == 0U && validate_calls == 0);

    put64(block, MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE);
    memset(&block[MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES], 0x5a,
           MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE);
    handle_data(7U, block, sizeof(block), sizeof(block), 2);
    ecg_block_validate_work_handler(&ecg_rx_slots[0].work);
    assert(tester.metadata.validated_records == 1U);

    put64(block, 2U * MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE);
    memset(&block[MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES], 0,
           MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE);
    validate_result = -5;
    handle_data(7U, block, sizeof(block), sizeof(block), 3);
    ecg_block_validate_work_handler(&ecg_rx_slots[0].work);
    assert(tester.state == TESTER_FAILED && disconnected_connection == &old_connection);

    reset_receiver();
    tester.metadata.infinity = false;
    tester.metadata.total_sensor_bytes = MSENSE_SENSOR_STREAM_FINITE_BYTES;
    tester.metadata.expected_byte_offset = MSENSE_SENSOR_STREAM_FINITE_BYTES;
    tester.metadata.skipped_history_blocks = 8U;
    tester.metadata.validated_records = 24U;
    handle_end(7U, success, sizeof(success));
    complete_pending_end(7U, tester.capture_generation);
    assert(tester.state == TESTER_COMPLETE);

    reset_receiver();
    tester.metadata.expected_byte_offset = MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE + 1U;
    tester.metadata.validated_records = 1U;
    tester.metadata.ecg_filling_slot = 0U;
    ecg_rx_slots[0].state = ECG_RX_BLOCK_FILLING;
    ecg_rx_slots[0].session_id = 7U;
    ecg_rx_slots[0].capture_generation = 3U;
    handle_end(7U, stopped, sizeof(stopped));
    complete_pending_end(7U, tester.capture_generation);
    assert(tester.state == TESTER_COMPLETE);
    assert(ecg_rx_slots[0].state == ECG_RX_BLOCK_FREE);
}}

static void test_subscription_removal_and_relay_order(void) {{
    uint8_t data[12] = {{0}};

    reset_receiver();
    assert(nus_notification(&old_connection, NULL, NULL, 0U) == BT_GATT_ITER_STOP);
    assert(tester.state == TESTER_FINISHING);
    assert(tester.state == TESTER_FINISHING);
    assert(tester.end_status == MSENSE_SENSOR_STREAM_STATUS_DISCONNECTED);
    assert(submit_calls == 1);
    assert(!tester.subscribed && disconnected_connection == &old_connection);

    reset_receiver();
    tester.state = TESTER_FINISHING;
    tester.end_status = 0U;
    assert(nus_notification(&old_connection, NULL, NULL, 0U) == BT_GATT_ITER_STOP);
    assert(tester.state == TESTER_FINISHING);
    assert(tester.state == TESTER_FINISHING);
    assert(tester.end_status == 0U);
    assert(submit_calls == 1);
    assert(!tester.subscribed && disconnected_connection == &old_connection);

    reset_receiver();
    tester.state = TESTER_START_PENDING;
    assert(nus_notification(&old_connection, NULL, NULL, 0U) == BT_GATT_ITER_STOP);
    assert(tester.state == TESTER_FAILED);
    assert(!tester.subscribed && disconnected_connection == &old_connection);

    reset_receiver();
    tester.state = TESTER_COMPLETE;
    assert(nus_notification(&old_connection, NULL, NULL, 0U) == BT_GATT_ITER_STOP);
    assert(tester.state == TESTER_COMPLETE);
    assert(!tester.subscribed && disconnected_connection == &old_connection);

    reset_receiver();
    tester.conn = &new_connection;
    assert(nus_notification(&old_connection, NULL, NULL, 0U) == BT_GATT_ITER_CONTINUE);
    assert(tester.state == TESTER_RECEIVING);
    assert(tester.state != TESTER_FINISHING);

    reset_receiver();
    tester.peer_device_type = MSENSE_SENSOR_STREAM_DEVICE_PPG;
    assert(nus_notification(&old_connection, NULL, NULL, 0U) == BT_GATT_ITER_STOP);
    assert(protocol_failures == 0 && !tester.subscribed);
    assert(tester.state == TESTER_FINISHING && disconnected_connection == &old_connection);

    reset_receiver();
    notification_is_ecg_data = true;
    assert(nus_notification(&old_connection, NULL, data, sizeof(data)) == BT_GATT_ITER_CONTINUE);
    assert(relay_calls == 1 && notification_calls == 1);
    assert(notification_order[0] == 1 && notification_order[1] == 2);

    reset_receiver();
    notification_is_ecg_data = true;
    relay_accept = false;
    assert(nus_notification(&old_connection, NULL, data, sizeof(data)) == BT_GATT_ITER_CONTINUE);
    assert(relay_calls == 1 && notification_calls == 0 && protocol_failures == 1);

    reset_receiver();
    assert(nus_notification(&old_connection, NULL, data, sizeof(data)) == BT_GATT_ITER_CONTINUE);
    assert(relay_calls == 1 && notification_calls == 1);
    assert(notification_order[0] == 2 && notification_order[1] == 1);
}}

static void test_ppg_fragment_prefix_and_long_offsets(void) {{
    uint8_t payload[40] = {{0}};
    reset_receiver();
    tester.metadata.device_type = MSENSE_SENSOR_STREAM_DEVICE_PPG;
    tester.metadata.record_size = 16U;
    handle_data(7U, payload, 23U, 35U, 1); /* 15-byte partial record */
    assert(tester.metadata.expected_byte_offset == 15U);
    assert(tester.metadata.validated_records == 0U && submit_calls == 0);
    put64(payload, 15U);
    handle_data(7U, payload, 26U, 38U, 2); /* two records and a one-byte tail */
    assert(protocol_failures == 0 && tester.metadata.expected_byte_offset == 33U);
    assert(tester.metadata.validated_records == 2U && submit_calls == 0);
    tester.metadata.expected_byte_offset = (uint64_t)UINT32_MAX + 1U;
    put64(payload, tester.metadata.expected_byte_offset);
    handle_data(7U, payload, sizeof(payload), 52U, 3);
    assert(protocol_failures == 0);
    assert(tester.metadata.validated_records == ((uint64_t)UINT32_MAX + 33U) / 16U);
    tester.metadata.infinity = false;
    tester.metadata.total_sensor_bytes = tester.metadata.expected_byte_offset;
    put64(payload, tester.metadata.expected_byte_offset);
    handle_data(7U, payload, 9U, 21U, 4);
    assert(protocol_failures == 1); /* finite byte cap remains strict */
}}

static void test_shared_ack_end_and_rejection(void) {{
    uint8_t ack[16] = {{1U, 8U}};
    uint8_t stopped[2] = {{8U, 0U}};
    uint8_t success[2] = {{0U, 0U}};
    uint8_t rejected[2] = {{2U, 0U}};

    for (unsigned int device = 1U; device <= 2U; ++device) {{
        reset_receiver();
        tester.peer_device_type = device;
        tester.start_infinity = true;
        tester.state = TESTER_START_PENDING;
        handle_start_ack(7U, ack, sizeof(ack));
        assert(protocol_failures == 0 && tester.state == TESTER_RECEIVING);
        assert(tester.metadata.record_size == (device == 1U ? 16U : 4096U));
        handle_end(7U, success, sizeof(success));
        assert(protocol_failures == 1); /* no SUCCESS for INFINITY */
        protocol_failures = 0;
        handle_end(7U, stopped, sizeof(stopped));
        assert(tester.state == TESTER_FINISHING);
        complete_pending_end(7U, tester.capture_generation);
        assert(tester.state == TESTER_COMPLETE); /* STOP during arming: zero bytes */
    }}
    reset_receiver();
    tester.peer_device_type = MSENSE_SENSOR_STREAM_DEVICE_PPG;
    tester.state = TESTER_START_PENDING;
    tester.start_infinity = false;
    handle_start_ack(7U, ack, sizeof(ack));
    assert(protocol_failures == 1); /* mode mismatch */
    ack[0] = 0U;
    put64(&ack[8], MSENSE_SENSOR_STREAM_FINITE_BYTES);
    handle_start_ack(7U, ack, sizeof(ack));
    assert(tester.state == TESTER_RECEIVING);
    tester.metadata.expected_byte_offset = MSENSE_SENSOR_STREAM_FINITE_BYTES;
    tester.metadata.validated_records = MSENSE_SENSOR_STREAM_FINITE_BYTES / 16U;
    handle_end(7U, success, sizeof(success));
    complete_pending_end(7U, tester.capture_generation);
    assert(tester.state == TESTER_COMPLETE);

    reset_receiver();
    tester.metadata.device_type = MSENSE_SENSOR_STREAM_DEVICE_PPG;
    tester.metadata.record_size = 16U;
    tester.metadata.expected_byte_offset = 33U;
    tester.metadata.validated_records = 2U;
    handle_end(7U, stopped, sizeof(stopped));
    complete_pending_end(7U, tester.capture_generation);
    assert(protocol_failures == 0 && tester.state == TESTER_COMPLETE);
    assert(tester.metadata.validated_records == 2U); /* partial tail is discarded */

    reset_receiver();
    tester.metadata.expected_byte_offset = 4096U;
    ecg_rx_slots[0].state = ECG_RX_BLOCK_VALIDATING;
    ecg_rx_slots[0].session_id = 7U;
    ecg_rx_slots[0].capture_generation = 3U;
	memset(ecg_rx_slots[0].data, 0x5a, sizeof(ecg_rx_slots[0].data));
    handle_end(7U, stopped, sizeof(stopped));
    complete_pending_end(7U, 3U);
    assert(tester.state == TESTER_FINISHING); /* END waits for retained complete block */
    ecg_block_validate_work_handler(&ecg_rx_slots[0].work);
    assert(tester.state == TESTER_COMPLETE && tester.metadata.validated_records == 1U);

    reset_receiver();
    tester.state = TESTER_START_PENDING;
    tester.last_command_session_id = 7U;
    tester.last_command_opcode = MSENSE_SENSOR_STREAM_OPCODE_STOP;
    handle_result(8U, rejected, sizeof(rejected));
    assert(tester.state == TESTER_START_PENDING); /* unrelated RESULT is ignored */
    handle_result(7U, rejected, sizeof(rejected));
    assert(tester.state == TESTER_READY); /* rejection after a queued STOP */
}}

static void test_deferred_stop_ack_write_orderings(void) {{
    uint8_t ack[16] = {{1U, 8U}};

    reset_receiver();
    tester.state = TESTER_START_PENDING;
    tester.start_infinity = true;
    tester.stop_after_start_ack = true;
    tester.write_pending = true;
    tester.command_pending = true;
    handle_start_ack(7U, ack, sizeof(ack));
    assert(tester.state == TESTER_RECEIVING && tester.stop_after_start_ack);
    assert(command_calls == 0 && !tester.command_pending);
    tester.write_pending = false;
    issue_deferred_stop();
    assert(command_calls == 1 && !tester.stop_after_start_ack && tester.command_pending);

    reset_receiver();
    tester.state = TESTER_START_PENDING;
    tester.start_infinity = true;
    tester.stop_after_start_ack = true;
    tester.command_pending = true;
    handle_start_ack(7U, ack, sizeof(ack));
    assert(command_calls == 1 && tester.command_pending && tester.write_pending);
}}

static void test_long_stream_statistics_and_phase_crossing(void) {{
    struct stream_statistics stats = {{0}};
    stream_statistics_record_data(&stats, 32760U, 52U, 32U, 1);
    assert(stats.total.sensor_bytes == 32U);
    assert(stats.history.sensor_bytes == 8U && stats.forward.sensor_bytes == 24U);
    assert(stats.history.raw_nus_bytes + stats.forward.raw_nus_bytes == 52U);
    stats.total.sensor_bytes = UINT32_MAX;
    stats.total.raw_nus_bytes = UINT32_MAX;
    stats.total.data_notifications = UINT32_MAX;
    stream_statistics_record_data(&stats, 32792U, 52U, 32U, 2);
    assert(stats.total.sensor_bytes == (uint64_t)UINT32_MAX + 32U);
    assert(stats.total.data_notifications == (uint64_t)UINT32_MAX + 1U);
}}

int main(void) {{
    test_long_stream_statistics_and_phase_crossing();
    test_shared_ack_end_and_rejection();
    test_deferred_stop_ack_write_orderings();
    test_ppg_fragment_prefix_and_long_offsets();
    test_cross_boundary();
    test_cross_boundary_backlog_keeps_completed_block_queued();
	    test_offsets_and_finite_bounds();
    test_stale_timeout_does_not_touch_new_session();
    test_finishing_and_validation_failure_prefix();
	    test_zero_history_padding_and_partial_stop();
    test_subscription_removal_and_relay_order();
    puts("central ECGv0 receiver harness passed");
    return 0;
}}
"""
        with tempfile.TemporaryDirectory() as directory:
            directory = pathlib.Path(directory)
            source_path = directory / "receiver_harness.c"
            executable = directory / "receiver_harness.exe"
            source_path.write_text(harness, encoding="utf-8")
            environment = os.environ.copy()
            environment["PATH"] = str(GCC.parent) + os.pathsep + environment.get("PATH", "")
            compilation = subprocess.run(
                [str(GCC), "-std=c11", "-Wall", "-Werror", str(source_path), "-o", str(executable)],
                env=environment,
                capture_output=True,
                text=True,
            )
            self.assertEqual(compilation.returncode, 0, compilation.stderr)
            completed = subprocess.run(
                [str(executable)], env=environment, capture_output=True, text=True
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("receiver harness passed", completed.stdout)


if __name__ == "__main__":
    unittest.main()
