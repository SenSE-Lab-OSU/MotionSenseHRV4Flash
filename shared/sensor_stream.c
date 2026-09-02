/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "msense_sensor_stream.h"

LOG_MODULE_REGISTER(msense_sensor_stream, LOG_LEVEL_INF);

#define STREAM_ATT_NOTIFY_OVERHEAD 3U
#define STREAM_REQUIRED_ATT_MTU 128U
#define STREAM_TX_SLOT_BYTES (CONFIG_BT_L2CAP_TX_MTU - STREAM_ATT_NOTIFY_OVERHEAD)
#define STREAM_COMMAND_QUEUE_DEPTH 4U
#define STREAM_RESULT_QUEUE_DEPTH 4U
#define STREAM_RETRY_DELAY_MS 20U
#define STREAM_MAX_RETRIES 100U
#define STREAM_THREAD_PRIORITY 8

BUILD_ASSERT(MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE *
		     MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS == 32768U,
	     "PPG history geometry must be 32 KiB");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE *
		     MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS == 65536U,
	     "PPG forward geometry must be 64 KiB");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE *
		     (MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS +
		      MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS) ==
		     MSENSE_SENSOR_STREAM_SENSOR_BYTES,
	     "PPG total geometry must be 96 KiB");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE *
		     MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS == 32772U,
	     "ECG history geometry must be 32772 bytes");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE *
		     MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS == 65532U,
	     "ECG forward geometry must be 65532 bytes");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE *
		     (MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS +
		      MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS) ==
		     MSENSE_SENSOR_STREAM_SENSOR_BYTES,
	     "ECG total geometry must be 96 KiB");
BUILD_ASSERT(CONFIG_BT_L2CAP_TX_MTU >= STREAM_REQUIRED_ATT_MTU,
	     "The local NUS stream MTU must support the protocol minimum");
BUILD_ASSERT(STREAM_TX_SLOT_BYTES >=
		     MSENSE_SENSOR_STREAM_HEADER_BYTES + MSENSE_SENSOR_STREAM_START_ACK_BYTES,
	     "A TX slot must hold START_ACK");

enum stream_command_kind {
	STREAM_COMMAND_START,
	STREAM_COMMAND_CANCEL,
	STREAM_COMMAND_RESULT,
};

struct stream_command {
	uint8_t kind;
	uint16_t result_status;
	uint32_t session_id;
	uint32_t connection_generation;
};

struct stream_pending_result {
	uint16_t status;
	uint8_t state;
	uint32_t session_id;
	uint32_t connection_generation;
};

struct stream_tx_slot {
	struct bt_gatt_notify_params params;
	uint8_t data[STREAM_TX_SLOT_BYTES];
	atomic_t in_use;
	uint32_t session_generation;
	bool history_data;
	bool terminal_end;
};

struct stream_runtime {
	struct k_spinlock lock;
	struct bt_conn *conn;
	uint8_t device_id[8];
	char device_name[17];
	char git_commit[41];
	uint32_t connection_generation;
	uint32_t session_generation;
	uint32_t record_rate_numerator;
	uint32_t record_rate_denominator;
	uint32_t history_record_count;
	uint32_t forward_record_count;
	uint32_t history_count;
	uint32_t history_write_index;
	uint32_t frozen_history_start;
	uint32_t history_tx_index;
	uint32_t history_data_inflight;
	uint32_t forward_count;
	uint32_t forward_tx_index;
	uint32_t data_sequence;
	uint32_t data_message_count;
	uint32_t session_id;
	uint32_t retry_count;
	uint32_t terminal_history_sent;
	uint32_t terminal_forward_captured;
	uint32_t terminal_bytes_sent;
	uint32_t terminal_data_messages;
	uint16_t record_size;
	uint16_t terminal_status;
	int32_t terminal_detail;
	uint8_t device_type;
	uint8_t record_format_version;
	uint8_t device_name_len;
	uint8_t git_tree_state;
	uint8_t state;
	uint8_t result_head;
	uint8_t result_tail;
	uint8_t result_count;
	bool initialized;
	bool recording;
	bool notifications_enabled;
	bool history_collecting;
	bool history_tx_finished;
	bool session_active;
	bool start_ack_pending;
	bool terminal_pending;
	bool terminal_submitted;
	bool terminal_complete;
	struct stream_pending_result results[STREAM_RESULT_QUEUE_DEPTH];
};

/* Keep the large zero-initialized payload buffer in .bss, not flash-backed .data. */
static uint8_t stream_record_buffer[MSENSE_SENSOR_STREAM_SENSOR_BYTES];

static struct stream_runtime stream = {
	.state = MSENSE_SENSOR_STREAM_STATE_UNINITIALIZED,
};

static struct stream_tx_slot tx_slots[CONFIG_MSENSE_SENSOR_STREAM_TX_SLOTS];

K_MSGQ_DEFINE(stream_command_queue, sizeof(struct stream_command),
	      STREAM_COMMAND_QUEUE_DEPTH, 4);
K_SEM_DEFINE(stream_wake, 0, 1);

static ssize_t stream_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset,
			       uint8_t flags);
static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void stream_notify_complete(struct bt_conn *conn, void *user_data);
static void stream_thread(void *arg1, void *arg2, void *arg3);

BT_GATT_SERVICE_DEFINE(msense_sensor_stream_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX_CHAR, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX_CHAR,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, stream_rx_write, NULL),
);

#define STREAM_TX_ATTR (&msense_sensor_stream_svc.attrs[2])

static void stream_retry_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_sem_give(&stream_wake);
}

K_WORK_DELAYABLE_DEFINE(stream_retry_work, stream_retry_work_handler);
K_THREAD_DEFINE(msense_sensor_stream_thread_id,
		CONFIG_MSENSE_SENSOR_STREAM_THREAD_STACK_SIZE, stream_thread,
		NULL, NULL, NULL, STREAM_THREAD_PRIORITY, 0, 0);

static void stream_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn *held_conn;
	struct bt_conn *discard_conn = NULL;
	uint32_t connection_generation;
	k_spinlock_key_t key;

	if (err != 0U) {
		return;
	}

	held_conn = bt_conn_ref(conn);
	key = k_spin_lock(&stream.lock);
	if (stream.conn == NULL) {
		stream.conn = held_conn;
		held_conn = NULL;
		stream.connection_generation++;
		stream.notifications_enabled = false;
	} else if (stream.conn != conn) {
		discard_conn = stream.conn;
		stream.conn = held_conn;
		held_conn = NULL;
		stream.connection_generation++;
		stream.notifications_enabled = false;
		stream.session_generation++;
		stream.session_active = false;
		stream.start_ack_pending = false;
		stream.terminal_pending = false;
		stream.terminal_submitted = false;
		stream.terminal_complete = false;
		stream.history_count = 0U;
		stream.history_write_index = 0U;
		stream.forward_count = 0U;
		stream.history_collecting = stream.recording;
		stream.state = stream.recording ? MSENSE_SENSOR_STREAM_STATE_HISTORY_FILLING :
							  MSENSE_SENSOR_STREAM_STATE_NOT_RECORDING;
	}
	connection_generation = stream.connection_generation;
	k_spin_unlock(&stream.lock, key);

	if (held_conn != NULL) {
		bt_conn_unref(held_conn);
	}
	if (discard_conn != NULL) {
		bt_conn_unref(discard_conn);
	}
	LOG_INF("NUS stream connection ready: generation %u", connection_generation);
	k_sem_give(&stream_wake);
}

static void stream_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *discard_conn = NULL;
	uint32_t connection_generation = 0U;
	k_spinlock_key_t key;

	key = k_spin_lock(&stream.lock);
	if (stream.conn == conn) {
		discard_conn = stream.conn;
		stream.conn = NULL;
		stream.connection_generation++;
		stream.notifications_enabled = false;
		stream.session_generation++;
		stream.session_active = false;
		stream.start_ack_pending = false;
		stream.terminal_pending = false;
		stream.terminal_submitted = false;
		stream.terminal_complete = false;
		stream.history_count = 0U;
		stream.history_write_index = 0U;
		stream.forward_count = 0U;
		stream.history_collecting = stream.recording;
		stream.state = stream.recording ? MSENSE_SENSOR_STREAM_STATE_HISTORY_FILLING :
							  MSENSE_SENSOR_STREAM_STATE_NOT_RECORDING;
		connection_generation = stream.connection_generation;
	}
	k_spin_unlock(&stream.lock, key);

	if (discard_conn != NULL) {
		bt_conn_unref(discard_conn);
		LOG_INF("NUS stream disconnected: reason 0x%02x, generation %u", reason,
			connection_generation);
	}
	k_sem_give(&stream_wake);
}

static struct bt_conn_cb stream_conn_callbacks = {
	.connected = stream_connected,
	.disconnected = stream_disconnected,
};

static bool stream_commit_is_valid(const char *commit)
{
	size_t i;

	if (commit == NULL) {
		return false;
	}

	for (i = 0U; i < 40U; i++) {
		if (!((commit[i] >= '0' && commit[i] <= '9') ||
		      (commit[i] >= 'a' && commit[i] <= 'f'))) {
			return false;
		}
	}

	return commit[40] == '\0';
}

static bool stream_tree_state(const char *tree_state, uint8_t *value)
{
	if (tree_state == NULL || value == NULL) {
		return false;
	}
	if (strcmp(tree_state, "clean") == 0) {
		*value = 0U;
		return true;
	}
	if (strcmp(tree_state, "dirty") == 0) {
		*value = 1U;
		return true;
	}
	if (strcmp(tree_state, "unknown") == 0) {
		*value = 2U;
		return true;
	}

	return false;
}

static bool stream_config_is_valid(const struct msense_sensor_stream_config *config,
					   uint8_t *tree_state)
{
	uint64_t total_bytes;

	if (config == NULL || config->device_id == NULL || config->device_name == NULL ||
	    config->device_name_len == 0U || config->device_name_len > 16U ||
	    config->record_size == 0U || config->record_rate_numerator == 0U ||
	    config->record_rate_denominator == 0U || config->history_record_count == 0U ||
	    config->forward_record_count == 0U || config->record_format_version != 1U ||
	    !stream_commit_is_valid(config->git_commit) ||
	    !stream_tree_state(config->git_tree_state, tree_state)) {
		return false;
	}

	total_bytes = (uint64_t)config->record_size *
		      ((uint64_t)config->history_record_count + config->forward_record_count);
	if (total_bytes != MSENSE_SENSOR_STREAM_SENSOR_BYTES) {
		return false;
	}

	if (config->device_type == MSENSE_SENSOR_STREAM_DEVICE_PPG) {
		return config->record_size == MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE &&
		       config->record_rate_numerator == 256U &&
		       config->record_rate_denominator == 1U &&
		       config->history_record_count ==
			       MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS &&
		       config->forward_record_count ==
			       MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS;
	}
	if (config->device_type == MSENSE_SENSOR_STREAM_DEVICE_ECG) {
		return config->record_size == MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE &&
		       config->record_rate_numerator == 512U &&
		       config->record_rate_denominator == 1U &&
		       config->history_record_count ==
			       MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS &&
		       config->forward_record_count ==
			       MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS;
	}

	return false;
}

static uint8_t *stream_history_buffer(void)
{
	return stream_record_buffer;
}

static uint8_t *stream_forward_buffer(void)
{
	return &stream_record_buffer[stream.history_record_count * stream.record_size];
}

static enum msense_sensor_stream_state stream_post_terminal_state_locked(void)
{
	if (!stream.recording) {
		return MSENSE_SENSOR_STREAM_STATE_NOT_RECORDING;
	}
	if (stream.history_count == stream.history_record_count) {
		return MSENSE_SENSOR_STREAM_STATE_READY;
	}

	return MSENSE_SENSOR_STREAM_STATE_HISTORY_FILLING;
}

static void stream_reset_history_locked(void)
{
	stream.history_count = 0U;
	stream.history_write_index = 0U;
	stream.history_collecting = stream.recording;
}

static void stream_reset_session_locked(void)
{
	stream.session_generation++;
	stream.session_active = false;
	stream.start_ack_pending = false;
	stream.terminal_pending = false;
	stream.terminal_submitted = false;
	stream.terminal_complete = false;
	stream.history_tx_finished = false;
	stream.history_data_inflight = 0U;
	stream.frozen_history_start = 0U;
	stream.history_tx_index = 0U;
	stream.forward_count = 0U;
	stream.forward_tx_index = 0U;
	stream.data_sequence = 0U;
	stream.data_message_count = 0U;
	stream.retry_count = 0U;
	stream_reset_history_locked();
	stream.state = stream_post_terminal_state_locked();
}

static void stream_queue_result_locked(uint32_t session_id, uint16_t status,
				       uint32_t connection_generation)
{
	struct stream_pending_result *result;

	if (stream.result_count == STREAM_RESULT_QUEUE_DEPTH) {
		return;
	}

	result = &stream.results[stream.result_tail];
	result->status = status;
	result->state = stream.state;
	result->session_id = session_id;
	result->connection_generation = connection_generation;
	stream.result_tail = (stream.result_tail + 1U) % STREAM_RESULT_QUEUE_DEPTH;
	stream.result_count++;
}

static void stream_begin_terminal_locked(uint16_t status, int32_t detail,
					 bool reset_for_fresh_history)
{
	if (!stream.session_active || stream.terminal_pending || stream.terminal_submitted) {
		return;
	}

	stream.terminal_status = status;
	stream.terminal_detail = detail;
	stream.terminal_history_sent = stream.history_tx_index;
	stream.terminal_forward_captured = stream.forward_count;
	stream.terminal_bytes_sent =
		(stream.history_tx_index + stream.forward_tx_index) * stream.record_size;
	stream.terminal_data_messages = stream.data_message_count;
	stream.terminal_pending = true;
	stream.state = MSENSE_SENSOR_STREAM_STATE_ABORTING;

	if (reset_for_fresh_history) {
		stream_reset_history_locked();
	}
}

static void stream_maybe_start_fresh_history_locked(void)
{
	if (!stream.session_active || stream.state != MSENSE_SENSOR_STREAM_STATE_ACTIVE ||
	    stream.history_collecting || !stream.history_tx_finished ||
	    stream.history_data_inflight != 0U ||
	    stream.forward_count != stream.forward_record_count) {
		return;
	}

	stream_reset_history_locked();
}

static void stream_append_history_locked(const uint8_t *record)
{
	uint8_t *destination;

	if (!stream.history_collecting) {
		return;
	}

	if (stream.history_count < stream.history_record_count) {
		destination = &stream_history_buffer()[stream.history_count * stream.record_size];
		memcpy(destination, record, stream.record_size);
		stream.history_count++;
		if (stream.history_count == stream.history_record_count &&
		    stream.state == MSENSE_SENSOR_STREAM_STATE_HISTORY_FILLING) {
			stream.state = MSENSE_SENSOR_STREAM_STATE_READY;
		}
		return;
	}

	destination = &stream_history_buffer()[stream.history_write_index * stream.record_size];
	memcpy(destination, record, stream.record_size);
	stream.history_write_index++;
	if (stream.history_write_index == stream.history_record_count) {
		stream.history_write_index = 0U;
	}
}

static struct bt_conn *stream_connection_ref(bool require_notifications,
					      uint32_t *connection_generation)
{
	struct bt_conn *conn = NULL;
	k_spinlock_key_t key;

	key = k_spin_lock(&stream.lock);
	if (stream.conn != NULL &&
	    (!require_notifications || stream.notifications_enabled)) {
		conn = bt_conn_ref(stream.conn);
		*connection_generation = stream.connection_generation;
	}
	k_spin_unlock(&stream.lock, key);

	return conn;
}

static struct stream_tx_slot *stream_claim_tx_slot(void)
{
	uint8_t i;

	for (i = 0U; i < ARRAY_SIZE(tx_slots); i++) {
		if (atomic_cas(&tx_slots[i].in_use, 0, 1)) {
			return &tx_slots[i];
		}
	}

	return NULL;
}

static void stream_release_tx_slot(struct stream_tx_slot *slot)
{
	slot->history_data = false;
	slot->terminal_end = false;
	atomic_clear(&slot->in_use);
}

static void stream_write_header(uint8_t *message, uint8_t message_type,
				uint32_t session_id, uint16_t payload_len)
{
	message[0] = MSENSE_SENSOR_STREAM_MAGIC0;
	message[1] = MSENSE_SENSOR_STREAM_MAGIC1;
	message[2] = MSENSE_SENSOR_STREAM_PROTOCOL_VERSION;
	message[3] = message_type;
	sys_put_le32(session_id, &message[4]);
	sys_put_le16(payload_len, &message[8]);
	sys_put_le16(0U, &message[10]);
}

static int stream_submit_slot(struct bt_conn *conn, struct stream_tx_slot *slot,
			      uint16_t length)
{
	memset(&slot->params, 0, sizeof(slot->params));
	slot->params.attr = STREAM_TX_ATTR;
	slot->params.data = slot->data;
	slot->params.len = length;
	slot->params.func = stream_notify_complete;
	slot->params.user_data = slot;

	return bt_gatt_notify_cb(conn, &slot->params);
}

static bool stream_schedule_retry(int error)
{
	bool retryable = error == -ENOMEM || error == -EAGAIN;
	bool retry = retryable;
	k_spinlock_key_t key;

	if (!retry) {
		return false;
	}

	key = k_spin_lock(&stream.lock);
	stream.retry_count++;
	if (stream.retry_count > STREAM_MAX_RETRIES) {
		stream.retry_count = 0U;
		retry = false;
	}
	k_spin_unlock(&stream.lock, key);

	if (retry) {
		(void)k_work_reschedule(&stream_retry_work, K_MSEC(STREAM_RETRY_DELAY_MS));
	} else if (retryable) {
		LOG_ERR("NUS stream TX retry budget exhausted");
	}

	return retry;
}

static void stream_note_submit_success(void)
{
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	stream.retry_count = 0U;
	k_spin_unlock(&stream.lock, key);
}

static void stream_notify_complete(struct bt_conn *conn, void *user_data)
{
	struct stream_tx_slot *slot = user_data;
	k_spinlock_key_t key;

	ARG_UNUSED(conn);

	key = k_spin_lock(&stream.lock);
	if (slot->session_generation == stream.session_generation && stream.session_active) {
		if (slot->history_data && stream.history_data_inflight != 0U) {
			stream.history_data_inflight--;
			stream_maybe_start_fresh_history_locked();
		}
		if (slot->terminal_end && stream.terminal_submitted) {
			stream.terminal_complete = true;
		}
	}
	k_spin_unlock(&stream.lock, key);

	stream_release_tx_slot(slot);
	k_sem_give(&stream_wake);
}

static bool stream_finalize_terminal(void)
{
	bool complete = false;
	k_spinlock_key_t key;

	key = k_spin_lock(&stream.lock);
	if (stream.session_active && stream.terminal_submitted && stream.terminal_complete) {
		stream.session_active = false;
		stream.start_ack_pending = false;
		stream.terminal_pending = false;
		stream.terminal_submitted = false;
		stream.terminal_complete = false;
		stream.history_tx_finished = false;
		stream.history_data_inflight = 0U;
		stream.frozen_history_start = 0U;
		stream.history_tx_index = 0U;
		stream.forward_count = 0U;
		stream.forward_tx_index = 0U;
		stream.data_sequence = 0U;
		stream.data_message_count = 0U;
		stream.state = stream_post_terminal_state_locked();
		complete = true;
	}
	k_spin_unlock(&stream.lock, key);

	return complete;
}

static int stream_send_start_ack(void)
{
	struct stream_tx_slot *slot;
	struct bt_conn *conn;
	uint32_t connection_generation;
	uint32_t session_generation;
	uint32_t session_id;
	uint16_t message_len = MSENSE_SENSOR_STREAM_HEADER_BYTES +
			       MSENSE_SENSOR_STREAM_START_ACK_BYTES;
	int ret;
	k_spinlock_key_t key;

	conn = stream_connection_ref(true, &connection_generation);
	if (conn == NULL) {
		return 0;
	}

	slot = stream_claim_tx_slot();
	if (slot == NULL) {
		bt_conn_unref(conn);
		return 0;
	}

	key = k_spin_lock(&stream.lock);
	if (!stream.session_active || !stream.start_ack_pending ||
	    connection_generation != stream.connection_generation) {
		k_spin_unlock(&stream.lock, key);
		stream_release_tx_slot(slot);
		bt_conn_unref(conn);
		return 1;
	}

	session_generation = stream.session_generation;
	session_id = stream.session_id;
	stream_write_header(slot->data, MSENSE_SENSOR_STREAM_MESSAGE_START_ACK, session_id,
			    MSENSE_SENSOR_STREAM_START_ACK_BYTES);
	slot->data[12] = stream.device_type;
	slot->data[13] = stream.record_format_version;
	sys_put_le16(stream.record_size, &slot->data[14]);
	sys_put_le32(stream.record_rate_numerator, &slot->data[16]);
	sys_put_le32(stream.record_rate_denominator, &slot->data[20]);
	sys_put_le32(stream.history_record_count, &slot->data[24]);
	sys_put_le32(stream.forward_record_count, &slot->data[28]);
	sys_put_le32(MSENSE_SENSOR_STREAM_SENSOR_BYTES, &slot->data[32]);
	memcpy(&slot->data[36], stream.device_id, sizeof(stream.device_id));
	slot->data[44] = stream.device_name_len;
	memset(&slot->data[45], 0, 16U);
	memcpy(&slot->data[45], stream.device_name, slot->data[44]);
	memcpy(&slot->data[61], stream.git_commit, 40U);
	slot->data[101] = stream.git_tree_state;
	memset(&slot->data[102], 0, 6U);
	slot->session_generation = session_generation;
	slot->history_data = false;
	slot->terminal_end = false;
	k_spin_unlock(&stream.lock, key);

	ret = stream_submit_slot(conn, slot, message_len);
	bt_conn_unref(conn);
	if (ret != 0) {
		stream_release_tx_slot(slot);
		if (!stream_schedule_retry(ret)) {
			key = k_spin_lock(&stream.lock);
			if (stream.session_active && stream.session_generation == session_generation) {
				stream_reset_session_locked();
			}
			k_spin_unlock(&stream.lock, key);
			LOG_WRN("START_ACK submit failed: %d", ret);
			return 1;
		}
		return 0;
	}

	key = k_spin_lock(&stream.lock);
	if (stream.session_active && stream.session_generation == session_generation) {
		stream.start_ack_pending = false;
	}
	k_spin_unlock(&stream.lock, key);
	stream_note_submit_success();
	return 1;
}

static int stream_send_terminal(void)
{
	struct stream_tx_slot *slot;
	struct bt_conn *conn;
	uint32_t connection_generation;
	uint32_t session_generation;
	uint32_t session_id;
	uint16_t message_len = MSENSE_SENSOR_STREAM_HEADER_BYTES +
			       MSENSE_SENSOR_STREAM_END_BYTES;
	int ret;
	k_spinlock_key_t key;

	conn = stream_connection_ref(true, &connection_generation);
	if (conn == NULL) {
		return 0;
	}

	slot = stream_claim_tx_slot();
	if (slot == NULL) {
		bt_conn_unref(conn);
		return 0;
	}

	key = k_spin_lock(&stream.lock);
	if (!stream.session_active || !stream.terminal_pending || stream.terminal_submitted ||
	    connection_generation != stream.connection_generation) {
		k_spin_unlock(&stream.lock, key);
		stream_release_tx_slot(slot);
		bt_conn_unref(conn);
		return 1;
	}

	session_generation = stream.session_generation;
	session_id = stream.session_id;
	stream_write_header(slot->data, MSENSE_SENSOR_STREAM_MESSAGE_END, session_id,
			    MSENSE_SENSOR_STREAM_END_BYTES);
	sys_put_le16(stream.terminal_status, &slot->data[12]);
	slot->data[14] = stream_post_terminal_state_locked();
	slot->data[15] = 0U;
	sys_put_le32(stream.terminal_history_sent, &slot->data[16]);
	sys_put_le32(stream.terminal_forward_captured, &slot->data[20]);
	sys_put_le32(stream.terminal_bytes_sent, &slot->data[24]);
	sys_put_le32(stream.terminal_data_messages, &slot->data[28]);
	sys_put_le32((uint32_t)stream.terminal_detail, &slot->data[32]);
	slot->session_generation = session_generation;
	slot->history_data = false;
	slot->terminal_end = true;
	/* Mark before submission so an immediate completion cannot be missed. */
	stream.terminal_submitted = true;
	k_spin_unlock(&stream.lock, key);

	ret = stream_submit_slot(conn, slot, message_len);
	bt_conn_unref(conn);
	if (ret != 0) {
		key = k_spin_lock(&stream.lock);
		if (stream.session_active && stream.session_generation == session_generation) {
			stream.terminal_submitted = false;
		}
		k_spin_unlock(&stream.lock, key);
		stream_release_tx_slot(slot);
		if (!stream_schedule_retry(ret)) {
			key = k_spin_lock(&stream.lock);
			if (stream.session_active && stream.session_generation == session_generation) {
				stream_reset_session_locked();
			}
			k_spin_unlock(&stream.lock, key);
			LOG_WRN("END submit failed: %d", ret);
			return 1;
		}
		return 0;
	}

	stream_note_submit_success();
	return 1;
}

static int stream_send_pending_result(void)
{
	struct stream_tx_slot *slot;
	struct stream_pending_result result;
	struct bt_conn *conn;
	uint32_t connection_generation;
	uint16_t message_len = MSENSE_SENSOR_STREAM_HEADER_BYTES +
			       MSENSE_SENSOR_STREAM_RESULT_BYTES;
	int ret;
	k_spinlock_key_t key;

	key = k_spin_lock(&stream.lock);
	if (stream.result_count == 0U) {
		k_spin_unlock(&stream.lock, key);
		return 1;
	}
	result = stream.results[stream.result_head];
	k_spin_unlock(&stream.lock, key);

	conn = stream_connection_ref(true, &connection_generation);
	if (conn == NULL || connection_generation != result.connection_generation) {
		if (conn != NULL) {
			bt_conn_unref(conn);
		}
		key = k_spin_lock(&stream.lock);
		if (stream.result_count != 0U &&
		    stream.results[stream.result_head].connection_generation ==
			    result.connection_generation) {
			stream.result_head = (stream.result_head + 1U) % STREAM_RESULT_QUEUE_DEPTH;
			stream.result_count--;
		}
		k_spin_unlock(&stream.lock, key);
		return 1;
	}

	slot = stream_claim_tx_slot();
	if (slot == NULL) {
		bt_conn_unref(conn);
		return 0;
	}

	stream_write_header(slot->data, MSENSE_SENSOR_STREAM_MESSAGE_RESULT,
			    result.session_id, MSENSE_SENSOR_STREAM_RESULT_BYTES);
	sys_put_le16(result.status, &slot->data[12]);
	slot->data[14] = result.state;
	slot->data[15] = 0U;
	slot->session_generation = stream.session_generation;
	slot->history_data = false;
	slot->terminal_end = false;

	ret = stream_submit_slot(conn, slot, message_len);
	bt_conn_unref(conn);
	if (ret != 0) {
		stream_release_tx_slot(slot);
		if (!stream_schedule_retry(ret)) {
			key = k_spin_lock(&stream.lock);
			if (stream.result_count != 0U &&
			    stream.results[stream.result_head].session_id == result.session_id) {
				stream.result_head =
					(stream.result_head + 1U) % STREAM_RESULT_QUEUE_DEPTH;
				stream.result_count--;
			}
			k_spin_unlock(&stream.lock, key);
		}
		return 1;
	}

	key = k_spin_lock(&stream.lock);
	if (stream.result_count != 0U &&
	    stream.results[stream.result_head].session_id == result.session_id &&
	    stream.results[stream.result_head].connection_generation ==
		    result.connection_generation) {
		stream.result_head = (stream.result_head + 1U) % STREAM_RESULT_QUEUE_DEPTH;
		stream.result_count--;
	}
	k_spin_unlock(&stream.lock, key);
	stream_note_submit_success();
	return 1;
}

static int stream_send_data(void)
{
	struct stream_tx_slot *slot;
	struct bt_conn *conn;
	uint32_t connection_generation;
	uint32_t session_generation;
	uint32_t source_index;
	uint32_t record_index;
	uint32_t remaining;
	uint16_t max_records;
	uint16_t record_count;
	uint16_t payload_length;
	uint16_t message_length;
	uint16_t mtu;
	uint8_t phase;
	bool history_data;
	int ret;
	uint16_t i;
	k_spinlock_key_t key;

	conn = stream_connection_ref(true, &connection_generation);
	if (conn == NULL) {
		return 0;
	}

	mtu = bt_gatt_get_mtu(conn);
	if (mtu < STREAM_REQUIRED_ATT_MTU) {
		bt_conn_unref(conn);
		key = k_spin_lock(&stream.lock);
		if (stream.session_active && !stream.terminal_pending &&
		    !stream.terminal_submitted) {
			stream_begin_terminal_locked(MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR,
						     -EMSGSIZE, true);
		}
		k_spin_unlock(&stream.lock, key);
		return 1;
	}

	max_records = (mtu - STREAM_ATT_NOTIFY_OVERHEAD -
		       MSENSE_SENSOR_STREAM_HEADER_BYTES -
		       MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES) / stream.record_size;
	if (max_records == 0U) {
		bt_conn_unref(conn);
		return 0;
	}

	slot = stream_claim_tx_slot();
	if (slot == NULL) {
		bt_conn_unref(conn);
		return 0;
	}

	key = k_spin_lock(&stream.lock);
	if (!stream.session_active || stream.state != MSENSE_SENSOR_STREAM_STATE_ACTIVE ||
	    stream.start_ack_pending || stream.terminal_pending || stream.terminal_submitted ||
	    connection_generation != stream.connection_generation) {
		k_spin_unlock(&stream.lock, key);
		stream_release_tx_slot(slot);
		bt_conn_unref(conn);
		return 1;
	}

	if (stream.history_tx_index < stream.history_record_count) {
		phase = 0U;
		record_index = stream.history_tx_index;
		remaining = stream.history_record_count - stream.history_tx_index;
		record_count = (uint16_t)MIN((uint32_t)max_records, remaining);
		history_data = true;
	} else if (stream.forward_tx_index < stream.forward_count) {
		phase = 1U;
		record_index = stream.history_record_count + stream.forward_tx_index;
		remaining = stream.forward_count - stream.forward_tx_index;
		record_count = (uint16_t)MIN((uint32_t)max_records, remaining);
		history_data = false;
	} else {
		k_spin_unlock(&stream.lock, key);
		stream_release_tx_slot(slot);
		bt_conn_unref(conn);
		return 0;
	}

	payload_length = MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES +
			 record_count * stream.record_size;
	message_length = MSENSE_SENSOR_STREAM_HEADER_BYTES + payload_length;
	session_generation = stream.session_generation;
	stream_write_header(slot->data, MSENSE_SENSOR_STREAM_MESSAGE_DATA, stream.session_id,
			    payload_length);
	sys_put_le32(stream.data_sequence, &slot->data[12]);
	sys_put_le32(record_index, &slot->data[16]);
	sys_put_le16(record_count, &slot->data[20]);
	slot->data[22] = phase;
	slot->data[23] = 0U;

	for (i = 0U; i < record_count; i++) {
		if (history_data) {
			source_index = stream.frozen_history_start + stream.history_tx_index + i;
			if (source_index >= stream.history_record_count) {
				source_index -= stream.history_record_count;
			}
			memcpy(&slot->data[24U + i * stream.record_size],
			       &stream_history_buffer()[source_index * stream.record_size],
			       stream.record_size);
		} else {
			source_index = stream.forward_tx_index + i;
			memcpy(&slot->data[24U + i * stream.record_size],
			       &stream_forward_buffer()[source_index * stream.record_size],
			       stream.record_size);
		}
	}

	slot->session_generation = session_generation;
	slot->history_data = history_data;
	slot->terminal_end = false;
	if (history_data) {
		stream.history_data_inflight++;
	}
	k_spin_unlock(&stream.lock, key);

	ret = stream_submit_slot(conn, slot, message_length);
	bt_conn_unref(conn);
	if (ret != 0) {
		key = k_spin_lock(&stream.lock);
		if (history_data && stream.session_generation == session_generation &&
		    stream.history_data_inflight != 0U) {
			stream.history_data_inflight--;
		}
		k_spin_unlock(&stream.lock, key);
		stream_release_tx_slot(slot);
		if (!stream_schedule_retry(ret)) {
			key = k_spin_lock(&stream.lock);
			if (stream.session_active && stream.session_generation == session_generation &&
			    !stream.terminal_pending && !stream.terminal_submitted) {
				if (ret == -ENOMEM || ret == -EAGAIN) {
					stream_begin_terminal_locked(
						MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR, ret, true);
				} else {
					stream_reset_session_locked();
				}
			}
			k_spin_unlock(&stream.lock, key);
			return 1;
		}
		return 0;
	}

	key = k_spin_lock(&stream.lock);
	if (stream.session_active && stream.session_generation == session_generation) {
		if (history_data) {
			stream.history_tx_index += record_count;
			if (stream.history_tx_index == stream.history_record_count) {
				stream.history_tx_finished = true;
			}
		} else {
			stream.forward_tx_index += record_count;
		}
		stream.data_sequence++;
		stream.data_message_count++;
		if (stream.terminal_pending || stream.terminal_submitted) {
			stream.terminal_history_sent = stream.history_tx_index;
			stream.terminal_bytes_sent =
				(stream.history_tx_index + stream.forward_tx_index) * stream.record_size;
			stream.terminal_data_messages = stream.data_message_count;
		}
		stream_maybe_start_fresh_history_locked();
	}
	k_spin_unlock(&stream.lock, key);
	stream_note_submit_success();
	return 1;
}

static void stream_process_start(const struct stream_command *command)
{
	struct bt_conn *conn;
	uint32_t connection_generation;
	uint16_t status = MSENSE_SENSOR_STREAM_STATUS_SUCCESS;
	uint16_t mtu;
	uint8_t state;
	bool notifications_enabled;
	bool recording;
	bool subscribed;
	k_spinlock_key_t key;

	conn = stream_connection_ref(false, &connection_generation);
	if (conn == NULL || connection_generation != command->connection_generation) {
		if (conn != NULL) {
			bt_conn_unref(conn);
		}
		return;
	}

	mtu = bt_gatt_get_mtu(conn);
	subscribed = bt_gatt_is_subscribed(conn, STREAM_TX_ATTR, BT_GATT_CCC_NOTIFY);
	bt_conn_unref(conn);

	key = k_spin_lock(&stream.lock);
	if (!stream.initialized) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_INITIALIZED;
	} else if (!stream.recording || stream.state == MSENSE_SENSOR_STREAM_STATE_NOT_RECORDING) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING;
	} else if (stream.state == MSENSE_SENSOR_STREAM_STATE_HISTORY_FILLING) {
		status = MSENSE_SENSOR_STREAM_STATUS_HISTORY_NOT_READY;
	} else if (stream.session_active || stream.state == MSENSE_SENSOR_STREAM_STATE_ACTIVE ||
		   stream.state == MSENSE_SENSOR_STREAM_STATE_ABORTING) {
		status = MSENSE_SENSOR_STREAM_STATUS_BUSY;
	} else if (!stream.notifications_enabled || !subscribed) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_SUBSCRIBED;
	} else if (mtu < STREAM_REQUIRED_ATT_MTU) {
		status = MSENSE_SENSOR_STREAM_STATUS_MTU_TOO_SMALL;
	} else if (stream.state != MSENSE_SENSOR_STREAM_STATE_READY) {
		status = MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR;
	} else {
		stream.session_generation++;
		stream.session_active = true;
		stream.session_id = command->session_id;
		stream.frozen_history_start = stream.history_write_index;
		stream.history_tx_index = 0U;
		stream.history_data_inflight = 0U;
		stream.forward_count = 0U;
		stream.forward_tx_index = 0U;
		stream.data_sequence = 0U;
		stream.data_message_count = 0U;
		stream.retry_count = 0U;
		stream.history_collecting = false;
		stream.history_tx_finished = false;
		stream.start_ack_pending = true;
		stream.terminal_pending = false;
		stream.terminal_submitted = false;
		stream.terminal_complete = false;
		stream.state = MSENSE_SENSOR_STREAM_STATE_ACTIVE;
	}
	if (status != MSENSE_SENSOR_STREAM_STATUS_SUCCESS) {
		stream_queue_result_locked(command->session_id, status,
					   command->connection_generation);
	}
	state = stream.state;
	recording = stream.recording;
	notifications_enabled = stream.notifications_enabled;
	k_spin_unlock(&stream.lock, key);

	if (status == MSENSE_SENSOR_STREAM_STATUS_SUCCESS) {
		LOG_INF("NUS START accepted: session 0x%08x, MTU %u", command->session_id,
			mtu);
	} else {
		LOG_WRN("NUS START rejected: session 0x%08x, status 0x%04x, state 0x%02x, "
			"recording %u, CCC %u, subscribed %u, MTU %u",
			command->session_id, status, state, recording, notifications_enabled,
			subscribed, mtu);
	}
}

static void stream_process_cancel(const struct stream_command *command)
{
	uint16_t status = MSENSE_SENSOR_STREAM_STATUS_SUCCESS;
	k_spinlock_key_t key;

	key = k_spin_lock(&stream.lock);
	if (!stream.initialized) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_INITIALIZED;
	} else if (!stream.session_active || command->session_id != stream.session_id) {
		status = MSENSE_SENSOR_STREAM_STATUS_WRONG_SESSION;
	} else if (!stream.terminal_pending && !stream.terminal_submitted) {
		stream_begin_terminal_locked(MSENSE_SENSOR_STREAM_STATUS_CANCELLED, 0, true);
	}
	if (status != MSENSE_SENSOR_STREAM_STATUS_SUCCESS) {
		stream_queue_result_locked(command->session_id, status,
					   command->connection_generation);
	}
	k_spin_unlock(&stream.lock, key);
}

static void stream_process_command(const struct stream_command *command)
{
	k_spinlock_key_t key;
	bool current_connection;

	key = k_spin_lock(&stream.lock);
	current_connection = stream.conn != NULL &&
			     stream.connection_generation == command->connection_generation;
	k_spin_unlock(&stream.lock, key);
	if (!current_connection) {
		return;
	}

	switch (command->kind) {
	case STREAM_COMMAND_START:
		stream_process_start(command);
		break;
	case STREAM_COMMAND_CANCEL:
		stream_process_cancel(command);
		break;
	case STREAM_COMMAND_RESULT:
		key = k_spin_lock(&stream.lock);
		stream_queue_result_locked(command->session_id, command->result_status,
					   command->connection_generation);
		k_spin_unlock(&stream.lock, key);
		break;
	default:
		break;
	}
}

static bool stream_has_start_ack_pending(void)
{
	bool pending;
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	pending = stream.session_active && stream.start_ack_pending;
	k_spin_unlock(&stream.lock, key);
	return pending;
}

static bool stream_has_terminal_pending(void)
{
	bool pending;
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	pending = stream.session_active && stream.terminal_pending && !stream.terminal_submitted;
	k_spin_unlock(&stream.lock, key);
	return pending;
}

static bool stream_has_result_pending(void)
{
	bool pending;
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	pending = stream.result_count != 0U;
	k_spin_unlock(&stream.lock, key);
	return pending;
}

static bool stream_maybe_begin_success_terminal(void)
{
	bool started = false;
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	if (stream.session_active && stream.state == MSENSE_SENSOR_STREAM_STATE_ACTIVE &&
	    !stream.start_ack_pending && !stream.terminal_pending &&
	    !stream.terminal_submitted && stream.history_tx_index == stream.history_record_count &&
	    stream.history_data_inflight == 0U && stream.history_collecting &&
	    stream.forward_count == stream.forward_record_count &&
	    stream.forward_tx_index == stream.forward_record_count) {
		stream_begin_terminal_locked(MSENSE_SENSOR_STREAM_STATUS_SUCCESS, 0, false);
		started = true;
	}
	k_spin_unlock(&stream.lock, key);
	return started;
}

static bool stream_can_send_data(void)
{
	bool active;
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	active = stream.session_active && stream.state == MSENSE_SENSOR_STREAM_STATE_ACTIVE &&
		 !stream.start_ack_pending && !stream.terminal_pending && !stream.terminal_submitted &&
		 (stream.history_tx_index < stream.history_record_count ||
		  stream.forward_tx_index < stream.forward_count);
	k_spin_unlock(&stream.lock, key);
	return active;
}

static void stream_thread(void *arg1, void *arg2, void *arg3)
{
	struct stream_command command;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&stream_wake, K_FOREVER);

		while (k_msgq_get(&stream_command_queue, &command, K_NO_WAIT) == 0) {
			stream_process_command(&command);
		}

		for (;;) {
			if (stream_finalize_terminal()) {
				continue;
			}
			if (stream_has_start_ack_pending()) {
				if (stream_send_start_ack() == 0) {
					break;
				}
				continue;
			}
			if (stream_has_terminal_pending()) {
				if (stream_send_terminal() == 0) {
					break;
				}
				continue;
			}
			if (stream_has_result_pending()) {
				if (stream_send_pending_result() == 0) {
					break;
				}
				continue;
			}
			if (stream_maybe_begin_success_terminal()) {
				continue;
			}
			if (stream_can_send_data()) {
				if (stream_send_data() == 0) {
					break;
				}
				continue;
			}
			break;
		}
	}
}

static ssize_t stream_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset,
			       uint8_t flags)
{
	const uint8_t *command_bytes = buf;
	struct stream_command command = { 0 };
	k_spinlock_key_t key;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		LOG_WRN("NUS command rejected: nonzero offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != MSENSE_SENSOR_STREAM_COMMAND_BYTES) {
		LOG_WRN("NUS command rejected: length %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	key = k_spin_lock(&stream.lock);
	if (stream.conn != conn) {
		k_spin_unlock(&stream.lock, key);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	command.connection_generation = stream.connection_generation;
	k_spin_unlock(&stream.lock, key);
	command.session_id = sys_get_le32(&command_bytes[4]);

	if (command_bytes[0] != MSENSE_SENSOR_STREAM_MAGIC0 ||
	    command_bytes[1] != MSENSE_SENSOR_STREAM_MAGIC1 || command.session_id == 0U) {
		command.kind = STREAM_COMMAND_RESULT;
		command.result_status = MSENSE_SENSOR_STREAM_STATUS_INVALID_COMMAND;
	} else if (command_bytes[2] != MSENSE_SENSOR_STREAM_PROTOCOL_VERSION) {
		command.kind = STREAM_COMMAND_RESULT;
		command.result_status = MSENSE_SENSOR_STREAM_STATUS_UNSUPPORTED_VERSION;
	} else if (command_bytes[3] == MSENSE_SENSOR_STREAM_OPCODE_START) {
		command.kind = STREAM_COMMAND_START;
	} else if (command_bytes[3] == MSENSE_SENSOR_STREAM_OPCODE_CANCEL) {
		command.kind = STREAM_COMMAND_CANCEL;
	} else {
		command.kind = STREAM_COMMAND_RESULT;
		command.result_status = MSENSE_SENSOR_STREAM_STATUS_INVALID_COMMAND;
	}

	if (k_msgq_put(&stream_command_queue, &command, K_NO_WAIT) != 0) {
		LOG_WRN("NUS command queue full: opcode 0x%02x, session 0x%08x",
			command_bytes[3], command.session_id);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}

	LOG_INF("NUS command queued: opcode 0x%02x, session 0x%08x, generation %u",
		command_bytes[3], command.session_id, command.connection_generation);
	k_sem_give(&stream_wake);
	return len;
}

static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool notifications_enabled;
	k_spinlock_key_t key;

	ARG_UNUSED(attr);

	key = k_spin_lock(&stream.lock);
	notifications_enabled = (value & BT_GATT_CCC_NOTIFY) != 0U;
	stream.notifications_enabled = notifications_enabled;
	if (!notifications_enabled && stream.session_active) {
		stream_reset_session_locked();
	}
	k_spin_unlock(&stream.lock, key);
	LOG_INF("NUS TX notifications %s (CCCD 0x%04x)",
		notifications_enabled ? "enabled" : "disabled", value);
	k_sem_give(&stream_wake);
}

int msense_sensor_stream_init(const struct msense_sensor_stream_config *config)
{
	uint8_t tree_state;
	int ret;
	k_spinlock_key_t key;

	ret = bt_conn_cb_register(&stream_conn_callbacks);
	if (ret != 0 && ret != -EEXIST) {
		return ret;
	}

	if (!stream_config_is_valid(config, &tree_state)) {
		return -EINVAL;
	}

	key = k_spin_lock(&stream.lock);
	if (stream.initialized) {
		k_spin_unlock(&stream.lock, key);
		return 0;
	}

	stream.device_type = config->device_type;
	stream.record_format_version = config->record_format_version;
	stream.device_name_len = (uint8_t)config->device_name_len;
	stream.record_size = config->record_size;
	stream.record_rate_numerator = config->record_rate_numerator;
	stream.record_rate_denominator = config->record_rate_denominator;
	stream.history_record_count = config->history_record_count;
	stream.forward_record_count = config->forward_record_count;
	memcpy(stream.device_id, config->device_id, sizeof(stream.device_id));
	memset(stream.device_name, 0, sizeof(stream.device_name));
	memcpy(stream.device_name, config->device_name, config->device_name_len);
	memcpy(stream.git_commit, config->git_commit, sizeof(stream.git_commit) - 1U);
	stream.git_commit[sizeof(stream.git_commit) - 1U] = '\0';
	stream.git_tree_state = tree_state;
	stream.initialized = true;
	stream.recording = false;
	stream.history_collecting = false;
	stream.state = MSENSE_SENSOR_STREAM_STATE_NOT_RECORDING;
	k_spin_unlock(&stream.lock, key);

	LOG_INF("NUS stream initialized: device 0x%02x, record size %u, rate %u/%u Hz, "
		"history %u, forward %u",
		config->device_type, config->record_size, config->record_rate_numerator,
		config->record_rate_denominator, config->history_record_count,
		config->forward_record_count);
	k_sem_give(&stream_wake);
	return 0;
}

void msense_sensor_stream_recording_started(void)
{
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	if (stream.initialized) {
		stream.recording = true;
		stream_reset_session_locked();
	}
	k_spin_unlock(&stream.lock, key);
	k_sem_give(&stream_wake);
}

void msense_sensor_stream_recording_stopped(void)
{
	k_spinlock_key_t key = k_spin_lock(&stream.lock);

	if (stream.initialized) {
		stream.recording = false;
		if (!stream.session_active) {
			stream_reset_session_locked();
		} else if (stream.conn == NULL || !stream.notifications_enabled) {
			stream_reset_session_locked();
		} else if (!stream.terminal_pending && !stream.terminal_submitted) {
			stream_begin_terminal_locked(MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR,
						     -ECANCELED, true);
		} else {
			stream.history_collecting = false;
			stream.history_count = 0U;
			stream.history_write_index = 0U;
			stream.state = MSENSE_SENSOR_STREAM_STATE_ABORTING;
		}
	}
	k_spin_unlock(&stream.lock, key);
	k_sem_give(&stream_wake);
}

int msense_sensor_stream_accept_record(const void *record, size_t record_size)
{
	const uint8_t *record_bytes = record;
	bool wake = false;
	k_spinlock_key_t key;

	if (record == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&stream.lock);
	if (!stream.initialized || !stream.recording) {
		k_spin_unlock(&stream.lock, key);
		return 0;
	}
	if (record_size != stream.record_size) {
		k_spin_unlock(&stream.lock, key);
		return -EMSGSIZE;
	}

	if (stream.state == MSENSE_SENSOR_STREAM_STATE_ACTIVE && stream.session_active &&
	    !stream.terminal_pending && !stream.terminal_submitted) {
		if (!stream.history_collecting && stream.forward_count < stream.forward_record_count) {
			memcpy(&stream_forward_buffer()[stream.forward_count * stream.record_size],
			       record_bytes, stream.record_size);
			stream.forward_count++;
			stream_maybe_start_fresh_history_locked();
			wake = true;
		} else if (stream.history_collecting) {
			stream_append_history_locked(record_bytes);
		}
	} else if (stream.state == MSENSE_SENSOR_STREAM_STATE_ABORTING &&
		   stream.session_active && stream.history_collecting) {
		stream_append_history_locked(record_bytes);
	} else if (stream.state == MSENSE_SENSOR_STREAM_STATE_HISTORY_FILLING ||
		   stream.state == MSENSE_SENSOR_STREAM_STATE_READY) {
		stream_append_history_locked(record_bytes);
	}
	k_spin_unlock(&stream.lock, key);

	if (wake) {
		k_sem_give(&stream_wake);
	}
	return 0;
}

void msense_sensor_stream_storage_failed(int error)
{
	k_spinlock_key_t key = k_spin_lock(&stream.lock);
	int32_t detail = error < 0 ? error : -EIO;

	if (!stream.initialized || !stream.session_active || stream.terminal_submitted) {
		k_spin_unlock(&stream.lock, key);
		k_sem_give(&stream_wake);
		return;
	}

	if (stream.conn == NULL || !stream.notifications_enabled) {
		stream_reset_session_locked();
	} else if (stream.terminal_pending) {
		stream.terminal_status = MSENSE_SENSOR_STREAM_STATUS_STORAGE_ERROR;
		stream.terminal_detail = detail;
		stream_reset_history_locked();
	} else {
		stream_begin_terminal_locked(MSENSE_SENSOR_STREAM_STATUS_STORAGE_ERROR,
					     detail, true);
	}
	k_spin_unlock(&stream.lock, key);
	k_sem_give(&stream_wake);
}
