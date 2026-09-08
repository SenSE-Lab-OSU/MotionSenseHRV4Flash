/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
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
#define STREAM_TX_BYTES (CONFIG_BT_L2CAP_TX_MTU - STREAM_ATT_NOTIFY_OVERHEAD)
#define STREAM_RETRY_DELAY_MS 20U
#define STREAM_OWNER_BUDGET 8U
#define STREAM_DISCONNECT_STACK_SIZE 1024U
#define STREAM_DISCONNECT_PRIORITY 8

BUILD_ASSERT(MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE *
		     MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS ==
		     MSENSE_SENSOR_STREAM_HISTORY_BYTES,
	     "PPG history geometry must be 32 KiB");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE *
		     MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS ==
		     MSENSE_SENSOR_STREAM_HISTORY_BYTES,
	     "ECG history geometry must be 32 KiB");
BUILD_ASSERT(MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES ==
		     MSENSE_SENSOR_STREAM_HISTORY_BYTES * 4U,
	     "Capture buffer must be the fixed 128 KiB partition");
BUILD_ASSERT(CONFIG_BT_L2CAP_TX_MTU >= STREAM_REQUIRED_ATT_MTU,
	     "The local NUS stream MTU must support the protocol minimum");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_CONN_TX_NOTIFY_WQ),
	     "The stream owner requires GATT completion on the system workqueue");

enum stream_command_kind {
	STREAM_COMMAND_START,
	STREAM_COMMAND_START_INFINITY,
	STREAM_COMMAND_STOP,
	STREAM_COMMAND_RESULT,
};

enum stream_tx_kind {
	STREAM_TX_ACK,
	STREAM_TX_DATA,
	STREAM_TX_END,
	STREAM_TX_RESULT,
};

enum stream_response_kind {
	STREAM_RESPONSE_NONE,
	STREAM_RESPONSE_ACK,
	STREAM_RESPONSE_END,
	STREAM_RESPONSE_RESULT,
};

enum stream_ingress_state {
	STREAM_INGRESS_FREE,
	STREAM_INGRESS_FILLING,
	STREAM_INGRESS_READY,
};

struct stream_command {
	uint32_t session_id;
	uint32_t connection_generation;
	uint16_t status;
	uint8_t kind;
};

struct stream_ingress_slot {
	uint8_t data[CONFIG_MSENSE_SENSOR_STREAM_INGRESS_RECORD_BYTES];
	uint32_t generation;
	uint8_t state;
};

/* Cross-context publication only; the owner never holds this lock around copies. */
struct stream_publication {
	struct k_spinlock lock;
	struct bt_conn *conn;
	struct msense_sensor_stream_config config;
	struct stream_command command;
	uint32_t connection_generation;
	uint32_t acquisition_generation;
	uint32_t ingress_head;
	uint32_t ingress_tail;
	uint16_t acquisition_status;
	bool configured;
	bool recording;
	bool notifications_enabled;
	bool command_pending;
	bool command_ready;
	bool ingress_loss_pending;
	struct stream_ingress_slot ingress[CONFIG_MSENSE_SENSOR_STREAM_INGRESS_SLOTS];
};

struct stream_tx {
	struct bt_gatt_notify_params params;
	uint8_t data[STREAM_TX_BYTES];
	uintptr_t token;
	uint32_t session_generation;
	uint32_t connection_generation;
	uint8_t kind;
	bool busy;
};

/* This state is serialized by the system workqueue owner and its TX callback. */
struct stream_runtime {
	struct bt_conn *conn;
	uint64_t future_quota_records;
	uint64_t future_enqueued;
	uint64_t data_byte_offset;
	int64_t no_progress_since_ms;
	int64_t retry_after_ms;
	uint32_t connection_generation;
	uint32_t acquisition_generation;
	uint32_t session_generation;
	uint32_t session_id;
	uint32_t history_record_count;
	uint32_t history_write_index;
	uint32_t live_capacity;
	uint32_t live_head;
	uint32_t live_tail;
	uint32_t live_count;
	uint16_t record_size;
	uint16_t live_head_offset;
	uint16_t max_sensor_bytes;
	uint16_t terminal_status;
	uint32_t response_connection_generation;
	uint8_t response_kind;
	bool initialized;
	bool recording;
	bool notifications_enabled;
	bool session_active;
	bool infinity;
	bool ack_pending;
	bool ack_complete;
	bool terminal_pending;
	bool terminal_submitted;
	bool terminal_complete;
	bool result_pending;
	uint32_t result_session_id;
	uint32_t result_connection_generation;
	uint16_t result_status;
};

static struct stream_publication publication;
static struct stream_runtime stream;
static struct stream_tx tx;
static uintptr_t next_tx_token;
static uint8_t stream_record_buffer[MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES];

static struct k_spinlock disconnect_lock;
static struct bt_conn *disconnect_conn;
K_SEM_DEFINE(disconnect_wake, 0, 1);

static ssize_t stream_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset,
			       uint8_t flags);
static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void stream_notify_complete(struct bt_conn *conn, void *user_data);
static void stream_owner_work_handler(struct k_work *work);
static void stream_disconnect_thread(void *arg1, void *arg2, void *arg3);

BT_GATT_SERVICE_DEFINE(msense_sensor_stream_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX_CHAR, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX_CHAR, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, stream_rx_write, NULL),
);

#define STREAM_TX_ATTR (&msense_sensor_stream_svc.attrs[2])

K_WORK_DELAYABLE_DEFINE(stream_owner_work, stream_owner_work_handler);
K_THREAD_DEFINE(msense_stream_disconnect_id, STREAM_DISCONNECT_STACK_SIZE,
		stream_disconnect_thread, NULL, NULL, NULL,
		STREAM_DISCONNECT_PRIORITY, 0, 0);

static void stream_wake(void)
{
	(void)k_work_reschedule(&stream_owner_work, K_NO_WAIT);
}

static bool stream_config_is_valid(const struct msense_sensor_stream_config *config)
{
	uint64_t total_bytes;

	if (config == NULL || config->record_size == 0U ||
	    config->record_size > CONFIG_MSENSE_SENSOR_STREAM_INGRESS_RECORD_BYTES) {
		return false;
	}
	total_bytes = (uint64_t)config->record_size *
		      ((uint64_t)config->history_record_count + config->forward_record_count);
	if (config->device_type == MSENSE_SENSOR_STREAM_DEVICE_PPG) {
		return config->record_size == MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE &&
		       config->history_record_count == MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS &&
		       config->forward_record_count == MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS &&
		       total_bytes == MSENSE_SENSOR_STREAM_FINITE_BYTES;
	}
	if (config->device_type == MSENSE_SENSOR_STREAM_DEVICE_ECG) {
		return config->record_size == MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE &&
		       config->history_record_count == MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS &&
		       config->forward_record_count == MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS &&
		       total_bytes == MSENSE_SENSOR_STREAM_FINITE_BYTES;
	}
	return false;
}

static uint8_t *stream_history_buffer(void)
{
	return stream_record_buffer;
}

static uint8_t *stream_snapshot_buffer(void)
{
	return &stream_record_buffer[MSENSE_SENSOR_STREAM_HISTORY_BYTES];
}

static uint8_t *stream_live_buffer(void)
{
	return &stream_record_buffer[MSENSE_SENSOR_STREAM_HISTORY_BYTES * 2U];
}

static void stream_reset_history(void)
{
	memset(stream_history_buffer(), 0, MSENSE_SENSOR_STREAM_HISTORY_BYTES);
	stream.history_write_index = 0U;
}

static void stream_clear_live(void)
{
	stream.live_head = 0U;
	stream.live_tail = 0U;
	stream.live_count = 0U;
	stream.live_head_offset = 0U;
	stream.future_enqueued = 0U;
}

static void stream_append_history(const uint8_t *record)
{
	memcpy(&stream_history_buffer()[stream.history_write_index * stream.record_size],
	       record, stream.record_size);
	stream.history_write_index = (stream.history_write_index + 1U) %
				     stream.history_record_count;
}

static void stream_snapshot_history(void)
{
	uint32_t tail_records = stream.history_record_count - stream.history_write_index;
	uint32_t tail_bytes = tail_records * stream.record_size;

	memcpy(stream_snapshot_buffer(),
	       &stream_history_buffer()[stream.history_write_index * stream.record_size],
	       tail_bytes);
	if (stream.history_write_index != 0U) {
		memcpy(&stream_snapshot_buffer()[tail_bytes], stream_history_buffer(),
		       stream.history_write_index * stream.record_size);
	}
}

static uint64_t stream_live_pending_bytes(void)
{
	return (uint64_t)stream.live_count * stream.record_size - stream.live_head_offset;
}

static void stream_write_header(uint8_t *message, uint8_t type, uint32_t session_id,
				uint16_t payload_len)
{
	message[0] = MSENSE_SENSOR_STREAM_MAGIC0;
	message[1] = MSENSE_SENSOR_STREAM_MAGIC1;
	message[2] = MSENSE_SENSOR_STREAM_PROTOCOL_VERSION;
	message[3] = type;
	sys_put_le32(session_id, &message[4]);
	sys_put_le16(payload_len, &message[8]);
	sys_put_le16(0U, &message[10]);
}

static void stream_command_answered(uint8_t kind)
{
	k_spinlock_key_t key;

	if (kind == STREAM_RESPONSE_NONE || stream.response_kind != kind) {
		return;
	}
	key = k_spin_lock(&publication.lock);
	if (publication.command_pending &&
	    stream.response_connection_generation == publication.connection_generation) {
		publication.command_pending = false;
	}
	k_spin_unlock(&publication.lock, key);
	stream.response_kind = STREAM_RESPONSE_NONE;
}

static void stream_forget_command(uint32_t connection_generation)
{
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	if (publication.command_pending &&
	    publication.command.connection_generation == connection_generation) {
		publication.command_pending = false;
	}
	k_spin_unlock(&publication.lock, key);
}

static void stream_invalidate_tx(void)
{
	tx.busy = false;
	if (++next_tx_token == 0U) {
		++next_tx_token;
	}
	tx.token = next_tx_token;
}

static void stream_session_retire(bool invalidate_tx)
{
	stream.session_generation++;
	if (stream.session_generation == 0U) {
		stream.session_generation++;
	}
	stream.session_active = false;
	stream.infinity = false;
	stream.ack_pending = false;
	stream.ack_complete = false;
	stream.terminal_pending = false;
	stream.terminal_submitted = false;
	stream.terminal_complete = false;
	stream.data_byte_offset = 0U;
	stream.no_progress_since_ms = 0;
	stream.retry_after_ms = 0;
	stream_clear_live();
	if (invalidate_tx) {
		stream_invalidate_tx();
	}
}

static void stream_begin_terminal(uint16_t status)
{
	if (stream.session_active && !stream.terminal_pending &&
	    !stream.terminal_submitted) {
		stream.terminal_status = status;
		stream.terminal_pending = true;
	}
}

static void stream_request_disconnect(void)
{
	struct bt_conn *held = NULL;
	k_spinlock_key_t key;

	if (stream.conn != NULL) {
		held = bt_conn_ref(stream.conn);
	}
	if (held == NULL) {
		return;
	}
	key = k_spin_lock(&disconnect_lock);
	if (disconnect_conn == NULL) {
		disconnect_conn = held;
		held = NULL;
	}
	k_spin_unlock(&disconnect_lock, key);
	if (held != NULL) {
		bt_conn_unref(held);
	} else {
		k_sem_give(&disconnect_wake);
	}
}

static void stream_disconnect_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		struct bt_conn *conn;
		k_spinlock_key_t key;

		(void)k_sem_take(&disconnect_wake, K_FOREVER);
		key = k_spin_lock(&disconnect_lock);
		conn = disconnect_conn;
		disconnect_conn = NULL;
		k_spin_unlock(&disconnect_lock, key);
		if (conn != NULL) {
			(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			bt_conn_unref(conn);
		}
	}
}

static bool stream_copy_next_data(uint8_t *destination, uint16_t length)
{
	uint64_t offset = stream.data_byte_offset;
	uint32_t live_bytes = stream.live_capacity * stream.record_size;
	uint32_t live_cursor = stream.live_head * stream.record_size + stream.live_head_offset;
	uint16_t copied = 0U;
	uint64_t available = (offset < MSENSE_SENSOR_STREAM_HISTORY_BYTES ?
		MSENSE_SENSOR_STREAM_HISTORY_BYTES - offset : 0U) + stream_live_pending_bytes();

	if (length == 0U || length > available) {
		return false;
	}
	while (copied != length) {
		uint16_t chunk;

		if (offset < MSENSE_SENSOR_STREAM_HISTORY_BYTES) {
			chunk = (uint16_t)MIN((uint64_t)length - copied,
					      MSENSE_SENSOR_STREAM_HISTORY_BYTES - offset);
			memcpy(&destination[copied], &stream_snapshot_buffer()[offset], chunk);
		} else {
			uint32_t contiguous = live_bytes - live_cursor;

			chunk = (uint16_t)MIN((uint32_t)(length - copied), contiguous);
			memcpy(&destination[copied], &stream_live_buffer()[live_cursor], chunk);
			live_cursor = (live_cursor + chunk) % live_bytes;
		}
		copied += chunk;
		offset += chunk;
	}
	return true;
}

static void stream_consume_data(uint16_t length)
{
	uint16_t remaining = length;

	if (stream.data_byte_offset < MSENSE_SENSOR_STREAM_HISTORY_BYTES) {
		uint16_t snapshot = (uint16_t)MIN((uint64_t)remaining,
			MSENSE_SENSOR_STREAM_HISTORY_BYTES - stream.data_byte_offset);

		stream.data_byte_offset += snapshot;
		remaining -= snapshot;
	}
	while (remaining != 0U) {
		uint16_t chunk = MIN(remaining,
			(uint16_t)(stream.record_size - stream.live_head_offset));

		stream.data_byte_offset += chunk;
		stream.live_head_offset += chunk;
		remaining -= chunk;
		if (stream.live_head_offset == stream.record_size) {
			stream.live_head_offset = 0U;
			stream.live_head = (stream.live_head + 1U) % stream.live_capacity;
			stream.live_count--;
		}
	}
}

static bool stream_append_live(const uint8_t *record)
{
	if (stream.future_enqueued == UINT64_MAX) {
		stream_begin_terminal(MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR);
		return false;
	}
	if (stream.live_count == stream.live_capacity) {
		stream_begin_terminal(MSENSE_SENSOR_STREAM_STATUS_BUFFER_OVERFLOW);
		return false;
	}
	memcpy(&stream_live_buffer()[stream.live_tail * stream.record_size], record,
	       stream.record_size);
	stream.live_tail = (stream.live_tail + 1U) % stream.live_capacity;
	stream.live_count++;
	stream.future_enqueued++;
	return true;
}

static struct stream_ingress_slot *stream_take_ingress(void)
{
	struct stream_ingress_slot *slot;
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	slot = &publication.ingress[publication.ingress_head];
	if (slot->state != STREAM_INGRESS_READY) {
		slot = NULL;
	} else {
		publication.ingress_head = (publication.ingress_head + 1U) %
			ARRAY_SIZE(publication.ingress);
	}
	k_spin_unlock(&publication.lock, key);
	return slot;
}

static void stream_release_ingress(struct stream_ingress_slot *slot)
{
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	slot->state = STREAM_INGRESS_FREE;
	k_spin_unlock(&publication.lock, key);
}

static bool stream_process_ingress(void)
{
	struct stream_ingress_slot *slot = stream_take_ingress();

	if (slot == NULL) {
		return false;
	}
	if (slot->generation == stream.acquisition_generation) {
		stream_append_history(slot->data);
		if (stream.session_active && !stream.terminal_pending &&
		    !stream.terminal_submitted &&
		    (stream.infinity || stream.future_enqueued < stream.future_quota_records)) {
			(void)stream_append_live(slot->data);
		}
	}
	stream_release_ingress(slot);
	return true;
}

static bool stream_connection_current(uint32_t generation)
{
	bool current;
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	current = publication.conn != NULL &&
		  publication.connection_generation == generation;
	k_spin_unlock(&publication.lock, key);
	return current;
}

static bool stream_sync_publication(void)
{
	struct bt_conn *conn = NULL;
	struct bt_conn *old;
	uint32_t connection_generation;
	uint32_t acquisition_generation;
	uint16_t acquisition_status;
	bool notifications_enabled;
	bool recording;
	bool changed = false;
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	connection_generation = publication.connection_generation;
	acquisition_generation = publication.acquisition_generation;
	notifications_enabled = publication.notifications_enabled;
	recording = publication.recording;
	acquisition_status = publication.acquisition_status;
	if (publication.conn != NULL) {
		conn = bt_conn_ref(publication.conn);
	}
	publication.ingress_loss_pending = false;
	k_spin_unlock(&publication.lock, key);

	if (connection_generation != stream.connection_generation) {
		old = stream.conn;
		stream.conn = conn;
		stream.connection_generation = connection_generation;
		stream.notifications_enabled = notifications_enabled;
		stream_session_retire(true);
		stream.result_pending = false;
		stream.response_kind = STREAM_RESPONSE_NONE;
		if (old != NULL) {
			bt_conn_unref(old);
		}
		conn = NULL;
		changed = true;
	} else {
		if (conn != NULL) {
			bt_conn_unref(conn);
		}
		if (notifications_enabled != stream.notifications_enabled) {
			stream.notifications_enabled = notifications_enabled;
			if (!notifications_enabled) {
				stream_session_retire(true);
				stream.result_pending = false;
				stream.response_kind = STREAM_RESPONSE_NONE;
			}
			changed = true;
		}
	}

	if (acquisition_generation != stream.acquisition_generation) {
		stream.acquisition_generation = acquisition_generation;
		stream_reset_history();
		stream_begin_terminal(acquisition_status);
		stream.recording = recording;
		changed = true;
	}
	return changed;
}

static bool stream_take_command(struct stream_command *command)
{
	bool ready;
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	ready = publication.command_ready;
	if (ready) {
		*command = publication.command;
		publication.command_ready = false;
	}
	k_spin_unlock(&publication.lock, key);
	return ready;
}

static void stream_queue_result(const struct stream_command *command, uint16_t status)
{
	stream.result_session_id = command->session_id;
	stream.result_connection_generation = command->connection_generation;
	stream.result_status = status;
	stream.result_pending = true;
	stream.response_kind = STREAM_RESPONSE_RESULT;
	stream.response_connection_generation = command->connection_generation;
}

static void stream_process_start(const struct stream_command *command, bool infinity)
{
	struct bt_conn *conn;
	uint16_t mtu = 0U;
	uint16_t status = MSENSE_SENSOR_STREAM_STATUS_SUCCESS;
	bool subscribed = false;

	if (!stream.initialized) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_INITIALIZED;
	} else if (!stream.recording) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING;
	} else if (stream.session_active || tx.busy) {
		status = MSENSE_SENSOR_STREAM_STATUS_BUSY;
	} else if (!stream.notifications_enabled || stream.conn == NULL ||
		   !stream_connection_current(command->connection_generation)) {
		status = MSENSE_SENSOR_STREAM_STATUS_NOT_SUBSCRIBED;
	} else {
		conn = bt_conn_ref(stream.conn);
		if (conn != NULL) {
			mtu = bt_gatt_get_mtu(conn);
			subscribed = bt_gatt_is_subscribed(conn, STREAM_TX_ATTR,
						  BT_GATT_CCC_NOTIFY);
			bt_conn_unref(conn);
		}
		if (!stream_connection_current(command->connection_generation) || !subscribed) {
			status = MSENSE_SENSOR_STREAM_STATUS_NOT_SUBSCRIBED;
		} else if (mtu < STREAM_REQUIRED_ATT_MTU) {
			status = MSENSE_SENSOR_STREAM_STATUS_MTU_TOO_SMALL;
		}
	}
	if (status != MSENSE_SENSOR_STREAM_STATUS_SUCCESS) {
		stream_queue_result(command, status);
		return;
	}

	stream.session_generation++;
	if (stream.session_generation == 0U) {
		stream.session_generation++;
	}
	stream.session_id = command->session_id;
	stream.max_sensor_bytes = MIN((uint16_t)(mtu - STREAM_ATT_NOTIFY_OVERHEAD -
		MSENSE_SENSOR_STREAM_HEADER_BYTES - MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES),
		(uint16_t)(STREAM_TX_BYTES - MSENSE_SENSOR_STREAM_HEADER_BYTES -
		MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES));
	stream.infinity = infinity;
	stream.session_active = true;
	stream.data_byte_offset = 0U;
	stream_clear_live();
	stream.ack_pending = true;
	stream.ack_complete = false;
	stream.terminal_pending = false;
	stream.terminal_submitted = false;
	stream.terminal_complete = false;
	stream.no_progress_since_ms = 0;
	stream.response_kind = STREAM_RESPONSE_ACK;
	stream.response_connection_generation = command->connection_generation;
	stream_snapshot_history();
	LOG_INF("NUS START accepted: session 0x%08x %s", command->session_id,
		infinity ? "infinity" : "finite");
}

static void stream_process_command(const struct stream_command *command)
{
	if (command->connection_generation != stream.connection_generation) {
		stream_forget_command(command->connection_generation);
		return;
	}

	switch (command->kind) {
	case STREAM_COMMAND_START:
		stream_process_start(command, false);
		break;
	case STREAM_COMMAND_START_INFINITY:
		stream_process_start(command, true);
		break;
	case STREAM_COMMAND_STOP:
		if (!stream.initialized) {
			stream_queue_result(command, MSENSE_SENSOR_STREAM_STATUS_NOT_INITIALIZED);
		} else if (!stream.session_active || command->session_id != stream.session_id) {
			stream_queue_result(command, MSENSE_SENSOR_STREAM_STATUS_WRONG_SESSION);
		} else {
			stream.response_kind = STREAM_RESPONSE_END;
			stream.response_connection_generation = command->connection_generation;
			stream_begin_terminal(MSENSE_SENSOR_STREAM_STATUS_STOPPED);
		}
		break;
	case STREAM_COMMAND_RESULT:
		stream_queue_result(command, command->status);
		break;
	default:
		break;
	}
}

static void stream_note_busy(void)
{
	if (stream.no_progress_since_ms == 0) {
		stream.no_progress_since_ms = k_uptime_get();
	}
}

static bool stream_submit(uint8_t kind, uint16_t length, uint16_t sensor_bytes)
{
	int ret;

	if (tx.busy || stream.conn == NULL) {
		return false;
	}
	if (++next_tx_token == 0U) {
		++next_tx_token;
	}
	tx.token = next_tx_token;
	tx.kind = kind;
	tx.session_generation = stream.session_generation;
	tx.connection_generation = stream.connection_generation;
	tx.params.attr = STREAM_TX_ATTR;
	tx.params.data = tx.data;
	tx.params.len = length;
	tx.params.func = stream_notify_complete;
	tx.params.user_data = (void *)tx.token;
	tx.busy = true;
	stream_note_busy();
	ret = bt_gatt_notify_cb(stream.conn, &tx.params);
	if (ret != 0) {
		tx.busy = false;
		stream.retry_after_ms = k_uptime_get() + STREAM_RETRY_DELAY_MS;
		if (kind == STREAM_TX_END) {
			stream_request_disconnect();
			stream_session_retire(true);
		}
		return false;
	}
	stream.retry_after_ms = 0;
	if (kind == STREAM_TX_ACK) {
		stream.ack_pending = false;
	} else if (kind == STREAM_TX_DATA) {
		stream_consume_data(sensor_bytes);
		if (!stream.infinity &&
		    stream.future_enqueued == stream.future_quota_records &&
		    stream.data_byte_offset == MSENSE_SENSOR_STREAM_FINITE_BYTES) {
			stream_begin_terminal(MSENSE_SENSOR_STREAM_STATUS_SUCCESS);
		}
	} else if (kind == STREAM_TX_END) {
		stream.terminal_pending = false;
		stream.terminal_submitted = true;
	} else if (kind == STREAM_TX_RESULT) {
		stream.result_pending = false;
	}
	stream_command_answered(kind == STREAM_TX_ACK ? STREAM_RESPONSE_ACK :
		kind == STREAM_TX_END ? STREAM_RESPONSE_END :
		kind == STREAM_TX_RESULT ? STREAM_RESPONSE_RESULT : STREAM_RESPONSE_NONE);
	return true;
}

static void stream_notify_complete(struct bt_conn *conn, void *user_data)
{
	uintptr_t token = (uintptr_t)user_data;
	uint8_t kind;

	ARG_UNUSED(conn);
	if (!tx.busy || tx.token != token) {
		return;
	}
	kind = tx.kind;
	if (tx.connection_generation != stream.connection_generation ||
	    (kind != STREAM_TX_RESULT &&
	     tx.session_generation != stream.session_generation)) {
		tx.busy = false;
		stream_wake();
		return;
	}
	tx.busy = false;
	stream.no_progress_since_ms = 0;
	if (kind == STREAM_TX_ACK) {
		stream.ack_complete = true;
	} else if (kind == STREAM_TX_END) {
		stream.terminal_complete = true;
	}
	stream_wake();
}

static bool stream_prepare_ack(void)
{
	uint16_t total;

	if (!stream.session_active || !stream.ack_pending || tx.busy) {
		return false;
	}
	total = MSENSE_SENSOR_STREAM_HEADER_BYTES + MSENSE_SENSOR_STREAM_START_ACK_BYTES;
	stream_write_header(tx.data, MSENSE_SENSOR_STREAM_MESSAGE_START_ACK,
			    stream.session_id, MSENSE_SENSOR_STREAM_START_ACK_BYTES);
	tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES] = stream.infinity ? 1U : 0U;
	tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES + 1U] = MSENSE_SENSOR_STREAM_HISTORY_UNITS;
	memset(&tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES + 2U], 0, 6U);
	sys_put_le64(stream.infinity ? 0U : MSENSE_SENSOR_STREAM_FINITE_BYTES,
		     &tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES + 8U]);
	return stream_submit(STREAM_TX_ACK, total, 0U);
}

static bool stream_prepare_data(void)
{
	uint64_t available;
	uint16_t sensor_bytes;
	uint16_t total;

	if (!stream.session_active || !stream.ack_complete || stream.terminal_pending ||
	    stream.terminal_submitted || tx.busy) {
		return false;
	}
	available = (stream.data_byte_offset < MSENSE_SENSOR_STREAM_HISTORY_BYTES ?
		MSENSE_SENSOR_STREAM_HISTORY_BYTES - stream.data_byte_offset : 0U) +
		stream_live_pending_bytes();
	if (available == 0U) {
		return false;
	}
	sensor_bytes = (uint16_t)MIN(available, stream.max_sensor_bytes);
	if (stream.data_byte_offset > UINT64_MAX - sensor_bytes) {
		stream_begin_terminal(MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR);
		return true;
	}
	if (!stream_copy_next_data(&tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES +
					     MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES],
				   sensor_bytes)) {
		return false;
	}
	stream_write_header(tx.data, MSENSE_SENSOR_STREAM_MESSAGE_DATA, stream.session_id,
			    MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES + sensor_bytes);
	sys_put_le64(stream.data_byte_offset, &tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES]);
	total = MSENSE_SENSOR_STREAM_HEADER_BYTES + MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES +
		sensor_bytes;
	return stream_submit(STREAM_TX_DATA, total, sensor_bytes);
}

static bool stream_prepare_terminal(void)
{
	uint16_t total;

	if (!stream.session_active || !stream.terminal_pending ||
	    stream.terminal_submitted || !stream.ack_complete || tx.busy) {
		return false;
	}
	stream_write_header(tx.data, MSENSE_SENSOR_STREAM_MESSAGE_END, stream.session_id,
			    MSENSE_SENSOR_STREAM_END_BYTES);
	sys_put_le16(stream.terminal_status, &tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES]);
	total = MSENSE_SENSOR_STREAM_HEADER_BYTES + MSENSE_SENSOR_STREAM_END_BYTES;
	return stream_submit(STREAM_TX_END, total, 0U);
}

static bool stream_prepare_result(void)
{
	uint16_t total;

	if (!stream.result_pending || tx.busy) {
		return false;
	}
	if (stream.result_connection_generation != stream.connection_generation ||
	    stream.conn == NULL || !stream.notifications_enabled) {
		stream.result_pending = false;
		stream_command_answered(STREAM_RESPONSE_RESULT);
		return true;
	}
	stream_write_header(tx.data, MSENSE_SENSOR_STREAM_MESSAGE_RESULT,
			    stream.result_session_id, MSENSE_SENSOR_STREAM_RESULT_BYTES);
	sys_put_le16(stream.result_status, &tx.data[MSENSE_SENSOR_STREAM_HEADER_BYTES]);
	total = MSENSE_SENSOR_STREAM_HEADER_BYTES + MSENSE_SENSOR_STREAM_RESULT_BYTES;
	return stream_submit(STREAM_TX_RESULT, total, 0U);
}

static bool stream_finalize_terminal(void)
{
	if (!stream.session_active || !stream.terminal_submitted ||
	    !stream.terminal_complete) {
		return false;
	}
	stream_session_retire(false);
	return true;
}

static bool stream_has_immediate_work(void)
{
	bool command_ready;
	bool ingress_ready;
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	command_ready = publication.command_ready;
	ingress_ready = publication.ingress[publication.ingress_head].state ==
		STREAM_INGRESS_READY;
	k_spin_unlock(&publication.lock, key);
	return command_ready || ingress_ready || stream.ack_pending ||
	       stream.terminal_pending || stream.result_pending ||
	       stream.terminal_complete ||
	       (stream.session_active && stream.ack_complete &&
		!stream.terminal_pending && !stream.terminal_submitted &&
		(stream.data_byte_offset < MSENSE_SENSOR_STREAM_HISTORY_BYTES ||
		 stream_live_pending_bytes() != 0U));
}

static void stream_owner_work_handler(struct k_work *work)
{
	uint8_t actions;
	bool retry_blocked;
	int64_t now;

	ARG_UNUSED(work);
	(void)stream_sync_publication();
	now = k_uptime_get();
	if (stream.no_progress_since_ms != 0 &&
	    now - stream.no_progress_since_ms >=
		CONFIG_MSENSE_SENSOR_STREAM_NO_PROGRESS_TIMEOUT_MS) {
		stream_request_disconnect();
		stream_session_retire(true);
		stream.result_pending = false;
		stream_command_answered(STREAM_RESPONSE_RESULT);
	}

	for (actions = 0U; actions < STREAM_OWNER_BUDGET; actions++) {
		struct stream_command command;
		bool progressed = false;

		(void)stream_sync_publication();
		if (stream_finalize_terminal()) {
			continue;
		}
		if (!tx.busy && stream_take_command(&command)) {
			stream_process_command(&command);
			continue;
		}
		now = k_uptime_get();
		retry_blocked = stream.retry_after_ms != 0 && now < stream.retry_after_ms;
		if (!retry_blocked && stream_prepare_ack()) {
			progressed = true;
		} else if (!retry_blocked && stream_prepare_terminal()) {
			progressed = true;
		} else if (!retry_blocked && stream_prepare_result()) {
			progressed = true;
		} else if (!retry_blocked && stream_prepare_data()) {
			progressed = true;
		} else if (stream_process_ingress()) {
			progressed = true;
		}
		if (!progressed) {
			break;
		}
	}

	if (!tx.busy && stream_has_immediate_work() &&
	    (stream.retry_after_ms == 0 || k_uptime_get() >= stream.retry_after_ms)) {
		stream_wake();
	} else {
		int64_t deadline = 0;

		if (stream.retry_after_ms > k_uptime_get()) {
			deadline = stream.retry_after_ms;
		}
		if (stream.no_progress_since_ms != 0) {
			int64_t progress_deadline = stream.no_progress_since_ms +
				CONFIG_MSENSE_SENSOR_STREAM_NO_PROGRESS_TIMEOUT_MS;

			if (deadline == 0 || progress_deadline < deadline) {
				deadline = progress_deadline;
			}
		}
		if (deadline != 0) {
			int64_t delay = MAX(deadline - k_uptime_get(), 1);

			/* Do not replace an immediate producer/command wake with this timer. */
			(void)k_work_schedule(&stream_owner_work, K_MSEC(delay));
		}
	}
}

static void stream_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn *held;
	struct bt_conn *old = NULL;
	k_spinlock_key_t key;

	if (err != 0U) {
		return;
	}
	held = bt_conn_ref(conn);
	if (held == NULL) {
		return;
	}
	key = k_spin_lock(&publication.lock);
	if (publication.conn != conn) {
		old = publication.conn;
		publication.conn = held;
		held = NULL;
		publication.connection_generation++;
		if (publication.connection_generation == 0U) {
			publication.connection_generation++;
		}
		publication.notifications_enabled = false;
		publication.command_pending = false;
		publication.command_ready = false;
	}
	k_spin_unlock(&publication.lock, key);
	if (held != NULL) {
		bt_conn_unref(held);
	}
	if (old != NULL) {
		bt_conn_unref(old);
	}
	stream_wake();
}

static void stream_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *old = NULL;
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	if (publication.conn == conn) {
		old = publication.conn;
		publication.conn = NULL;
		publication.connection_generation++;
		if (publication.connection_generation == 0U) {
			publication.connection_generation++;
		}
		publication.notifications_enabled = false;
		publication.command_pending = false;
		publication.command_ready = false;
	}
	k_spin_unlock(&publication.lock, key);
	if (old != NULL) {
		LOG_INF("NUS stream disconnected: reason 0x%02x", reason);
		bt_conn_unref(old);
	}
	stream_wake();
}

static struct bt_conn_cb stream_conn_callbacks = {
	.connected = stream_connected,
	.disconnected = stream_disconnected,
};

static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	k_spinlock_key_t key;

	ARG_UNUSED(attr);
	key = k_spin_lock(&publication.lock);
	if (publication.conn != NULL) {
		publication.notifications_enabled = (value & BT_GATT_CCC_NOTIFY) != 0U;
		if (!publication.notifications_enabled) {
			publication.command_pending = false;
			publication.command_ready = false;
		}
	}
	k_spin_unlock(&publication.lock, key);
	stream_wake();
}

static ssize_t stream_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset,
			       uint8_t flags)
{
	const uint8_t *bytes = buf;
	struct stream_command command = { 0 };
	k_spinlock_key_t key;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);
	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != MSENSE_SENSOR_STREAM_COMMAND_BYTES) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	command.session_id = sys_get_le32(&bytes[4]);
	if (bytes[0] != MSENSE_SENSOR_STREAM_MAGIC0 ||
	    bytes[1] != MSENSE_SENSOR_STREAM_MAGIC1 || command.session_id == 0U) {
		command.kind = STREAM_COMMAND_RESULT;
		command.status = MSENSE_SENSOR_STREAM_STATUS_INVALID_COMMAND;
	} else if (bytes[2] != MSENSE_SENSOR_STREAM_PROTOCOL_VERSION) {
		command.kind = STREAM_COMMAND_RESULT;
		command.status = MSENSE_SENSOR_STREAM_STATUS_UNSUPPORTED_VERSION;
	} else if (bytes[3] == MSENSE_SENSOR_STREAM_OPCODE_START) {
		command.kind = STREAM_COMMAND_START;
	} else if (bytes[3] == MSENSE_SENSOR_STREAM_OPCODE_START_INFINITY) {
		command.kind = STREAM_COMMAND_START_INFINITY;
	} else if (bytes[3] == MSENSE_SENSOR_STREAM_OPCODE_STOP) {
		command.kind = STREAM_COMMAND_STOP;
	} else {
		command.kind = STREAM_COMMAND_RESULT;
		command.status = MSENSE_SENSOR_STREAM_STATUS_INVALID_COMMAND;
	}

	key = k_spin_lock(&publication.lock);
	if (publication.conn != conn) {
		k_spin_unlock(&publication.lock, key);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	if (publication.command_pending) {
		k_spin_unlock(&publication.lock, key);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	}
	command.connection_generation = publication.connection_generation;
	publication.command = command;
	publication.command_pending = true;
	publication.command_ready = true;
	k_spin_unlock(&publication.lock, key);
	stream_wake();
	return len;
}

static void stream_publish_acquisition(bool recording, uint16_t status)
{
	k_spinlock_key_t key = k_spin_lock(&publication.lock);

	if (publication.configured) {
		publication.recording = recording;
		publication.acquisition_status = status;
		publication.acquisition_generation++;
		if (publication.acquisition_generation == 0U) {
			publication.acquisition_generation++;
		}
	}
	k_spin_unlock(&publication.lock, key);
	stream_wake();
}

int msense_sensor_stream_init(const struct msense_sensor_stream_config *config)
{
	int ret;
	k_spinlock_key_t key;

	if (!stream_config_is_valid(config)) {
		return -EINVAL;
	}
	ret = bt_conn_cb_register(&stream_conn_callbacks);
	if (ret != 0 && ret != -EEXIST) {
		return ret;
	}
	key = k_spin_lock(&publication.lock);
	if (publication.configured) {
		k_spin_unlock(&publication.lock, key);
		return 0;
	}
	publication.config = *config;
	publication.configured = true;
	publication.acquisition_generation = 1U;
	publication.acquisition_status = MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING;
	k_spin_unlock(&publication.lock, key);

	stream.record_size = config->record_size;
	stream.history_record_count = config->history_record_count;
	stream.future_quota_records = config->forward_record_count;
	stream.live_capacity = MSENSE_SENSOR_STREAM_HISTORY_BYTES * 2U / config->record_size;
	stream.initialized = true;
	stream.acquisition_generation = 1U;
	stream_reset_history();
	memset(&tx.params, 0, sizeof(tx.params));
	stream_wake();
	return 0;
}

void msense_sensor_stream_recording_started(void)
{
	stream_publish_acquisition(true, MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING);
}

void msense_sensor_stream_recording_stopped(void)
{
	stream_publish_acquisition(false, MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING);
}

void msense_sensor_stream_storage_failed(int error)
{
	ARG_UNUSED(error);
	stream_publish_acquisition(false, MSENSE_SENSOR_STREAM_STATUS_STORAGE_ERROR);
}

void msense_sensor_stream_recording_failed(int error)
{
	ARG_UNUSED(error);
	stream_publish_acquisition(false, MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR);
}

int msense_sensor_stream_accept_record(const void *record, size_t record_size)
{
	struct stream_ingress_slot *slot;
	uint32_t tail;
	k_spinlock_key_t key;

	if (record == NULL) {
		return -EINVAL;
	}
	key = k_spin_lock(&publication.lock);
	if (!publication.configured || !publication.recording) {
		k_spin_unlock(&publication.lock, key);
		return 0;
	}
	if (record_size != publication.config.record_size) {
		k_spin_unlock(&publication.lock, key);
		return -EMSGSIZE;
	}
	tail = publication.ingress_tail;
	slot = &publication.ingress[tail];
	if (slot->state != STREAM_INGRESS_FREE) {
		if (!publication.ingress_loss_pending) {
			publication.ingress_loss_pending = true;
			publication.acquisition_generation++;
			if (publication.acquisition_generation == 0U) {
				publication.acquisition_generation++;
			}
			publication.acquisition_status =
				MSENSE_SENSOR_STREAM_STATUS_BUFFER_OVERFLOW;
		}
		k_spin_unlock(&publication.lock, key);
		stream_wake();
		return 0;
	}
	slot->state = STREAM_INGRESS_FILLING;
	slot->generation = publication.acquisition_generation;
	publication.ingress_tail = (tail + 1U) % ARRAY_SIZE(publication.ingress);
	k_spin_unlock(&publication.lock, key);

	memcpy(slot->data, record, record_size);

	key = k_spin_lock(&publication.lock);
	slot->state = STREAM_INGRESS_READY;
	k_spin_unlock(&publication.lock, key);
	stream_wake();
	return 0;
}
