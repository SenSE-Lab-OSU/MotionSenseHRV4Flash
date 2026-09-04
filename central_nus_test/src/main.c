/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Minimal, one-off BLE Central test tool for the MotionSense sensor stream. */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "msense_sensor_stream_protocol.h"

#define COMMAND_LINE_BYTES 96U
/* Holds complete phase-throughput summaries with full-width numeric fields. */
#define CONTROL_EVENT_BYTES 320U
#define RELAY_MAX_NUS_BYTES 512U
#define RELAY_HEADER_BYTES 12U
#define RELAY_FRAME_BYTES (RELAY_HEADER_BYTES + RELAY_MAX_NUS_BYTES)
#define RELAY_FRAME_COUNT 2U
#define RELAY_TX_DRIVER_TIMEOUT_US 250000U
#define RELAY_TX_COMPLETION_TIMEOUT_MS 300U
#define RELAY_TX_ABORT_TIMEOUT_MS 50U
/* Holds the complete 32 KiB history burst while the VCOM relay drains it. */
#define RELAY_QUEUE_DEPTH 80U
#define REQUIRED_ATT_MTU 128U

#define NUS_HEADER_MESSAGE_TYPE_OFFSET 3U
#define NUS_HEADER_SESSION_ID_OFFSET 4U
#define NUS_HEADER_PAYLOAD_LENGTH_OFFSET 8U
#define NUS_HEADER_FLAGS_OFFSET 10U
#define NUS_PAYLOAD_OFFSET MSENSE_SENSOR_STREAM_HEADER_BYTES

#define START_ACK_DEVICE_TYPE_OFFSET 0U
#define START_ACK_FORMAT_VERSION_OFFSET 1U
#define START_ACK_RECORD_SIZE_OFFSET 2U
#define START_ACK_RATE_NUMERATOR_OFFSET 4U
#define START_ACK_RATE_DENOMINATOR_OFFSET 8U
#define START_ACK_HISTORY_COUNT_OFFSET 12U
#define START_ACK_FORWARD_COUNT_OFFSET 16U
#define START_ACK_TOTAL_BYTES_OFFSET 20U
#define START_ACK_DEVICE_ID_OFFSET 24U
#define START_ACK_NAME_LENGTH_OFFSET 32U
#define START_ACK_NAME_OFFSET 33U
#define START_ACK_GIT_COMMIT_OFFSET 49U
#define START_ACK_TREE_STATE_OFFSET 89U
#define START_ACK_RESERVED_OFFSET 90U

#define DATA_SEQUENCE_OFFSET 0U
#define DATA_FIRST_RECORD_OFFSET 4U
#define DATA_RECORD_COUNT_OFFSET 8U
#define DATA_PHASE_OFFSET 10U
#define DATA_RESERVED_OFFSET 11U
#define DATA_PHASE_HISTORY 0U
#define DATA_PHASE_FORWARD 1U

#define END_STATUS_OFFSET 0U
#define END_STATE_OFFSET 2U
#define END_RESERVED_OFFSET 3U
#define END_HISTORY_SENT_OFFSET 4U
#define END_FORWARD_CAPTURED_OFFSET 8U
#define END_SENSOR_BYTES_OFFSET 12U
#define END_DATA_MESSAGES_OFFSET 16U
#define END_DETAIL_OFFSET 20U

#define RESULT_STATUS_OFFSET 0U
#define RESULT_STATE_OFFSET 2U
#define RESULT_RESERVED_OFFSET 3U

BUILD_ASSERT(RELAY_MAX_NUS_BYTES >= CONFIG_BT_L2CAP_TX_MTU - 3U,
	     "Relay storage must hold one complete NUS notification");

enum tester_state {
	TESTER_IDLE,
	TESTER_SCANNING,
	TESTER_CONNECTING,
	TESTER_MTU_EXCHANGE,
	TESTER_DISCOVERING,
	TESTER_SUBSCRIBING,
	TESTER_READY,
	TESTER_START_PENDING,
	TESTER_RECEIVING,
	TESTER_COMPLETE,
	TESTER_FAILED,
};

enum scan_target {
	SCAN_TARGET_LIST,
	SCAN_TARGET_PPG,
	SCAN_TARGET_ECG,
	SCAN_TARGET_ANY,
};

struct command_line {
	char text[COMMAND_LINE_BYTES];
};

struct control_event {
	char text[CONTROL_EVENT_BYTES];
};

struct relay_message {
	uint32_t sequence;
	uint16_t length;
	uint8_t data[RELAY_MAX_NUS_BYTES];
};

struct relay_frame {
	uint8_t data[RELAY_FRAME_BYTES];
};

enum relay_tx_terminal {
	RELAY_TX_PENDING,
	RELAY_TX_DONE,
	RELAY_TX_ABORTED,
};

struct stream_metadata {
	uint32_t history_records;
	uint32_t forward_records;
	uint32_t total_sensor_bytes;
	uint32_t rate_numerator;
	uint32_t rate_denominator;
	uint32_t expected_sequence;
	uint32_t expected_record_index;
	uint32_t received_sensor_bytes;
	uint32_t received_data_messages;
	uint32_t session_id;
	uint16_t record_size;
	uint8_t device_type;
	uint8_t record_format_version;
	uint8_t device_id[8];
	uint8_t git_tree_state;
	char device_name[17];
	char git_commit[41];
};

struct phase_statistics {
	int64_t first_data_ms;
	int64_t last_data_ms;
	uint32_t raw_nus_bytes;
	uint32_t sensor_bytes;
	uint32_t data_notifications;
	uint32_t max_gap_ms;
	bool has_data;
};

struct stream_statistics {
	struct phase_statistics total;
	struct phase_statistics history;
	struct phase_statistics forward;
	int64_t request_start_ms;
	bool request_started;
	bool history_reported;
};

struct ble_link_info {
	uint32_t interval_ms_x100;
	uint32_t timeout_ms;
	uint16_t interval_units;
	uint16_t latency;
	uint16_t tx_max_len;
	uint16_t tx_max_time;
	uint16_t rx_max_len;
	uint16_t rx_max_time;
	uint8_t tx_phy;
	uint8_t rx_phy;
};

struct throughput_values {
	uint32_t elapsed_ms;
	uint32_t mean_notification_bytes;
	uint32_t notifications_per_second_x10;
	uint32_t raw_kib_per_second_x10;
	uint32_t sensor_kib_per_second_x10;
};

struct tester_context {
	struct k_spinlock lock;
	struct bt_conn *conn;
	struct stream_metadata metadata;
	struct stream_statistics statistics;
	struct ble_link_info link;
	enum tester_state state;
	enum scan_target scan_target;
	uint32_t next_session_id;
	uint32_t relay_sequence;
	uint32_t relay_dropped;
	uint32_t last_command_session_id;
	uint16_t att_mtu;
	uint8_t last_command_opcode;
	bool subscribed;
	bool write_pending;
};

static const struct device *const command_uart =
	DEVICE_DT_GET(DT_ALIAS(msense_command_uart));
static const struct device *const relay_uart =
	DEVICE_DT_GET(DT_ALIAS(msense_relay_uart));

static struct tester_context tester = {
	.state = TESTER_IDLE,
	.scan_target = SCAN_TARGET_LIST,
	.next_session_id = 1U,
};

static struct bt_nus_client nus_client;
static struct bt_gatt_exchange_params exchange_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_write_params command_write_params;
static uint8_t command_write_data[MSENSE_SENSOR_STREAM_COMMAND_BYTES];
static char command_rx_buffer[COMMAND_LINE_BYTES];
static size_t command_rx_length;

K_MSGQ_DEFINE(command_queue, sizeof(struct command_line), 8, 4);
K_MSGQ_DEFINE(control_event_queue, sizeof(struct control_event), 16, 4);
/*
 * NUS notifications run on the Bluetooth RX thread.  Keep the full relay
 * message in this static slab rather than on that thread's limited stack.
 * The queue carries pointers to slab blocks, so both enqueue operations can
 * remain nonblocking.
 */
K_MEM_SLAB_DEFINE_STATIC(relay_message_slab, sizeof(struct relay_message),
			 RELAY_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(relay_queue, sizeof(struct relay_message *), RELAY_QUEUE_DEPTH, 4);
K_SEM_DEFINE(relay_tx_complete, 0, 1);

static struct relay_frame relay_frames[RELAY_FRAME_COUNT];
static atomic_t relay_tx_active;
static atomic_t relay_tx_bytes;
static atomic_t relay_tx_terminal;
static atomic_ptr_t relay_tx_buffer;

static void start_scan(void);
static void command_write_complete(struct bt_conn *conn, uint8_t err,
				   struct bt_gatt_write_params *params);
static void mark_protocol_failure(const char *reason);

static const char *tester_state_name(enum tester_state state)
{
	switch (state) {
	case TESTER_IDLE:
		return "IDLE";
	case TESTER_SCANNING:
		return "SCANNING";
	case TESTER_CONNECTING:
		return "CONNECTING";
	case TESTER_MTU_EXCHANGE:
		return "MTU_EXCHANGE";
	case TESTER_DISCOVERING:
		return "DISCOVERING";
	case TESTER_SUBSCRIBING:
		return "SUBSCRIBING";
	case TESTER_READY:
		return "READY";
	case TESTER_START_PENDING:
		return "START_PENDING";
	case TESTER_RECEIVING:
		return "RECEIVING";
	case TESTER_COMPLETE:
		return "COMPLETE";
	case TESTER_FAILED:
		return "FAILED";
	default:
		return "UNKNOWN";
	}
}

static const char *status_name(uint16_t status)
{
	switch (status) {
	case MSENSE_SENSOR_STREAM_STATUS_SUCCESS:
		return "SUCCESS";
	case MSENSE_SENSOR_STREAM_STATUS_NOT_RECORDING:
		return "NOT_RECORDING";
	case MSENSE_SENSOR_STREAM_STATUS_HISTORY_NOT_READY:
		return "HISTORY_NOT_READY";
	case MSENSE_SENSOR_STREAM_STATUS_NOT_SUBSCRIBED:
		return "NOT_SUBSCRIBED";
	case MSENSE_SENSOR_STREAM_STATUS_BUSY:
		return "BUSY";
	case MSENSE_SENSOR_STREAM_STATUS_MTU_TOO_SMALL:
		return "MTU_TOO_SMALL";
	case MSENSE_SENSOR_STREAM_STATUS_INVALID_COMMAND:
		return "INVALID_COMMAND";
	case MSENSE_SENSOR_STREAM_STATUS_UNSUPPORTED_VERSION:
		return "UNSUPPORTED_VERSION";
	case MSENSE_SENSOR_STREAM_STATUS_CANCELLED:
		return "CANCELLED";
	case MSENSE_SENSOR_STREAM_STATUS_STORAGE_ERROR:
		return "STORAGE_ERROR";
	case MSENSE_SENSOR_STREAM_STATUS_INTERNAL_ERROR:
		return "INTERNAL_ERROR";
	case MSENSE_SENSOR_STREAM_STATUS_NOT_INITIALIZED:
		return "NOT_INITIALIZED";
	case MSENSE_SENSOR_STREAM_STATUS_WRONG_SESSION:
		return "WRONG_SESSION";
	case MSENSE_SENSOR_STREAM_STATUS_DISCONNECTED:
		return "DISCONNECTED";
	default:
		return "UNKNOWN";
	}
}

static void command_write_bytes(const uint8_t *data, size_t length)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		uart_poll_out(command_uart, data[index]);
	}
}

static void command_printf(const char *format, ...)
{
	char text[CONTROL_EVENT_BYTES];
	va_list args;
	int length;

	va_start(args, format);
	length = vsnprintk(text, sizeof(text), format, args);
	va_end(args);
	if (length < 0) {
		return;
	}
	if ((size_t)length >= sizeof(text)) {
		length = sizeof(text) - 1U;
	}
	command_write_bytes((const uint8_t *)text, (size_t)length);
	command_write_bytes((const uint8_t *)"\r\n", 2U);
}

static void post_event(const char *format, ...)
{
	struct control_event event;
	va_list args;

	va_start(args, format);
	(void)vsnprintk(event.text, sizeof(event.text), format, args);
	va_end(args);
	(void)k_msgq_put(&control_event_queue, &event, K_NO_WAIT);
}

static void relay_uart_callback(const struct device *device, struct uart_event *event,
				void *user_data)
{
	const uint8_t *active_buffer;
	enum relay_tx_terminal terminal;

	ARG_UNUSED(device);
	ARG_UNUSED(user_data);

	switch (event->type) {
	case UART_TX_DONE:
		terminal = RELAY_TX_DONE;
		break;
	case UART_TX_ABORTED:
		terminal = RELAY_TX_ABORTED;
		break;
	default:
		return;
	}

	active_buffer = (const uint8_t *)atomic_ptr_get(&relay_tx_buffer);
	/* Ignore stale terminal events from the previous frame buffer. */
	if (event->data.tx.buf != active_buffer) {
		return;
	}
	/* Only the relay thread initiates TX; ignore a duplicate terminal event. */
	if (!atomic_cas(&relay_tx_active, 1, 0)) {
		return;
	}
	atomic_set(&relay_tx_bytes, (atomic_val_t)event->data.tx.len);
	atomic_set(&relay_tx_terminal, (atomic_val_t)terminal);
	k_sem_give(&relay_tx_complete);
}

static enum relay_tx_terminal relay_wait_for_terminal(k_timeout_t timeout)
{
	if (k_sem_take(&relay_tx_complete, timeout) != 0) {
		/* Catch a terminal event that raced the timeout. */
		return (enum relay_tx_terminal)atomic_get(&relay_tx_terminal);
	}

	return (enum relay_tx_terminal)atomic_get(&relay_tx_terminal);
}

static bool relay_send_frame(struct relay_frame *frame, size_t frame_length,
			     bool *terminal_missing)
{
	enum relay_tx_terminal terminal;
	int err;

	*terminal_missing = false;
	k_sem_reset(&relay_tx_complete);
	atomic_set(&relay_tx_bytes, 0);
	atomic_set(&relay_tx_terminal, RELAY_TX_PENDING);
	atomic_ptr_set(&relay_tx_buffer, frame->data);
	atomic_set(&relay_tx_active, 1);
	err = uart_tx(relay_uart, frame->data, frame_length, RELAY_TX_DRIVER_TIMEOUT_US);
	if (err != 0) {
		atomic_set(&relay_tx_active, 0);
		post_event("ERROR relay TX start=%d; capture invalid", err);
		return false;
	}

	terminal = relay_wait_for_terminal(K_MSEC(RELAY_TX_COMPLETION_TIMEOUT_MS));
	if (terminal == RELAY_TX_DONE) {
		if ((size_t)atomic_get(&relay_tx_bytes) == frame_length) {
			return true;
		}
		post_event("ERROR relay TX short completion bytes=%u/%u; capture invalid",
			   (uint32_t)atomic_get(&relay_tx_bytes), (uint32_t)frame_length);
		return false;
	}
	if (terminal == RELAY_TX_ABORTED) {
		post_event("ERROR relay TX aborted bytes=%u/%u; capture invalid",
			   (uint32_t)atomic_get(&relay_tx_bytes), (uint32_t)frame_length);
		return false;
	}

	post_event("ERROR relay TX timeout after %u ms; aborting",
		   RELAY_TX_COMPLETION_TIMEOUT_MS);
	err = uart_tx_abort(relay_uart);
	terminal = relay_wait_for_terminal(K_MSEC(RELAY_TX_ABORT_TIMEOUT_MS));
	if (terminal == RELAY_TX_DONE) {
		post_event("ERROR relay TX completed after timeout (abort=%d); capture invalid", err);
		return false;
	}
	if (terminal == RELAY_TX_ABORTED) {
		post_event("ERROR relay TX aborted after timeout (abort=%d bytes=%u/%u); "
			   "capture invalid", err, (uint32_t)atomic_get(&relay_tx_bytes),
			   (uint32_t)frame_length);
		return false;
	}

	/* Keep the static frame intact if the driver did not confirm it stopped. */
	*terminal_missing = true;
	post_event("ERROR relay TX no terminal event after abort=%d; relay stopped", err);
	return false;
}

static void relay_thread(void *arg1, void *arg2, void *arg3)
{
	struct relay_message *message;
	struct relay_frame *frame;
	bool terminal_missing;
	size_t frame_length;
	uint8_t frame_index = 0U;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		if (k_msgq_get(&relay_queue, &message, K_FOREVER) != 0) {
			continue;
		}
		if (message == NULL) {
			post_event("ERROR relay null message; capture invalid");
			mark_protocol_failure("relay null message");
			continue;
		}
		if (message->length > RELAY_MAX_NUS_BYTES) {
			post_event("ERROR relay frame too large: %u; capture invalid", message->length);
			mark_protocol_failure("relay frame too large");
			k_mem_slab_free(&relay_message_slab, (void *)message);
			continue;
		}

		frame = &relay_frames[frame_index];
		memcpy(frame->data, "MRLY", 4U);
		frame->data[4] = 1U;
		frame->data[5] = 1U;
		sys_put_le16(message->length, &frame->data[6]);
		sys_put_le32(message->sequence, &frame->data[8]);
		memcpy(&frame->data[RELAY_HEADER_BYTES], message->data, message->length);
		frame_length = RELAY_HEADER_BYTES + message->length;

		/* The asynchronous UART owns only frame->data, not this slab block. */
		k_mem_slab_free(&relay_message_slab, (void *)message);
		message = NULL;
		if (!relay_send_frame(frame, frame_length, &terminal_missing)) {
			mark_protocol_failure("relay TX failed");
			if (terminal_missing) {
				/* The driver may still own frame->data, so never reuse either buffer. */
				return;
			}
		}
		frame_index = (frame_index + 1U) % ARRAY_SIZE(relay_frames);
	}
}

K_THREAD_DEFINE(relay_thread_id, 2048, relay_thread, NULL, NULL, NULL, 8, 0, 0);

static bool is_lower_hex_ascii(const uint8_t *data, size_t length)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		if (!((data[index] >= '0' && data[index] <= '9') ||
		      (data[index] >= 'a' && data[index] <= 'f'))) {
			return false;
		}
	}

	return true;
}

static bool all_zero(const uint8_t *data, size_t length)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		if (data[index] != 0U) {
			return false;
		}
	}

	return true;
}

static uint32_t elapsed_ms_between(int64_t start_ms, int64_t end_ms)
{
	int64_t elapsed_ms;

	if (end_ms <= start_ms) {
		return 0U;
	}
	elapsed_ms = end_ms - start_ms;
	if ((uint64_t)elapsed_ms > UINT32_MAX) {
		return UINT32_MAX;
	}

	return (uint32_t)elapsed_ms;
}

static uint32_t rate_per_second_x10(uint32_t value, uint32_t elapsed_ms)
{
	if (elapsed_ms == 0U) {
		return 0U;
	}

	return (uint32_t)((uint64_t)value * 10000U / elapsed_ms);
}

static uint32_t kib_per_second_x10(uint32_t bytes, uint32_t elapsed_ms)
{
	if (elapsed_ms == 0U) {
		return 0U;
	}

	return (uint32_t)((uint64_t)bytes * 10000U / (1024U * elapsed_ms));
}

static void phase_statistics_record_data(struct phase_statistics *statistics,
					 uint16_t raw_nus_length, uint32_t sensor_bytes,
					 int64_t received_ms)
{
	uint32_t gap_ms;

	if (statistics->has_data) {
		gap_ms = elapsed_ms_between(statistics->last_data_ms, received_ms);
		if (gap_ms > statistics->max_gap_ms) {
			statistics->max_gap_ms = gap_ms;
		}
	} else {
		statistics->first_data_ms = received_ms;
		statistics->has_data = true;
	}

	statistics->last_data_ms = received_ms;
	statistics->raw_nus_bytes += raw_nus_length;
	statistics->sensor_bytes += sensor_bytes;
	statistics->data_notifications++;
}

static void stream_statistics_record_data(struct stream_statistics *statistics, uint8_t phase,
					  uint16_t raw_nus_length, uint32_t sensor_bytes,
					  int64_t received_ms)
{
	struct phase_statistics *phase_statistics =
		phase == DATA_PHASE_HISTORY ? &statistics->history : &statistics->forward;

	phase_statistics_record_data(&statistics->total, raw_nus_length, sensor_bytes, received_ms);
	phase_statistics_record_data(phase_statistics, raw_nus_length, sensor_bytes, received_ms);
}

static struct throughput_values stream_throughput_values(
	const struct phase_statistics *statistics, int64_t end_ms)
{
	struct throughput_values values;

	memset(&values, 0, sizeof(values));
	if (!statistics->has_data) {
		return values;
	}

	values.elapsed_ms = elapsed_ms_between(statistics->first_data_ms, end_ms);
	values.mean_notification_bytes =
		statistics->raw_nus_bytes / statistics->data_notifications;
	values.notifications_per_second_x10 =
		rate_per_second_x10(statistics->data_notifications, values.elapsed_ms);
	values.raw_kib_per_second_x10 =
		kib_per_second_x10(statistics->raw_nus_bytes, values.elapsed_ms);
	values.sensor_kib_per_second_x10 =
		kib_per_second_x10(statistics->sensor_bytes, values.elapsed_ms);

	return values;
}

static void post_throughput_history(uint32_t session_id,
				    const struct phase_statistics *statistics,
				    int64_t request_start_ms, bool request_started)
{
	struct throughput_values values =
		stream_throughput_values(statistics, statistics->last_data_ms);
	uint32_t request_elapsed_ms = request_started ?
		elapsed_ms_between(request_start_ms, statistics->last_data_ms) : 0U;

	post_event("THROUGHPUT_HISTORY id=%u active_elapsed_ms=%u request_elapsed_ms=%u "
		   "data_notifs=%u raw_nus_bytes=%u sensor_bytes=%u mean_notif_bytes=%u "
		   "notif_s=%u.%u raw_kib_s=%u.%u sensor_kib_s=%u.%u max_gap_ms=%u",
		   session_id, values.elapsed_ms, request_elapsed_ms, statistics->data_notifications,
		   statistics->raw_nus_bytes, statistics->sensor_bytes, values.mean_notification_bytes,
		   values.notifications_per_second_x10 / 10U,
		   values.notifications_per_second_x10 % 10U, values.raw_kib_per_second_x10 / 10U,
		   values.raw_kib_per_second_x10 % 10U, values.sensor_kib_per_second_x10 / 10U,
		   values.sensor_kib_per_second_x10 % 10U, statistics->max_gap_ms);
}

static void post_throughput_forward(uint32_t session_id,
				    const struct phase_statistics *statistics)
{
	struct throughput_values values =
		stream_throughput_values(statistics, statistics->last_data_ms);

	post_event("THROUGHPUT_FORWARD id=%u active_elapsed_ms=%u data_notifs=%u "
		   "raw_nus_bytes=%u sensor_bytes=%u mean_notif_bytes=%u notif_s=%u.%u "
		   "raw_kib_s=%u.%u sensor_kib_s=%u.%u max_gap_ms=%u",
		   session_id, values.elapsed_ms, statistics->data_notifications,
		   statistics->raw_nus_bytes, statistics->sensor_bytes, values.mean_notification_bytes,
		   values.notifications_per_second_x10 / 10U,
		   values.notifications_per_second_x10 % 10U, values.raw_kib_per_second_x10 / 10U,
		   values.raw_kib_per_second_x10 % 10U, values.sensor_kib_per_second_x10 / 10U,
		   values.sensor_kib_per_second_x10 % 10U, statistics->max_gap_ms);
}

static void post_throughput_summary(uint32_t session_id,
				    const struct phase_statistics *statistics)
{
	struct throughput_values values =
		stream_throughput_values(statistics, statistics->last_data_ms);

	post_event("THROUGHPUT id=%u elapsed_ms=%u data_notifs=%u raw_nus_bytes=%u "
		   "sensor_bytes=%u mean_notif_bytes=%u notif_s=%u.%u raw_kib_s=%u.%u "
		   "sensor_kib_s=%u.%u max_gap_ms=%u",
		   session_id, values.elapsed_ms, statistics->data_notifications,
		   statistics->raw_nus_bytes, statistics->sensor_bytes,
		   values.mean_notification_bytes, values.notifications_per_second_x10 / 10U,
		   values.notifications_per_second_x10 % 10U, values.raw_kib_per_second_x10 / 10U,
		   values.raw_kib_per_second_x10 % 10U, values.sensor_kib_per_second_x10 / 10U,
		   values.sensor_kib_per_second_x10 % 10U, statistics->max_gap_ms);
}

static void print_live_throughput(uint32_t session_id,
				  const struct phase_statistics *statistics, int64_t end_ms)
{
	struct throughput_values values = stream_throughput_values(statistics, end_ms);

	command_printf("THROUGHPUT_LIVE id=%u elapsed_ms=%u data_notifs=%u raw_nus_bytes=%u "
		       "sensor_bytes=%u mean_notif_bytes=%u notif_s=%u.%u raw_kib_s=%u.%u "
		       "sensor_kib_s=%u.%u max_gap_ms=%u",
		       session_id, values.elapsed_ms, statistics->data_notifications,
		       statistics->raw_nus_bytes, statistics->sensor_bytes,
		       values.mean_notification_bytes, values.notifications_per_second_x10 / 10U,
		       values.notifications_per_second_x10 % 10U,
		       values.raw_kib_per_second_x10 / 10U,
		       values.raw_kib_per_second_x10 % 10U,
		       values.sensor_kib_per_second_x10 / 10U,
		       values.sensor_kib_per_second_x10 % 10U, statistics->max_gap_ms);
}

static void update_link_info_from_connection(struct bt_conn *conn)
{
	struct bt_conn_info info;
	k_spinlock_key_t key;

	if (bt_conn_get_info(conn, &info) != 0 || info.type != BT_CONN_TYPE_LE) {
		return;
	}

	key = k_spin_lock(&tester.lock);
	if (tester.conn == conn) {
		tester.link.interval_units = info.le.interval;
		tester.link.interval_ms_x100 = (uint32_t)info.le.interval * 125U;
		tester.link.latency = info.le.latency;
		tester.link.timeout_ms = (uint32_t)info.le.timeout * 10U;
#if defined(CONFIG_BT_USER_PHY_UPDATE)
		if (info.le.phy != NULL) {
			tester.link.tx_phy = info.le.phy->tx_phy;
			tester.link.rx_phy = info.le.phy->rx_phy;
		}
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
		if (info.le.data_len != NULL) {
			tester.link.tx_max_len = info.le.data_len->tx_max_len;
			tester.link.tx_max_time = info.le.data_len->tx_max_time;
			tester.link.rx_max_len = info.le.data_len->rx_max_len;
			tester.link.rx_max_time = info.le.data_len->rx_max_time;
		}
#endif
	}
	k_spin_unlock(&tester.lock, key);
}

static void post_ble_link(struct bt_conn *conn)
{
	struct ble_link_info link;
	uint16_t mtu;
	bool current = false;
	k_spinlock_key_t key;

	key = k_spin_lock(&tester.lock);
	if (tester.conn == conn) {
		link = tester.link;
		mtu = tester.att_mtu;
		current = true;
	}
	k_spin_unlock(&tester.lock, key);
	if (!current) {
		return;
	}

	post_event("BLE_LINK mtu=%u interval_units=%u interval_ms_x100=%u latency=%u "
		   "timeout_ms=%u tx_phy=%u rx_phy=%u tx_octets=%u rx_octets=%u "
		   "tx_time_us=%u rx_time_us=%u",
		   mtu, link.interval_units, link.interval_ms_x100, link.latency,
		   link.timeout_ms, link.tx_phy, link.rx_phy, link.tx_max_len,
		   link.rx_max_len, link.tx_max_time, link.rx_max_time);
}

static void print_ble_link(const struct ble_link_info *link, uint16_t mtu)
{
	command_printf("BLE_LINK mtu=%u interval_units=%u interval_ms_x100=%u latency=%u "
		       "timeout_ms=%u tx_phy=%u rx_phy=%u tx_octets=%u rx_octets=%u "
		       "tx_time_us=%u rx_time_us=%u",
		       mtu, link->interval_units, link->interval_ms_x100, link->latency,
		       link->timeout_ms, link->tx_phy, link->rx_phy, link->tx_max_len,
		       link->rx_max_len, link->tx_max_time, link->rx_max_time);
}

static bool geometry_is_valid(const struct stream_metadata *metadata)
{
	if (metadata->total_sensor_bytes != MSENSE_SENSOR_STREAM_SENSOR_BYTES ||
	    (uint64_t)metadata->record_size *
		    ((uint64_t)metadata->history_records + metadata->forward_records) !=
		    metadata->total_sensor_bytes) {
		return false;
	}

	if (metadata->device_type == MSENSE_SENSOR_STREAM_DEVICE_PPG) {
		return metadata->record_format_version == 1U &&
		       metadata->record_size == MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE &&
		       metadata->rate_numerator == 256U && metadata->rate_denominator == 1U &&
		       metadata->history_records == MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS &&
		       metadata->forward_records == MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS;
	}
	if (metadata->device_type == MSENSE_SENSOR_STREAM_DEVICE_ECG) {
		return metadata->record_format_version == 1U &&
		       metadata->record_size == MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE &&
		       metadata->rate_numerator == 512U && metadata->rate_denominator == 1U &&
		       metadata->history_records == MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS &&
		       metadata->forward_records == MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS;
	}

	return false;
}

static void device_id_to_hex(const uint8_t *device_id, char *hex)
{
	static const char digits[] = "0123456789ABCDEF";
	size_t index;

	for (index = 0U; index < 8U; index++) {
		hex[index * 2U] = digits[device_id[index] >> 4U];
		hex[index * 2U + 1U] = digits[device_id[index] & 0x0fU];
	}
	hex[16] = '\0';
}

static void mark_protocol_failure(const char *reason)
{
	k_spinlock_key_t key = k_spin_lock(&tester.lock);

	if (tester.state == TESTER_START_PENDING || tester.state == TESTER_RECEIVING ||
	    tester.state == TESTER_COMPLETE) {
		tester.state = TESTER_FAILED;
	}
	k_spin_unlock(&tester.lock, key);
	post_event("PROTOCOL_ERROR %s", reason);
}

static bool relay_enqueue(const uint8_t *data, uint16_t length)
{
	struct relay_message *message;
	void *slab_block;
	k_spinlock_key_t key;
	int err;

	if (length > RELAY_MAX_NUS_BYTES) {
		post_event("ERROR relay notification too large: %u", length);
		return false;
	}

	err = k_mem_slab_alloc(&relay_message_slab, &slab_block, K_NO_WAIT);
	if (err != 0) {
		key = k_spin_lock(&tester.lock);
		tester.relay_dropped++;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR relay slab exhausted; capture invalid");
		return false;
	}
	message = slab_block;

	key = k_spin_lock(&tester.lock);
	message->sequence = tester.relay_sequence++;
	k_spin_unlock(&tester.lock, key);
	message->length = length;
	memcpy(message->data, data, length);
	if (k_msgq_put(&relay_queue, &message, K_NO_WAIT) != 0) {
		k_mem_slab_free(&relay_message_slab, (void *)message);
		key = k_spin_lock(&tester.lock);
		tester.relay_dropped++;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR relay queue overflow; capture invalid");
		return false;
	}

	return true;
}

static bool parse_start_ack(const uint8_t *payload, struct stream_metadata *metadata)
{
	uint8_t name_length = payload[START_ACK_NAME_LENGTH_OFFSET];

	if (name_length == 0U || name_length > 16U ||
	    payload[START_ACK_TREE_STATE_OFFSET] > 2U ||
	    !all_zero(&payload[START_ACK_RESERVED_OFFSET], 6U) ||
	    !all_zero(&payload[START_ACK_NAME_OFFSET + name_length], 16U - name_length) ||
	    !is_lower_hex_ascii(&payload[START_ACK_GIT_COMMIT_OFFSET], 40U)) {
		return false;
	}

	memset(metadata, 0, sizeof(*metadata));
	metadata->device_type = payload[START_ACK_DEVICE_TYPE_OFFSET];
	metadata->record_format_version = payload[START_ACK_FORMAT_VERSION_OFFSET];
	metadata->record_size = sys_get_le16(&payload[START_ACK_RECORD_SIZE_OFFSET]);
	metadata->rate_numerator = sys_get_le32(&payload[START_ACK_RATE_NUMERATOR_OFFSET]);
	metadata->rate_denominator = sys_get_le32(&payload[START_ACK_RATE_DENOMINATOR_OFFSET]);
	metadata->history_records = sys_get_le32(&payload[START_ACK_HISTORY_COUNT_OFFSET]);
	metadata->forward_records = sys_get_le32(&payload[START_ACK_FORWARD_COUNT_OFFSET]);
	metadata->total_sensor_bytes = sys_get_le32(&payload[START_ACK_TOTAL_BYTES_OFFSET]);
	memcpy(metadata->device_id, &payload[START_ACK_DEVICE_ID_OFFSET], sizeof(metadata->device_id));
	memcpy(metadata->device_name, &payload[START_ACK_NAME_OFFSET], name_length);
	memcpy(metadata->git_commit, &payload[START_ACK_GIT_COMMIT_OFFSET], 40U);
	metadata->git_tree_state = payload[START_ACK_TREE_STATE_OFFSET];

	return geometry_is_valid(metadata);
}

static bool notification_for_current_connection(struct bt_conn *conn)
{
	bool current;
	k_spinlock_key_t key = k_spin_lock(&tester.lock);

	current = tester.conn == conn;
	k_spin_unlock(&tester.lock, key);
	return current;
}

static void handle_start_ack(uint32_t session_id, const uint8_t *payload, uint16_t length)
{
	struct stream_metadata metadata;
	char device_id[17];
	uint16_t mtu;
	k_spinlock_key_t key;

	if (length != MSENSE_SENSOR_STREAM_START_ACK_BYTES || !parse_start_ack(payload, &metadata)) {
		mark_protocol_failure("invalid START_ACK");
		return;
	}

	key = k_spin_lock(&tester.lock);
	if (tester.state != TESTER_START_PENDING || session_id != tester.metadata.session_id) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("unexpected START_ACK");
		return;
	}
	metadata.session_id = session_id;
	tester.metadata = metadata;
	tester.state = TESTER_RECEIVING;
	mtu = tester.att_mtu;
	k_spin_unlock(&tester.lock, key);

	device_id_to_hex(metadata.device_id, device_id);
	post_event("START_ACK type=%s name=%s id=%s commit=%s tree=%u records=%u+%u mtu=%u",
		   metadata.device_type == MSENSE_SENSOR_STREAM_DEVICE_PPG ? "PPG" : "ECG",
		   metadata.device_name, device_id, metadata.git_commit, metadata.git_tree_state,
		   metadata.history_records, metadata.forward_records, mtu);
}

static void handle_data(uint32_t session_id, const uint8_t *payload, uint16_t length,
			uint16_t raw_nus_length, int64_t received_ms)
{
	struct phase_statistics history_statistics;
	uint32_t sequence;
	uint32_t first_record;
	uint32_t expected_end;
	uint32_t sensor_bytes;
	uint16_t record_count;
	uint16_t expected_length;
	uint8_t phase;
	int64_t request_start_ms;
	bool request_started;
	bool report_history = false;
	k_spinlock_key_t key;

	if (length < MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES) {
		mark_protocol_failure("short DATA");
		return;
	}

	key = k_spin_lock(&tester.lock);
	if (tester.state != TESTER_RECEIVING || session_id != tester.metadata.session_id) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("unexpected DATA");
		return;
	}

	sequence = sys_get_le32(&payload[DATA_SEQUENCE_OFFSET]);
	first_record = sys_get_le32(&payload[DATA_FIRST_RECORD_OFFSET]);
	record_count = sys_get_le16(&payload[DATA_RECORD_COUNT_OFFSET]);
	phase = payload[DATA_PHASE_OFFSET];
	if (record_count == 0U || payload[DATA_RESERVED_OFFSET] != 0U ||
	    phase > DATA_PHASE_FORWARD ||
	    record_count > (UINT16_MAX - MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES) /
			   tester.metadata.record_size) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("invalid DATA prefix");
		return;
	}

	expected_length = MSENSE_SENSOR_STREAM_DATA_PREFIX_BYTES +
			  record_count * tester.metadata.record_size;
	expected_end = first_record + record_count;
	if (expected_length != length || sequence != tester.metadata.expected_sequence ||
	    first_record != tester.metadata.expected_record_index || expected_end < first_record ||
	    expected_end > tester.metadata.history_records + tester.metadata.forward_records ||
	    (first_record < tester.metadata.history_records &&
	     (phase != DATA_PHASE_HISTORY || expected_end > tester.metadata.history_records)) ||
	    (first_record >= tester.metadata.history_records && phase != DATA_PHASE_FORWARD)) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("DATA sequence/index/phase");
		return;
	}

	tester.metadata.expected_sequence++;
	tester.metadata.expected_record_index = expected_end;
	sensor_bytes = (uint32_t)record_count * tester.metadata.record_size;
	tester.metadata.received_sensor_bytes += sensor_bytes;
	tester.metadata.received_data_messages++;
	if (phase == DATA_PHASE_FORWARD && !tester.statistics.history_reported) {
		tester.statistics.history_reported = true;
		history_statistics = tester.statistics.history;
		request_start_ms = tester.statistics.request_start_ms;
		request_started = tester.statistics.request_started;
		report_history = true;
	}
	stream_statistics_record_data(&tester.statistics, phase, raw_nus_length, sensor_bytes,
				      received_ms);
	k_spin_unlock(&tester.lock, key);

	if (report_history) {
		post_throughput_history(session_id, &history_statistics, request_start_ms,
					request_started);
	}
}

static void handle_result(uint32_t session_id, const uint8_t *payload, uint16_t length)
{
	uint16_t status;
	uint8_t peripheral_state;
	uint8_t opcode;
	bool rejected_start = false;
	k_spinlock_key_t key;

	if (length != MSENSE_SENSOR_STREAM_RESULT_BYTES || payload[RESULT_RESERVED_OFFSET] != 0U) {
		mark_protocol_failure("invalid RESULT");
		return;
	}

	status = sys_get_le16(&payload[RESULT_STATUS_OFFSET]);
	peripheral_state = payload[RESULT_STATE_OFFSET];
	if (status > MSENSE_SENSOR_STREAM_STATUS_DISCONNECTED) {
		mark_protocol_failure("invalid RESULT status");
		return;
	}
	key = k_spin_lock(&tester.lock);
	opcode = tester.last_command_opcode;
	if (session_id != tester.last_command_session_id) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("unexpected RESULT session");
		return;
	}
	if (peripheral_state > MSENSE_SENSOR_STREAM_STATE_UNINITIALIZED) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("invalid RESULT state");
		return;
	}
	if (opcode == MSENSE_SENSOR_STREAM_OPCODE_START && tester.state == TESTER_START_PENDING &&
	    status != MSENSE_SENSOR_STREAM_STATUS_SUCCESS) {
		tester.state = TESTER_READY;
		rejected_start = true;
	} else if (opcode != MSENSE_SENSOR_STREAM_OPCODE_CANCEL ||
		   tester.state != TESTER_RECEIVING || status == MSENSE_SENSOR_STREAM_STATUS_SUCCESS) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("unexpected RESULT");
		return;
	}
	k_spin_unlock(&tester.lock, key);

	post_event(rejected_start ? "START_RESULT status=%s(0x%04x) peripheral_state=%u" :
		   "CANCEL_RESULT status=%s(0x%04x) peripheral_state=%u", status_name(status),
		   status, peripheral_state);
}

static void handle_end(uint32_t session_id, const uint8_t *payload, uint16_t length)
{
	struct stream_statistics statistics;
	struct phase_statistics history_statistics;
	uint16_t status;
	uint8_t peripheral_state;
	uint32_t history_sent;
	uint32_t forward_captured;
	uint32_t sensor_bytes;
	uint32_t data_messages;
	int32_t detail;
	int64_t request_start_ms;
	bool success;
	bool request_started;
	bool report_history = false;
	k_spinlock_key_t key;

	if (length != MSENSE_SENSOR_STREAM_END_BYTES || payload[END_RESERVED_OFFSET] != 0U) {
		mark_protocol_failure("invalid END");
		return;
	}

	status = sys_get_le16(&payload[END_STATUS_OFFSET]);
	peripheral_state = payload[END_STATE_OFFSET];
	if (peripheral_state > MSENSE_SENSOR_STREAM_STATE_UNINITIALIZED) {
		mark_protocol_failure("invalid END state");
		return;
	}
	history_sent = sys_get_le32(&payload[END_HISTORY_SENT_OFFSET]);
	forward_captured = sys_get_le32(&payload[END_FORWARD_CAPTURED_OFFSET]);
	sensor_bytes = sys_get_le32(&payload[END_SENSOR_BYTES_OFFSET]);
	data_messages = sys_get_le32(&payload[END_DATA_MESSAGES_OFFSET]);
	detail = (int32_t)sys_get_le32(&payload[END_DETAIL_OFFSET]);

	key = k_spin_lock(&tester.lock);
	if (tester.state != TESTER_RECEIVING || session_id != tester.metadata.session_id) {
		k_spin_unlock(&tester.lock, key);
		mark_protocol_failure("unexpected END");
		return;
	}

	success = status == MSENSE_SENSOR_STREAM_STATUS_SUCCESS && detail == 0 &&
		  history_sent == tester.metadata.history_records &&
		  forward_captured == tester.metadata.forward_records &&
		  sensor_bytes == tester.metadata.total_sensor_bytes &&
		  sensor_bytes == tester.metadata.received_sensor_bytes &&
		  data_messages == tester.metadata.received_data_messages &&
		  tester.metadata.expected_record_index ==
			  tester.metadata.history_records + tester.metadata.forward_records;
	tester.state = success ? TESTER_COMPLETE : TESTER_FAILED;
	if (!tester.statistics.history_reported && tester.statistics.history.has_data) {
		tester.statistics.history_reported = true;
		history_statistics = tester.statistics.history;
		request_start_ms = tester.statistics.request_start_ms;
		request_started = tester.statistics.request_started;
		report_history = true;
	}
	statistics = tester.statistics;
	k_spin_unlock(&tester.lock, key);

	if (report_history) {
		post_throughput_history(session_id, &history_statistics, request_start_ms,
					request_started);
	}
	if (success) {
		post_throughput_forward(session_id, &statistics.forward);
		post_throughput_summary(session_id, &statistics.total);
		post_event("STREAM_OK id=%u bytes=%u data_messages=%u", session_id, sensor_bytes,
			   data_messages);
	} else {
		post_event("STREAM_END status=%s(0x%04x) peripheral_state=%u bytes=%u data=%u "
			   "detail=%d", status_name(status), status, peripheral_state, sensor_bytes,
			   data_messages, detail);
	}
}

static void handle_notification(const uint8_t *data, uint16_t length, int64_t received_ms)
{
	uint16_t payload_length;
	uint16_t flags;
	uint32_t session_id;
	uint8_t message_type;

	if (length < MSENSE_SENSOR_STREAM_HEADER_BYTES ||
	    data[0] != MSENSE_SENSOR_STREAM_MAGIC0 || data[1] != MSENSE_SENSOR_STREAM_MAGIC1 ||
	    data[2] != MSENSE_SENSOR_STREAM_PROTOCOL_VERSION) {
		mark_protocol_failure("invalid common header");
		return;
	}

	payload_length = sys_get_le16(&data[NUS_HEADER_PAYLOAD_LENGTH_OFFSET]);
	flags = sys_get_le16(&data[NUS_HEADER_FLAGS_OFFSET]);
	if (flags != 0U || length != MSENSE_SENSOR_STREAM_HEADER_BYTES + payload_length) {
		mark_protocol_failure("invalid header length/flags");
		return;
	}

	message_type = data[NUS_HEADER_MESSAGE_TYPE_OFFSET];
	session_id = sys_get_le32(&data[NUS_HEADER_SESSION_ID_OFFSET]);
	switch (message_type) {
	case MSENSE_SENSOR_STREAM_MESSAGE_START_ACK:
		handle_start_ack(session_id, &data[NUS_PAYLOAD_OFFSET], payload_length);
		break;
	case MSENSE_SENSOR_STREAM_MESSAGE_DATA:
		handle_data(session_id, &data[NUS_PAYLOAD_OFFSET], payload_length, length,
			    received_ms);
		break;
	case MSENSE_SENSOR_STREAM_MESSAGE_END:
		handle_end(session_id, &data[NUS_PAYLOAD_OFFSET], payload_length);
		break;
	case MSENSE_SENSOR_STREAM_MESSAGE_RESULT:
		handle_result(session_id, &data[NUS_PAYLOAD_OFFSET], payload_length);
		break;
	default:
		mark_protocol_failure("unsupported message type");
		break;
	}
}

static uint8_t nus_notification(struct bt_conn *conn,
				struct bt_gatt_subscribe_params *params,
				const void *data, uint16_t length)
{
	int64_t received_ms = k_uptime_get();

	ARG_UNUSED(params);
	if (data == NULL) {
		mark_protocol_failure("NUS subscription removed");
		post_event("ERROR NUS subscription removed");
		return BT_GATT_ITER_STOP;
	}
	if (!notification_for_current_connection(conn)) {
		return BT_GATT_ITER_CONTINUE;
	}
	/* Parse first so relay backpressure cannot hide a valid RESULT or END. */
	handle_notification(data, length, received_ms);
	if (!relay_enqueue(data, length)) {
		mark_protocol_failure("relay delivery failed");
	}

	return BT_GATT_ITER_CONTINUE;
}

static void subscribe_complete(struct bt_conn *conn, uint8_t err,
			       struct bt_gatt_subscribe_params *params)
{
	uint16_t mtu;
	k_spinlock_key_t key;

	ARG_UNUSED(params);
	if (!notification_for_current_connection(conn)) {
		return;
	}
	if (err != 0U) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR NUS subscription failed: ATT 0x%02x", err);
		return;
	}

	mtu = bt_gatt_get_mtu(conn);
	key = k_spin_lock(&tester.lock);
	tester.att_mtu = mtu;
	if (mtu >= REQUIRED_ATT_MTU) {
		tester.subscribed = true;
		tester.state = TESTER_READY;
	} else {
		tester.state = TESTER_FAILED;
	}
	k_spin_unlock(&tester.lock, key);

	if (mtu < REQUIRED_ATT_MTU) {
		post_event("ERROR negotiated ATT MTU %u is below %u", mtu, REQUIRED_ATT_MTU);
		return;
	}
	post_event("NUS_READY mtu=%u tx=0x%04x rx=0x%04x cccd=0x%04x", mtu,
		   nus_client.handles.tx, nus_client.handles.rx, nus_client.handles.tx_ccc);
}

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	int err;
	k_spinlock_key_t key;

	ARG_UNUSED(context);
	err = bt_nus_handles_assign(dm, &nus_client);
	if (err != 0) {
		(void)bt_gatt_dm_data_release(dm);
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR NUS handle discovery failed: %d", err);
		return;
	}

	memset(&subscribe_params, 0, sizeof(subscribe_params));
	subscribe_params.notify = nus_notification;
	subscribe_params.subscribe = subscribe_complete;
	subscribe_params.value = BT_GATT_CCC_NOTIFY;
	subscribe_params.value_handle = nus_client.handles.tx;
	subscribe_params.ccc_handle = nus_client.handles.tx_ccc;
	atomic_set_bit(subscribe_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	key = k_spin_lock(&tester.lock);
	tester.state = TESTER_SUBSCRIBING;
	k_spin_unlock(&tester.lock, key);
	err = bt_gatt_subscribe(nus_client.conn, &subscribe_params);
	(void)bt_gatt_dm_data_release(dm);
	if (err != 0) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR NUS subscribe request failed: %d", err);
	}
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	k_spinlock_key_t key;

	ARG_UNUSED(conn);
	ARG_UNUSED(context);
	key = k_spin_lock(&tester.lock);
	tester.state = TESTER_FAILED;
	k_spin_unlock(&tester.lock, key);
	post_event("ERROR standard NUS service not found");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	k_spinlock_key_t key;

	ARG_UNUSED(conn);
	ARG_UNUSED(context);
	key = k_spin_lock(&tester.lock);
	tester.state = TESTER_FAILED;
	k_spin_unlock(&tester.lock, key);
	post_event("ERROR NUS discovery failed: %d", err);
}

static const struct bt_gatt_dm_cb discovery_callbacks = {
	.completed = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found = discovery_error,
};

static void start_nus_discovery(struct bt_conn *conn)
{
	int err;
	k_spinlock_key_t key;

	key = k_spin_lock(&tester.lock);
	tester.state = TESTER_DISCOVERING;
	k_spin_unlock(&tester.lock, key);
	err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_callbacks, NULL);
	if (err != 0) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR NUS discovery could not start: %d", err);
	}
}

static void mtu_exchange_complete(struct bt_conn *conn, uint8_t err,
				  struct bt_gatt_exchange_params *params)
{
	uint16_t mtu;
	k_spinlock_key_t key;

	ARG_UNUSED(params);
	if (!notification_for_current_connection(conn)) {
		return;
	}
	if (err != 0U) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR ATT MTU exchange failed: 0x%02x", err);
		return;
	}

	mtu = bt_gatt_get_mtu(conn);
	key = k_spin_lock(&tester.lock);
	tester.att_mtu = mtu;
	k_spin_unlock(&tester.lock, key);
	if (mtu < REQUIRED_ATT_MTU) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR negotiated ATT MTU %u is below %u", mtu, REQUIRED_ATT_MTU);
		return;
	}

	post_event("ATT_MTU %u", mtu);
	update_link_info_from_connection(conn);
	post_ble_link(conn);
	start_nus_discovery(conn);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char address[BT_ADDR_LE_STR_LEN];
	int ret;
	k_spinlock_key_t key;

	if (!notification_for_current_connection(conn)) {
		return;
	}
	bt_addr_le_to_str(bt_conn_get_dst(conn), address, sizeof(address));
	if (err != 0U) {
		struct bt_conn *held_conn;

		key = k_spin_lock(&tester.lock);
		held_conn = tester.conn;
		tester.conn = NULL;
		memset(&tester.link, 0, sizeof(tester.link));
		tester.att_mtu = 0U;
		tester.state = TESTER_IDLE;
		k_spin_unlock(&tester.lock, key);
		if (held_conn != NULL) {
			bt_conn_unref(held_conn);
		}
		post_event("ERROR connect %s failed: 0x%02x", address, err);
		return;
	}

	key = k_spin_lock(&tester.lock);
	tester.state = TESTER_MTU_EXCHANGE;
	k_spin_unlock(&tester.lock, key);
	post_event("CONNECTED %s", address);
	update_link_info_from_connection(conn);
	post_ble_link(conn);

	memset(&exchange_params, 0, sizeof(exchange_params));
	exchange_params.func = mtu_exchange_complete;
	ret = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (ret != 0) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_FAILED;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR ATT MTU exchange could not start: %d", ret);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *held_conn = NULL;
	bool stream_was_active;
	k_spinlock_key_t key;

	key = k_spin_lock(&tester.lock);
	if (tester.conn == conn) {
		stream_was_active = tester.state == TESTER_START_PENDING ||
				    tester.state == TESTER_RECEIVING;
		held_conn = tester.conn;
		tester.conn = NULL;
		tester.subscribed = false;
		tester.write_pending = false;
		tester.att_mtu = 0U;
		memset(&tester.link, 0, sizeof(tester.link));
		tester.state = TESTER_IDLE;
		nus_client.conn = NULL;
	}
	k_spin_unlock(&tester.lock, key);

	if (held_conn != NULL) {
		bt_conn_unref(held_conn);
		post_event("DISCONNECTED reason=0x%02x%s", reason,
			   stream_was_active ? " stream_aborted" : "");
	}
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout)
{
	k_spinlock_key_t key;

	if (!notification_for_current_connection(conn)) {
		return;
	}

	key = k_spin_lock(&tester.lock);
	tester.link.interval_units = interval;
	tester.link.interval_ms_x100 = (uint32_t)interval * 125U;
	tester.link.latency = latency;
	tester.link.timeout_ms = (uint32_t)timeout * 10U;
	k_spin_unlock(&tester.lock, key);
	post_ble_link(conn);
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *info)
{
	k_spinlock_key_t key;

	if (!notification_for_current_connection(conn)) {
		return;
	}

	key = k_spin_lock(&tester.lock);
	tester.link.tx_phy = info->tx_phy;
	tester.link.rx_phy = info->rx_phy;
	k_spin_unlock(&tester.lock, key);
	post_ble_link(conn);
}
#endif

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	k_spinlock_key_t key;

	if (!notification_for_current_connection(conn)) {
		return;
	}

	key = k_spin_lock(&tester.lock);
	tester.link.tx_max_len = info->tx_max_len;
	tester.link.tx_max_time = info->tx_max_time;
	tester.link.rx_max_len = info->rx_max_len;
	tester.link.rx_max_time = info->rx_max_time;
	k_spin_unlock(&tester.lock, key);
	post_ble_link(conn);
}
#endif

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
	.le_phy_updated = le_phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	.le_data_len_updated = le_data_len_updated,
#endif
};

struct advertisement_name {
	char value[17];
	bool present;
};

static bool advertising_data_parse(struct bt_data *data, void *user_data)
{
	struct advertisement_name *name = user_data;
	uint8_t length;

	if (data->type != BT_DATA_NAME_COMPLETE && data->type != BT_DATA_NAME_SHORTENED) {
		return true;
	}

	length = MIN(data->data_len, sizeof(name->value) - 1U);
	memcpy(name->value, data->data, length);
	name->value[length] = '\0';
	name->present = true;
	return false;
}

static bool name_matches_target(const char *name, enum scan_target target)
{
	if (target == SCAN_TARGET_PPG) {
		return strncmp(name, "MSense4PPG-", 11U) == 0;
	}
	if (target == SCAN_TARGET_ECG) {
		return strncmp(name, "MSense4ECG-", 11U) == 0;
	}
	return target == SCAN_TARGET_ANY &&
	       (strncmp(name, "MSense4PPG-", 11U) == 0 ||
		strncmp(name, "MSense4ECG-", 11U) == 0);
}

static bool name_is_msense(const char *name)
{
	return strncmp(name, "MSense4PPG-", 11U) == 0 ||
	       strncmp(name, "MSense4ECG-", 11U) == 0;
}

static void device_found(const bt_addr_le_t *address, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct advertisement_name name;
	enum scan_target target;
	char address_text[BT_ADDR_LE_STR_LEN];
	struct bt_conn *connection = NULL;
	int err;
	k_spinlock_key_t key;

	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
	    type != BT_GAP_ADV_TYPE_SCAN_RSP) {
		return;
	}

	memset(&name, 0, sizeof(name));
	bt_data_parse(ad, advertising_data_parse, &name);
	if (!name.present || !name_is_msense(name.value)) {
		return;
	}

	key = k_spin_lock(&tester.lock);
	if (tester.state != TESTER_SCANNING) {
		k_spin_unlock(&tester.lock, key);
		return;
	}
	target = tester.scan_target;
	if (target == SCAN_TARGET_LIST) {
		tester.state = TESTER_IDLE;
	} else if (name_matches_target(name.value, target)) {
		tester.state = TESTER_CONNECTING;
	} else {
		k_spin_unlock(&tester.lock, key);
		return;
	}
	k_spin_unlock(&tester.lock, key);

	bt_addr_le_to_str(address, address_text, sizeof(address_text));
	if (bt_le_scan_stop() != 0) {
		post_event("ERROR could not stop scan for %s", name.value);
		return;
	}
	if (target == SCAN_TARGET_LIST) {
		post_event("FOUND name=%s address=%s rssi=%d", name.value, address_text, rssi);
		return;
	}

	err = bt_conn_le_create(address, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				&connection);
	if (err != 0) {
		key = k_spin_lock(&tester.lock);
		tester.state = TESTER_IDLE;
		k_spin_unlock(&tester.lock, key);
		post_event("ERROR connect create for %s failed: %d", name.value, err);
		return;
	}

	key = k_spin_lock(&tester.lock);
	tester.conn = connection;
	tester.att_mtu = 0U;
	memset(&tester.link, 0, sizeof(tester.link));
	k_spin_unlock(&tester.lock, key);
	post_event("CONNECTING name=%s address=%s", name.value, address_text);
}

static void start_scan(void)
{
	const struct bt_le_scan_param parameters = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	err = bt_le_scan_start(&parameters, device_found);
	if (err != 0) {
		k_spinlock_key_t key = k_spin_lock(&tester.lock);

		tester.state = TESTER_IDLE;
		k_spin_unlock(&tester.lock, key);
		command_printf("ERR scan start %d", err);
		return;
	}
	command_printf("OK scanning");
}

static bool parse_u32(const char *text, uint32_t *value)
{
	uint32_t result = 0U;
	uint32_t base = 10U;
	uint8_t digit;

	if (text == NULL || *text == '\0') {
		return false;
	}
	if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
		base = 16U;
		text += 2;
		if (*text == '\0') {
			return false;
		}
	}

	while (*text != '\0') {
		if (*text >= '0' && *text <= '9') {
			digit = *text - '0';
		} else if (*text >= 'a' && *text <= 'f') {
			digit = *text - 'a' + 10U;
		} else if (*text >= 'A' && *text <= 'F') {
			digit = *text - 'A' + 10U;
		} else {
			return false;
		}
		if (digit >= base || result > (UINT32_MAX - digit) / base) {
			return false;
		}
		result = result * base + digit;
		text++;
	}

	*value = result;
	return true;
}

static char *next_token(char **text)
{
	char *start;

	while (**text == ' ' || **text == '\t') {
		(*text)++;
	}
	if (**text == '\0') {
		return NULL;
	}

	start = *text;
	while (**text != '\0' && **text != ' ' && **text != '\t') {
		(*text)++;
	}
	if (**text != '\0') {
		**text = '\0';
		(*text)++;
	}

	return start;
}

static struct bt_conn *connection_ref(void)
{
	struct bt_conn *connection = NULL;
	k_spinlock_key_t key = k_spin_lock(&tester.lock);

	if (tester.conn != NULL) {
		connection = bt_conn_ref(tester.conn);
	}
	k_spin_unlock(&tester.lock, key);
	return connection;
}

static int issue_nus_command(uint8_t opcode, uint32_t session_id)
{
	struct bt_conn *connection;
	int err;
	k_spinlock_key_t key;

	connection = connection_ref();
	if (connection == NULL) {
		return -ENOTCONN;
	}

	key = k_spin_lock(&tester.lock);
	if (tester.write_pending || !tester.subscribed || nus_client.handles.rx == 0U) {
		k_spin_unlock(&tester.lock, key);
		bt_conn_unref(connection);
		return -EBUSY;
	}
	tester.write_pending = true;
	tester.last_command_opcode = opcode;
	tester.last_command_session_id = session_id;
	if (opcode == MSENSE_SENSOR_STREAM_OPCODE_START) {
		tester.statistics.request_start_ms = k_uptime_get();
		tester.statistics.request_started = true;
	}
	k_spin_unlock(&tester.lock, key);

	command_write_data[0] = MSENSE_SENSOR_STREAM_MAGIC0;
	command_write_data[1] = MSENSE_SENSOR_STREAM_MAGIC1;
	command_write_data[2] = MSENSE_SENSOR_STREAM_PROTOCOL_VERSION;
	command_write_data[3] = opcode;
	sys_put_le32(session_id, &command_write_data[4]);
	memset(&command_write_params, 0, sizeof(command_write_params));
	command_write_params.func = command_write_complete;
	command_write_params.handle = nus_client.handles.rx;
	command_write_params.offset = 0U;
	command_write_params.data = command_write_data;
	command_write_params.length = sizeof(command_write_data);
	err = bt_gatt_write(connection, &command_write_params);
	bt_conn_unref(connection);
	if (err != 0) {
		key = k_spin_lock(&tester.lock);
		tester.write_pending = false;
		k_spin_unlock(&tester.lock, key);
	}

	return err;
}

static void command_write_complete(struct bt_conn *conn, uint8_t err,
				   struct bt_gatt_write_params *params)
{
	k_spinlock_key_t key;

	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	key = k_spin_lock(&tester.lock);
	tester.write_pending = false;
	k_spin_unlock(&tester.lock, key);
	if (err != 0U) {
		post_event("ERROR NUS command write ATT=0x%02x", err);
	}
}

static void begin_start(uint32_t requested_session_id, bool has_requested_id)
{
	uint32_t session_id;
	int err;
	k_spinlock_key_t key;

	key = k_spin_lock(&tester.lock);
	if ((tester.state != TESTER_READY && tester.state != TESTER_COMPLETE) ||
	    !tester.subscribed || tester.write_pending) {
		enum tester_state state = tester.state;

		k_spin_unlock(&tester.lock, key);
		command_printf("ERR start requires READY or COMPLETE (state=%s)",
			       tester_state_name(state));
		return;
	}
	session_id = has_requested_id ? requested_session_id : tester.next_session_id++;
	if (session_id == 0U) {
		k_spin_unlock(&tester.lock, key);
		command_printf("ERR session ID must be nonzero");
		return;
	}
	memset(&tester.metadata, 0, sizeof(tester.metadata));
	memset(&tester.statistics, 0, sizeof(tester.statistics));
	tester.metadata.session_id = session_id;
	tester.state = TESTER_START_PENDING;
	k_spin_unlock(&tester.lock, key);

	err = issue_nus_command(MSENSE_SENSOR_STREAM_OPCODE_START, session_id);
	if (err != 0) {
		key = k_spin_lock(&tester.lock);
		if (tester.state == TESTER_START_PENDING) {
			tester.state = TESTER_READY;
		}
		k_spin_unlock(&tester.lock, key);
		command_printf("ERR START write %d", err);
		return;
	}
	command_printf("START_SENT id=%u", session_id);
}

static void begin_cancel(uint32_t requested_session_id, bool has_requested_id)
{
	uint32_t session_id;
	int err;
	k_spinlock_key_t key;

	key = k_spin_lock(&tester.lock);
	if (tester.state != TESTER_RECEIVING || tester.write_pending) {
		enum tester_state state = tester.state;

		k_spin_unlock(&tester.lock, key);
		command_printf("ERR cancel requires RECEIVING (state=%s)", tester_state_name(state));
		return;
	}
	session_id = has_requested_id ? requested_session_id : tester.metadata.session_id;
	k_spin_unlock(&tester.lock, key);

	err = issue_nus_command(MSENSE_SENSOR_STREAM_OPCODE_CANCEL, session_id);
	if (err != 0) {
		command_printf("ERR CANCEL write %d", err);
		return;
	}
	command_printf("CANCEL_SENT id=%u", session_id);
}

static void print_status(void)
{
	struct stream_metadata metadata;
	struct stream_statistics statistics;
	struct ble_link_info link;
	enum tester_state state;
	int64_t throughput_end_ms;
	uint16_t mtu;
	uint32_t relay_dropped;
	bool subscribed;
	k_spinlock_key_t key = k_spin_lock(&tester.lock);

	state = tester.state;
	mtu = tester.att_mtu;
	subscribed = tester.subscribed;
	relay_dropped = tester.relay_dropped;
	metadata = tester.metadata;
	statistics = tester.statistics;
	link = tester.link;
	k_spin_unlock(&tester.lock, key);

	command_printf("STATUS state=%s subscribed=%u mtu=%u id=%u bytes=%u data=%u "
		       "record_index=%u relay_dropped=%u", tester_state_name(state), subscribed, mtu,
		       metadata.session_id, metadata.received_sensor_bytes,
		       metadata.received_data_messages, metadata.expected_record_index, relay_dropped);
	throughput_end_ms =
		state == TESTER_RECEIVING ? k_uptime_get() : statistics.total.last_data_ms;
	print_live_throughput(metadata.session_id, &statistics.total, throughput_end_ms);
	print_ble_link(&link, mtu);
}

static void show_help(void)
{
	command_printf("COMMANDS: help | scan | connect ppg|ecg|any | status | start [id] | "
		       "cancel [id] | disconnect");
}

static void handle_command(struct command_line *line)
{
	char *cursor = line->text;
	char *command = next_token(&cursor);
	char *argument = next_token(&cursor);
	char *extra = next_token(&cursor);
	uint32_t value;
	k_spinlock_key_t key;

	if (command == NULL) {
		return;
	}
	if (strcmp(command, "help") == 0 && argument == NULL) {
		show_help();
		return;
	}
	if (strcmp(command, "status") == 0 && argument == NULL) {
		print_status();
		return;
	}
	if (strcmp(command, "scan") == 0 && argument == NULL) {
		key = k_spin_lock(&tester.lock);
		if (tester.conn != NULL || tester.state == TESTER_SCANNING ||
		    tester.state == TESTER_CONNECTING) {
			k_spin_unlock(&tester.lock, key);
			command_printf("ERR scan requires no active connection");
			return;
		}
		tester.scan_target = SCAN_TARGET_LIST;
		tester.state = TESTER_SCANNING;
		k_spin_unlock(&tester.lock, key);
		start_scan();
		return;
	}
	if (strcmp(command, "connect") == 0 && extra == NULL) {
		enum scan_target target;

		if (argument == NULL) {
			command_printf("ERR usage: connect ppg|ecg|any");
			return;
		}
		if (strcmp(argument, "ppg") == 0) {
			target = SCAN_TARGET_PPG;
		} else if (strcmp(argument, "ecg") == 0) {
			target = SCAN_TARGET_ECG;
		} else if (strcmp(argument, "any") == 0) {
			target = SCAN_TARGET_ANY;
		} else {
			command_printf("ERR usage: connect ppg|ecg|any");
			return;
		}

		key = k_spin_lock(&tester.lock);
		if (tester.conn != NULL || tester.state == TESTER_SCANNING ||
		    tester.state == TESTER_CONNECTING) {
			k_spin_unlock(&tester.lock, key);
			command_printf("ERR connect requires no active connection");
			return;
		}
		tester.scan_target = target;
		tester.state = TESTER_SCANNING;
		k_spin_unlock(&tester.lock, key);
		start_scan();
		return;
	}
	if (strcmp(command, "start") == 0 && extra == NULL) {
		if (argument != NULL && (!parse_u32(argument, &value) || value == 0U)) {
			command_printf("ERR start ID must be a nonzero decimal or 0x hexadecimal uint32");
			return;
		}
		begin_start(value, argument != NULL);
		return;
	}
	if (strcmp(command, "cancel") == 0 && extra == NULL) {
		if (argument != NULL && (!parse_u32(argument, &value) || value == 0U)) {
			command_printf("ERR cancel ID must be a nonzero decimal or 0x hexadecimal uint32");
			return;
		}
		begin_cancel(value, argument != NULL);
		return;
	}
	if (strcmp(command, "disconnect") == 0 && argument == NULL) {
		struct bt_conn *connection = connection_ref();
		int err;

		if (connection == NULL) {
			command_printf("ERR no active connection");
			return;
		}
		err = bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		bt_conn_unref(connection);
		if (err != 0) {
			command_printf("ERR disconnect %d", err);
		} else {
			command_printf("DISCONNECT_SENT");
		}
		return;
	}

	command_printf("ERR unknown command; use help");
}

static void command_uart_callback(const struct device *device, void *user_data)
{
	uint8_t byte;

	ARG_UNUSED(user_data);
	if (!uart_irq_update(device) || !uart_irq_rx_ready(device)) {
		return;
	}

	while (uart_fifo_read(device, &byte, 1U) == 1) {
		if (byte == '\r' || byte == '\n') {
			struct command_line line;

			if (command_rx_length == 0U) {
				continue;
			}
			printk("UART_COMMAND_RX length=%u\n", (uint32_t)command_rx_length);
			memset(&line, 0, sizeof(line));
			memcpy(line.text, command_rx_buffer, command_rx_length);
			command_rx_length = 0U;
			(void)k_msgq_put(&command_queue, &line, K_NO_WAIT);
			continue;
		}
		if (byte >= 0x20U && byte <= 0x7eU) {
			if (command_rx_length < sizeof(command_rx_buffer) - 1U) {
				command_rx_buffer[command_rx_length++] = (char)byte;
			} else {
				command_rx_length = 0U;
			}
		}
	}
}

static int uart_initialize(void)
{
	int err;

	if (!device_is_ready(command_uart) || !device_is_ready(relay_uart)) {
		printk("UART_INIT_ERROR VCOM UARTE not ready\n");
		return -ENODEV;
	}
	err = uart_irq_callback_user_data_set(command_uart, command_uart_callback, NULL);
	if (err != 0) {
		printk("UART_INIT_ERROR command callback=%d\n", err);
		return err;
	}
	err = uart_callback_set(relay_uart, relay_uart_callback, NULL);
	if (err != 0) {
		printk("UART_INIT_ERROR relay async callback=%d\n", err);
		return err;
	}
	uart_irq_rx_enable(command_uart);
	printk("UART_INIT_OK\n");

	return 0;
}

int main(void)
{
	struct command_line command;
	struct control_event event;
	struct bt_nus_client_init_param nus_init = { 0 };
	int err;

	err = uart_initialize();
	if (err != 0) {
		printk("FATAL UART initialization %d\n", err);
		while (true) {
			k_sleep(K_FOREVER);
		}
	}
	err = bt_nus_client_init(&nus_client, &nus_init);
	if (err != 0) {
		printk("FATAL NUS client init %d\n", err);
		command_printf("FATAL NUS client init %d", err);
		while (true) {
			k_sleep(K_FOREVER);
		}
	}
	err = bt_enable(NULL);
	if (err != 0) {
		printk("FATAL Bluetooth init %d\n", err);
		command_printf("FATAL Bluetooth init %d", err);
		while (true) {
			k_sleep(K_FOREVER);
		}
	}

	command_printf("MSENSE_NUS_CENTRAL_READY protocol=1; use help");
	while (true) {
		while (k_msgq_get(&control_event_queue, &event, K_NO_WAIT) == 0) {
			command_printf("%s", event.text);
		}
		if (k_msgq_get(&command_queue, &command, K_MSEC(100)) == 0) {
			handle_command(&command);
		}
	}

	return 0;
}
