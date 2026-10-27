/*
 * Copyright (c) 2015-2016 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(bridge_central, LOG_LEVEL_INF);

#define TARGET_NAME "MSense4ECG-0ATEJ"
#define SCAN_SECONDS 10
#define RETRY_SECONDS 30

/* ECGv0/src/BLEService.h: legacy ENMO UUIDs, now timing-only notifications. */
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xda39c950, 0x1d81, 0x48e2, 0x9c68, 0xd0ae4bbd351f));
static struct bt_uuid_128 update_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xda39c951, 0x1d81, 0x48e2, 0x9c68, 0xd0ae4bbd351f));

static const struct device *const leds = DEVICE_DT_GET(DT_NODELABEL(leds));
static const struct device *const led_supply = DEVICE_DT_GET(DT_NODELABEL(led_supply));
static const struct device *const eeg_uart = DEVICE_DT_GET(DT_ALIAS(eeg_uart));
static const uint32_t led_order[] = {
	DT_NODE_CHILD_IDX(DT_ALIAS(led1)), /* Violet */
	DT_NODE_CHILD_IDX(DT_ALIAS(led0)), /* Pink */
	DT_NODE_CHILD_IDX(DT_ALIAS(led2)), /* Cyan */
};
static const char *const led_names[] = { "violet", "pink", "cyan" };

/* Main owns connection references and GATT requests. Callbacks signal completion;
 * notification data is copied before returning to the Bluetooth RX thread.
 */
K_SEM_DEFINE(found, 0, 1);
K_SEM_DEFINE(connection_done, 0, 1);
K_SEM_DEFINE(disconnection_done, 0, 1);
K_SEM_DEFINE(gatt_done, 0, 1);
K_MSGQ_DEFINE(packets, sizeof(uint32_t), 8, 4);
static atomic_t searching;
static atomic_t link_up;
static int64_t disconnected_at_ms;
static bt_addr_le_t peer;
static struct bt_gatt_discover_params discovery;
static struct bt_gatt_subscribe_params subscription;
static uint16_t found_handle;
static uint16_t service_end;
static uint8_t subscribe_error;

static bool match_name(struct bt_data *data, void *user_data)
{
	bool *match = user_data;

	if (data->type == BT_DATA_NAME_COMPLETE &&
	    data->data_len == sizeof(TARGET_NAME) - 1 &&
	    memcmp(data->data, TARGET_NAME, sizeof(TARGET_NAME) - 1) == 0) {
		*match = true;
		return false;
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	bool match = false;

	/* ECG puts its full name in the scan response, requiring active scanning. */
	if (!atomic_get(&searching) ||
	    (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_SCAN_RSP)) {
		return;
	}
	bt_data_parse(ad, match_name, &match);
	if (match && atomic_cas(&searching, 1, 0)) {
		bt_addr_le_copy(&peer, addr);
		LOG_INF("Found %s, RSSI %d dBm", TARGET_NAME, rssi);
		k_sem_give(&found);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	atomic_set(&link_up, err == 0);
	if (err) {
		LOG_WRN("Connection failed: 0x%02x", err);
	} else {
		LOG_INF("Connected to %s", TARGET_NAME);
	}
	k_sem_give(&connection_done);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	disconnected_at_ms = k_uptime_get();
	atomic_clear(&link_up);
	LOG_WRN("Disconnected: 0x%02x; retry in 30 seconds", reason);
	k_sem_give(&disconnection_done);
}

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static uint8_t discovered(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  struct bt_gatt_discover_params *params)
{
	if (attr != NULL) {
		found_handle = attr->handle;
		if (params->type == BT_GATT_DISCOVER_PRIMARY) {
			const struct bt_gatt_service_val *service = attr->user_data;

			service_end = service->end_handle;
		} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
			const struct bt_gatt_chrc *characteristic = attr->user_data;

			found_handle = (characteristic->properties & BT_GATT_CHRC_NOTIFY) ?
				characteristic->value_handle : 0;
		}
	}
	k_sem_give(&gatt_done);
	return BT_GATT_ITER_STOP;
}

static int discover(struct bt_conn *conn, const struct bt_uuid *uuid, uint8_t type,
		    uint16_t start, uint16_t end)
{
	int err;

	found_handle = 0;
	k_sem_reset(&gatt_done);
	discovery = (struct bt_gatt_discover_params) {
		.uuid = uuid, .func = discovered, .start_handle = start,
		.end_handle = end, .type = type,
	};
	err = bt_gatt_discover(conn, &discovery);
	if (err) {
		return err;
	}
	if (k_sem_take(&gatt_done, K_SECONDS(10))) {
		return -ETIMEDOUT;
	}
	return found_handle ? 0 : -ENOENT;
}

static uint8_t notified(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length)
{
	uint32_t ticks;

	if (data == NULL) {
		LOG_INF("Notification subscription removed");
		return BT_GATT_ITER_STOP;
	}
	if (length != 8) {
		LOG_WRN("Ignoring update with unexpected length %u", length);
		return BT_GATT_ITER_CONTINUE;
	}
	ticks = sys_get_le32((const uint8_t *)data + 4);
	if (k_msgq_put(&packets, &ticks, K_NO_WAIT)) {
		LOG_WRN("Notification queue full");
	}
	return BT_GATT_ITER_CONTINUE;
}

static void subscribed(struct bt_conn *conn, uint8_t err,
		       struct bt_gatt_subscribe_params *params)
{
	subscribe_error = err;
	k_sem_give(&gatt_done);
}

static int subscribe_updates(struct bt_conn *conn)
{
	int err;

	err = discover(conn, &service_uuid.uuid, BT_GATT_DISCOVER_PRIMARY, 1, UINT16_MAX);
	if (err) {
		return err;
	}
	err = discover(conn, &update_uuid.uuid, BT_GATT_DISCOVER_CHARACTERISTIC,
		       found_handle + 1, service_end);
	if (err) {
		return err;
	}
	memset(&subscription, 0, sizeof(subscription));
	subscription.value_handle = found_handle;
	/* This ECG service contains exactly one characteristic followed by its CCC. */
	err = discover(conn, BT_UUID_GATT_CCC, BT_GATT_DISCOVER_DESCRIPTOR,
		       found_handle + 1, service_end);
	if (err) {
		return err;
	}
	subscription.ccc_handle = found_handle;
	subscription.value = BT_GATT_CCC_NOTIFY;
	subscription.notify = notified;
	subscription.subscribe = subscribed;
	atomic_set_bit(subscription.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
	k_sem_reset(&gatt_done);
	err = bt_gatt_subscribe(conn, &subscription);
	if (err) {
		return err;
	}
	if (k_sem_take(&gatt_done, K_SECONDS(10))) {
		return -ETIMEDOUT;
	}
	return subscribe_error ? -EIO : 0;
}

static void send_rtc(uint32_t ticks)
{
	/* Ten decimal digits followed by one NUL byte. */
	char text[11];
	int length = snprintk(text, sizeof(text), "%u", (unsigned int)ticks);

	for (int i = 0; i < length; i++) {
		uart_poll_out(eeg_uart, text[i]);
	}
	uart_poll_out(eeg_uart, '\0');
}

int main(void)
{
	unsigned int next_led = 0;
	uint32_t packet_count = 0;
	int err;

	err = usb_enable(NULL);
	if (err) {
		LOG_ERR("USB initialization failed: %d", err);
		return 0;
	}
	if (!device_is_ready(leds) || !device_is_ready(led_supply)) {
		LOG_ERR("LEDs or 5 V supply not ready");
		return 0;
	}
	if (!device_is_ready(eeg_uart)) {
		LOG_ERR("EEG UART not ready");
		return 0;
	}
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth initialization failed: %d", err);
		return 0;
	}
	LOG_INF("Bridge central ready; target %s; 5 V enabled only for LED pulses", TARGET_NAME);

	for (;;) {
		struct bt_conn *conn = NULL;
		uint32_t ticks;
		int64_t scan_started = k_uptime_get();
		int64_t retry_at_ms = 0;

		k_sem_reset(&found);
		atomic_set(&searching, 1);
		LOG_INF("Scanning for %s for %d seconds", TARGET_NAME, SCAN_SECONDS);
		err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
		if (err) {
			atomic_clear(&searching);
			LOG_ERR("Scan start failed: %d", err);
			k_sleep(K_SECONDS(RETRY_SECONDS));
			continue;
		}
		err = k_sem_take(&found, K_SECONDS(SCAN_SECONDS));
		atomic_clear(&searching);
		int stop_err = bt_le_scan_stop();

		if (stop_err) {
			LOG_ERR("Scan stop failed: %d", stop_err);
			return 0;
		}
		if (err) {
			LOG_INF("Target not found; next scan 30 seconds after previous start");
			k_sleep(K_MSEC(MAX(0, RETRY_SECONDS * 1000 -
					  (k_uptime_get() - scan_started))));
			continue;
		}
		k_sem_reset(&connection_done);
		k_sem_reset(&disconnection_done);
		k_msgq_purge(&packets);
		err = bt_conn_le_create(&peer, BT_CONN_LE_CREATE_CONN,
					BT_LE_CONN_PARAM_DEFAULT, &conn);
		if (err) {
			LOG_ERR("Connection request failed: %d", err);
		} else {
			/* Controller creation timeout is 10 seconds; callback owns completion. */
			k_sem_take(&connection_done, K_FOREVER);
			if (atomic_get(&link_up)) {
				err = subscribe_updates(conn);
				if (err) {
					LOG_ERR("Update discovery/subscription failed: %d", err);
				} else {
					LOG_INF("Subscribed; ECG sends updates while collecting");
				}
				while (!err && atomic_get(&link_up)) {
					if (k_msgq_get(&packets, &ticks, K_MSEC(100))) {
						continue;
					}
					LOG_INF("Packet %u: RTC=%u ticks (512 Hz), LED=%s",
						++packet_count, ticks, led_names[next_led]);
					send_rtc(ticks);
					err = regulator_enable(led_supply);
					if (err) {
						LOG_ERR("LED supply enable failed: %d", err);
						break;
					}
					err = led_on(leds, led_order[next_led]);
					if (!err) {
						k_sleep(K_MSEC(200));
					}
					int off_err = led_off(leds, led_order[next_led]);
					int supply_err = regulator_disable(led_supply);

					if (err || off_err || supply_err) {
						LOG_ERR("LED pulse failed: on=%d off=%d supply=%d",
							err, off_err, supply_err);
						err = err ? err : (off_err ? off_err : supply_err);
					}
					next_led = (next_led + 1) % ARRAY_SIZE(led_order);
				}
				if (atomic_get(&link_up)) {
					err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
					if (err && err != -ENOTCONN) {
						LOG_ERR("Disconnect failed: %d", err);
						return 0;
					}
				}
				/* Do not reuse GATT storage until the connection has ended. */
				k_sem_take(&disconnection_done, K_FOREVER);
				retry_at_ms = disconnected_at_ms + RETRY_SECONDS * 1000;
			}
			bt_conn_unref(conn);
		}
		k_sleep(retry_at_ms ? K_MSEC(MAX(0, retry_at_ms - k_uptime_get())) :
			K_SECONDS(RETRY_SECONDS));
	}
}
