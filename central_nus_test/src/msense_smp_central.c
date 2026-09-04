/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/dfu_smp.h>

#include <errno.h>
#include <string.h>

#include "msense_smp_central.h"

#define MSENSE_SMP_RESPONSE_STORAGE_BYTES 2048U

struct smp_central_context {
	struct bt_dfu_smp dfu_smp;
	struct k_mutex command_lock;
	struct k_sem command_complete;
	struct k_work completion_work;
	msense_smp_ready_cb ready_cb;
	void *ready_context;
	uint8_t response_storage[MSENSE_SMP_RESPONSE_STORAGE_BYTES];
	size_t response_length;
	int command_error;
	atomic_t ready;
	atomic_t command_pending;
	atomic_t completion_pending;
	atomic_t transport_stalled;
};

static struct smp_central_context smp_central;

static void smp_error(struct bt_dfu_smp *dfu_smp, int error);

static int reset_dfu_smp_client(void)
{
	const struct bt_dfu_smp_init_params params = {
		.error_cb = smp_error,
	};

	return bt_dfu_smp_init(&smp_central.dfu_smp, &params);
}

static void command_complete_work(struct k_work *work)
{
	ARG_UNUSED(work);
	atomic_set(&smp_central.completion_pending, 0);
	k_sem_give(&smp_central.command_complete);
}

static void schedule_command_completion(void)
{
	if (atomic_cas(&smp_central.completion_pending, 0, 1)) {
		(void)k_work_submit(&smp_central.completion_work);
	}
}

static void smp_error(struct bt_dfu_smp *dfu_smp, int error)
{
	ARG_UNUSED(dfu_smp);
	if (atomic_cas(&smp_central.command_pending, 1, 0)) {
		smp_central.command_error = error;
		schedule_command_completion();
	}
}

static void smp_response_part(struct bt_dfu_smp *dfu_smp)
{
	const struct bt_dfu_smp_rsp_state *state;

	state = bt_dfu_smp_rsp_state(dfu_smp);
	if (!atomic_get(&smp_central.command_pending)) {
		return;
	}
	if (state->offset > sizeof(smp_central.response_storage) ||
	    state->chunk_size > sizeof(smp_central.response_storage) - state->offset) {
		smp_central.command_error = -EMSGSIZE;
	} else {
		memcpy(&smp_central.response_storage[state->offset], state->data, state->chunk_size);
		if (state->offset + state->chunk_size >= state->total_size) {
			smp_central.response_length = state->total_size;
		}
	}

	if (bt_dfu_smp_rsp_total_check(dfu_smp)) {
		if (atomic_cas(&smp_central.command_pending, 1, 0)) {
			schedule_command_completion();
		}
	}
}

static void smp_discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	int error;

	ARG_UNUSED(context);
	error = bt_dfu_smp_handles_assign(dm, &smp_central.dfu_smp);
	(void)bt_gatt_dm_data_release(dm);
	if (error == 0) {
		atomic_set(&smp_central.ready, 1);
	}
	if (smp_central.ready_cb != NULL) {
		smp_central.ready_cb(error == 0, error, smp_central.ready_context);
	}
}

static void smp_discovery_not_found(struct bt_conn *connection, void *context)
{
	ARG_UNUSED(connection);
	ARG_UNUSED(context);
	atomic_set(&smp_central.ready, 0);
	if (smp_central.ready_cb != NULL) {
		smp_central.ready_cb(false, -ENOENT, smp_central.ready_context);
	}
}

static void smp_discovery_error(struct bt_conn *connection, int error, void *context)
{
	ARG_UNUSED(connection);
	ARG_UNUSED(context);
	atomic_set(&smp_central.ready, 0);
	if (smp_central.ready_cb != NULL) {
		smp_central.ready_cb(false, error, smp_central.ready_context);
	}
}

static const struct bt_gatt_dm_cb smp_discovery_callbacks = {
	.completed = smp_discovery_complete,
	.service_not_found = smp_discovery_not_found,
	.error_found = smp_discovery_error,
};

int msense_smp_central_init(msense_smp_ready_cb ready_cb, void *context)
{
	memset(&smp_central, 0, sizeof(smp_central));
	k_mutex_init(&smp_central.command_lock);
	k_sem_init(&smp_central.command_complete, 0, 1);
	k_work_init(&smp_central.completion_work, command_complete_work);
	smp_central.ready_cb = ready_cb;
	smp_central.ready_context = context;
	return reset_dfu_smp_client();
}

int msense_smp_central_discover(struct bt_conn *connection)
{
	int error;

	if (connection == NULL) {
		return -EINVAL;
	}
	atomic_set(&smp_central.ready, 0);
	error = bt_gatt_dm_start(connection, BT_UUID_DFU_SMP_SERVICE,
				&smp_discovery_callbacks, NULL);
	return error;
}

void msense_smp_central_disconnected(struct bt_conn *connection)
{
	ARG_UNUSED(connection);
	atomic_set(&smp_central.ready, 0);
	if (atomic_cas(&smp_central.command_pending, 1, 0)) {
		smp_central.command_error = -ENOTCONN;
		schedule_command_completion();
	}
	/* bt_dfu_smp retains the old connection and volatile subscription object.
	 * Reinitializing at disconnect makes a post-reset rediscovery subscribe to
	 * the new peer instead of trying to use stale notification state. */
	(void)reset_dfu_smp_client();
	atomic_set(&smp_central.transport_stalled, 0);
}

bool msense_smp_central_ready(void)
{
	return atomic_get(&smp_central.ready) != 0;
}

int msense_smp_central_command(const uint8_t *command, size_t command_length,
			       uint8_t *response, size_t response_capacity,
			       size_t *response_length, k_timeout_t timeout)
{
	int error;

	if (command == NULL || command_length == 0U || response == NULL ||
	    response_capacity < sizeof(struct bt_dfu_smp_header) || response_length == NULL) {
		return -EINVAL;
	}
	if (!msense_smp_central_ready()) {
		return -ENOTCONN;
	}
	if (atomic_get(&smp_central.transport_stalled)) {
		return -ETIMEDOUT;
	}

	k_mutex_lock(&smp_central.command_lock, K_FOREVER);
	if (!msense_smp_central_ready()) {
		k_mutex_unlock(&smp_central.command_lock);
		return -ENOTCONN;
	}
	if (atomic_get(&smp_central.transport_stalled)) {
		k_mutex_unlock(&smp_central.command_lock);
		return -ETIMEDOUT;
	}
	k_sem_reset(&smp_central.command_complete);
	smp_central.response_length = 0U;
	smp_central.command_error = 0;
	atomic_set(&smp_central.command_pending, 1);
	error = bt_dfu_smp_command(&smp_central.dfu_smp, smp_response_part,
				   command_length, command);
	if (error != 0) {
		atomic_set(&smp_central.command_pending, 0);
		k_mutex_unlock(&smp_central.command_lock);
		return error;
	}
	if (k_sem_take(&smp_central.command_complete, timeout) != 0) {
		/* The Nordic client intentionally exposes no cancel API. Do not issue a
		 * second request while it can still own its response callback. A new
		 * discovery after disconnect resets this condition. */
		atomic_set(&smp_central.transport_stalled, 1);
		k_mutex_unlock(&smp_central.command_lock);
		return -ETIMEDOUT;
	}
	error = smp_central.command_error;
	if (smp_central.response_length > response_capacity) {
		error = -EMSGSIZE;
	} else if (error == 0) {
		memcpy(response, smp_central.response_storage, smp_central.response_length);
	}
	*response_length = smp_central.response_length;
	k_mutex_unlock(&smp_central.command_lock);
	return error;
}
