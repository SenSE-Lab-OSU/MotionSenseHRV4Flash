/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_SMP_CENTRAL_H_
#define MSENSE_SMP_CENTRAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

struct bt_conn;

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*msense_smp_ready_cb)(bool ready, int error, void *context);

/** Initialize the singleton SMP-over-BLE transport before Bluetooth is enabled. */
int msense_smp_central_init(msense_smp_ready_cb ready_cb, void *context);

/** Start GATT discovery for the DFU SMP service on an established connection. */
int msense_smp_central_discover(struct bt_conn *connection);

/** Tell the transport that its current connection has been removed. */
void msense_smp_central_disconnected(struct bt_conn *connection);

bool msense_smp_central_ready(void);

/**
 * Send one complete SMP request and wait for its complete response.
 *
 * The caller owns the command and response buffers. Only one call may be in
 * flight. This function is intended for the dedicated DFU thread, never a
 * Bluetooth callback or the system workqueue.
 */
int msense_smp_central_command(const uint8_t *command, size_t command_length,
			       uint8_t *response, size_t response_capacity,
			       size_t *response_length, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* MSENSE_SMP_CENTRAL_H_ */
