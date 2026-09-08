/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_DFU_ENGINE_H_
#define MSENSE_DFU_ENGINE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "msense_dfu_wire.h"

#ifdef __cplusplus
extern "C" {
#endif

enum msense_dfu_state {
	MSENSE_DFU_IDLE,
	MSENSE_DFU_LISTING,
	MSENSE_DFU_PREPARING,
	MSENSE_DFU_RECEIVING,
	MSENSE_DFU_VERIFYING,
	MSENSE_DFU_TESTING,
	MSENSE_DFU_RESETTING,
	MSENSE_DFU_RECONNECTING,
	MSENSE_DFU_CONFIRMING,
	MSENSE_DFU_COMPLETE,
	MSENSE_DFU_FAILED,
	MSENSE_DFU_ABORTED,
};

struct msense_dfu_status {
	enum msense_dfu_state state;
	uint32_t transaction;
	uint32_t offset;
	uint32_t image_size;
	uint16_t credit_payload_max;
	uint32_t dropped_frames;
	bool binary_port_claimed;
};

struct msense_dfu_platform {
	int (*claim_binary_port)(void *context);
	void (*release_binary_port)(void *context);
	bool (*smp_ready)(void *context);
	bool (*security_ok)(void *context);
	uint16_t (*att_mtu)(void *context);
	bool (*peer_requires_nus)(void *context);
	void (*request_reconnect)(void *context);
	void (*set_reconnect_enabled)(void *context, bool enabled);
	void *context;
};

typedef void (*msense_dfu_event_cb)(const char *text, void *context);

int msense_dfu_engine_init(const struct msense_dfu_platform *platform,
			   msense_dfu_event_cb event_cb, void *event_context);

int msense_dfu_engine_request_list(uint32_t transaction);
int msense_dfu_engine_request_begin(uint32_t transaction, uint32_t image_size,
				    const uint8_t upload_hash[32],
				    const uint8_t tlv_hash[32], bool allow_same);
int msense_dfu_engine_request_erase(uint32_t transaction, uint32_t slot);
int msense_dfu_engine_request_confirm(uint32_t transaction, const uint8_t hash[32]);
int msense_dfu_engine_request_test(uint32_t transaction, const uint8_t hash[32]);
int msense_dfu_engine_request_reset(uint32_t transaction);
int msense_dfu_engine_abort(uint32_t transaction);

void msense_dfu_engine_receive_frame(const struct msense_dfu_frame *frame);
void msense_dfu_engine_note_wire_error(enum msense_dfu_wire_error error);
void msense_dfu_engine_binary_rx_stopped(void);
void msense_dfu_engine_connection_lost(void);
void msense_dfu_engine_peer_ready(bool nus_ready, bool smp_ready);
void msense_dfu_engine_get_status(struct msense_dfu_status *status);
const char *msense_dfu_engine_state_name(enum msense_dfu_state state);

#ifdef __cplusplus
}
#endif

#endif /* MSENSE_DFU_ENGINE_H_ */
