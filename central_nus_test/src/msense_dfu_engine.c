/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <bluetooth/services/dfu_smp.h>

#include <zcbor_common.h>
#include <zcbor_decode.h>
#include <zcbor_encode.h>

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "msense_dfu_engine.h"
#include "msense_smp_central.h"

#define MSENSE_DFU_HASH_BYTES 32U
#define MSENSE_DFU_MAX_IMAGES 8U
#define MSENSE_DFU_SMP_REQUEST_BYTES 512U
#define MSENSE_DFU_SMP_RESPONSE_BYTES 2048U
#define MSENSE_DFU_SMP_TIMEOUT_MS 15000U
#define MSENSE_DFU_HOST_FRAME_TIMEOUT_MS 30000U
#define MSENSE_DFU_RECONNECT_TIMEOUT_MS 30000U
#define MSENSE_DFU_POST_RESET_RECONNECT_TIMEOUT_MS 60000U
/* SMP header plus the largest first-upload CBOR map without its image data:
 * map/image/data/off/len/sha, including max-width offset/length encodings. */
#define MSENSE_DFU_SMP_OVERHEAD_BYTES 80U
#define MSENSE_DFU_ATT_WRITE_OVERHEAD_BYTES 3U

/* SMP management protocol values.  Keep this client independent from the
 * peripheral's MCUboot/MCUmgr server headers: the Central only constructs and
 * consumes the on-wire management protocol. */
#define MSENSE_MGMT_OP_READ 0U
#define MSENSE_MGMT_OP_READ_RSP 1U
#define MSENSE_MGMT_OP_WRITE 2U
#define MSENSE_MGMT_OP_WRITE_RSP 3U
#define MSENSE_MGMT_GROUP_OS 0U
#define MSENSE_MGMT_GROUP_IMAGE 1U
#define MSENSE_OS_ID_RESET 5U
#define MSENSE_IMG_ID_STATE 0U
#define MSENSE_IMG_ID_UPLOAD 1U
#define MSENSE_IMG_ID_ERASE 5U

enum dfu_job_type {
	DFU_JOB_LIST,
	DFU_JOB_BEGIN,
	DFU_JOB_ERASE,
	DFU_JOB_CONFIRM,
	DFU_JOB_TEST,
	DFU_JOB_RESET,
};

struct dfu_job {
	enum dfu_job_type type;
	uint32_t transaction;
	uint32_t image_size;
	uint32_t slot;
	uint8_t upload_hash[MSENSE_DFU_HASH_BYTES];
	uint8_t tlv_hash[MSENSE_DFU_HASH_BYTES];
	bool allow_same;
};

struct dfu_image {
	uint32_t image;
	uint32_t slot;
	uint8_t hash[MSENSE_DFU_HASH_BYTES];
	bool bootable;
	bool pending;
	bool confirmed;
	bool active;
	bool permanent;
};

struct dfu_image_list {
	struct dfu_image images[MSENSE_DFU_MAX_IMAGES];
	size_t count;
};

struct dfu_smp_packet {
	struct bt_dfu_smp_header header;
	uint8_t payload[MSENSE_DFU_SMP_REQUEST_BYTES -
			sizeof(struct bt_dfu_smp_header)];
};

/* Only the dedicated DFU thread enters the SMP helpers. Keeping their large
 * request/response storage here leaves the thread stack for CBOR state and
 * workflow locals. */
struct dfu_smp_scratch {
	struct dfu_smp_packet packet;
	uint8_t response[MSENSE_DFU_SMP_RESPONSE_BYTES];
};

struct dfu_engine_context {
	struct msense_dfu_platform platform;
	msense_dfu_event_cb event_cb;
	void *event_context;
	atomic_t state;
	atomic_t accepting_job;
	atomic_t abort_requested;
	atomic_t credit_active;
	atomic_t wire_errors;
	atomic_t dropped_frames;
	atomic_t peer_connected;
	atomic_t peer_nus_ready;
	atomic_t peer_smp_ready;
	atomic_t disconnect_epoch;
	atomic_t peer_ready_epoch;
	atomic_t binary_rx_stopped;
	uint32_t transaction;
	uint32_t image_size;
	uint32_t offset;
	uint16_t credit_payload_max;
	uint8_t upload_hash[MSENSE_DFU_HASH_BYTES];
	uint8_t tlv_hash[MSENSE_DFU_HASH_BYTES];
	uint8_t smp_sequence;
	bool binary_port_claimed;
};

static struct dfu_engine_context dfu;
static struct dfu_smp_scratch smp_scratch;

K_MSGQ_DEFINE(dfu_job_queue, sizeof(struct dfu_job), 4, 4);
K_MEM_SLAB_DEFINE_STATIC(dfu_frame_slab, sizeof(struct msense_dfu_frame), 2, 4);
K_MSGQ_DEFINE(dfu_frame_queue, sizeof(struct msense_dfu_frame *), 2, 4);
K_SEM_DEFINE(dfu_frame_ready, 0, 2);
K_SEM_DEFINE(dfu_peer_ready, 0, 1);
K_SEM_DEFINE(dfu_disconnect_ready, 0, 1);

static void emit_event(const char *format, ...)
{
	char text[256];
	va_list args;

	if (dfu.event_cb == NULL) {
		return;
	}
	va_start(args, format);
	(void)vsnprintk(text, sizeof(text), format, args);
	va_end(args);
	dfu.event_cb(text, dfu.event_context);
}

static void hash_to_hex(const uint8_t hash[MSENSE_DFU_HASH_BYTES], char text[65])
{
	static const char hex[] = "0123456789abcdef";
	size_t index;

	for (index = 0U; index < MSENSE_DFU_HASH_BYTES; index++) {
		text[index * 2U] = hex[hash[index] >> 4];
		text[index * 2U + 1U] = hex[hash[index] & 0x0fU];
	}
	text[64] = '\0';
}

const char *msense_dfu_engine_state_name(enum msense_dfu_state state)
{
	switch (state) {
	case MSENSE_DFU_IDLE:
		return "IDLE";
	case MSENSE_DFU_LISTING:
		return "LISTING";
	case MSENSE_DFU_PREPARING:
		return "PREPARING";
	case MSENSE_DFU_RECEIVING:
		return "RECEIVING";
	case MSENSE_DFU_VERIFYING:
		return "VERIFYING";
	case MSENSE_DFU_TESTING:
		return "TESTING";
	case MSENSE_DFU_RESETTING:
		return "RESETTING";
	case MSENSE_DFU_RECONNECTING:
		return "RECONNECTING";
	case MSENSE_DFU_CONFIRMING:
		return "CONFIRMING";
	case MSENSE_DFU_COMPLETE:
		return "COMPLETE";
	case MSENSE_DFU_FAILED:
		return "FAILED";
	case MSENSE_DFU_ABORTED:
		return "ABORTED";
	default:
		return "UNKNOWN";
	}
}

static void set_state(enum msense_dfu_state state)
{
	atomic_set(&dfu.state, (atomic_val_t)state);
}

static enum msense_dfu_state get_state(void)
{
	return (enum msense_dfu_state)atomic_get(&dfu.state);
}

static void discard_queued_frames(void)
{
	struct msense_dfu_frame *frame;

	while (k_msgq_get(&dfu_frame_queue, &frame, K_NO_WAIT) == 0) {
		if (frame != NULL) {
			k_mem_slab_free(&dfu_frame_slab, (void *)frame);
		}
	}
	k_sem_reset(&dfu_frame_ready);
}

static void finish_job(enum msense_dfu_state terminal_state)
{
	atomic_set(&dfu.credit_active, 0);
	discard_queued_frames();
	if (dfu.binary_port_claimed) {
		dfu.platform.release_binary_port(dfu.platform.context);
		dfu.binary_port_claimed = false;
	}
	if (dfu.platform.set_reconnect_enabled != NULL) {
		dfu.platform.set_reconnect_enabled(dfu.platform.context, false);
	}
	set_state(terminal_state);
	atomic_set(&dfu.accepting_job, 1);
}

static void fail_job(const char *operation, int error, const char *reason)
{
	emit_event("DFU_FAIL tx=%u operation=%s code=%d reason=%s", dfu.transaction,
		   operation, error, reason);
	finish_job(MSENSE_DFU_FAILED);
}

static void abort_job(void)
{
	emit_event("DFU_ABORTED tx=%u offset=%u", dfu.transaction, dfu.offset);
	finish_job(MSENSE_DFU_ABORTED);
}

static uint16_t calculate_credit_payload_max(void)
{
	uint16_t mtu;
	uint16_t maximum;

	mtu = dfu.platform.att_mtu(dfu.platform.context);
	if (mtu <= MSENSE_DFU_ATT_WRITE_OVERHEAD_BYTES + MSENSE_DFU_SMP_OVERHEAD_BYTES) {
		return 0U;
	}
	maximum = mtu - MSENSE_DFU_ATT_WRITE_OVERHEAD_BYTES - MSENSE_DFU_SMP_OVERHEAD_BYTES;
	maximum &= (uint16_t)~0x3U;
	return MIN(maximum, (uint16_t)MSENSE_DFU_WIRE_MAX_PAYLOAD);
}

static int response_payload(const uint8_t *response, size_t response_length,
			    uint16_t expected_group, uint8_t expected_id,
			    uint8_t request_op, uint8_t expected_sequence,
			    const uint8_t **payload,
			    size_t *payload_length)
{
	const struct bt_dfu_smp_header *header;
	uint16_t group;
	uint16_t length;
	uint8_t expected_op;

	if (response_length < sizeof(*header)) {
		return -EBADMSG;
	}
	header = (const struct bt_dfu_smp_header *)response;
	group = ((uint16_t)header->group_h8 << 8) | header->group_l8;
	length = ((uint16_t)header->len_h8 << 8) | header->len_l8;
	expected_op = request_op == MSENSE_MGMT_OP_READ ? MSENSE_MGMT_OP_READ_RSP :
						       MSENSE_MGMT_OP_WRITE_RSP;
	if (header->op != expected_op || header->seq != expected_sequence ||
	    group != expected_group || header->id != expected_id ||
	    length != response_length - sizeof(*header)) {
		return -EBADMSG;
	}
	*payload = response + sizeof(*header);
	*payload_length = length;
	return 0;
}

static int smp_send(struct dfu_smp_packet *packet, size_t payload_length,
		    uint16_t group, uint8_t id, uint8_t operation,
		    uint8_t *response, size_t *response_length,
		    uint8_t *request_sequence)
{
	int error;
	uint8_t sequence;

	if (packet == NULL || response == NULL || response_length == NULL ||
	    request_sequence == NULL) {
		return -EINVAL;
	}
	if (payload_length > sizeof(packet->payload)) {
		return -EMSGSIZE;
	}
	packet->header.op = operation;
	packet->header.flags = 0U;
	packet->header.len_h8 = (uint8_t)(payload_length >> 8);
	packet->header.len_l8 = (uint8_t)payload_length;
	packet->header.group_h8 = (uint8_t)(group >> 8);
	packet->header.group_l8 = (uint8_t)group;
	sequence = dfu.smp_sequence++;
	packet->header.seq = sequence;
	packet->header.id = id;
	error = msense_smp_central_command((const uint8_t *)packet,
					  sizeof(packet->header) + payload_length, response,
					  MSENSE_DFU_SMP_RESPONSE_BYTES, response_length,
					  K_MSEC(MSENSE_DFU_SMP_TIMEOUT_MS));
	if (error == 0) {
		*request_sequence = sequence;
	}
	return error;
}

static int decode_response_rc(const uint8_t *payload, size_t payload_length)
{
	zcbor_state_t states[4];
	struct zcbor_string key;
	int32_t response_rc = 0;
	bool have_rc = false;

	if (payload_length == 0U) {
		return -EBADMSG;
	}
	zcbor_new_decode_state(states, ARRAY_SIZE(states), payload, payload_length, 1, NULL, 0);
	if (!zcbor_map_start_decode(states)) {
		return -EBADMSG;
	}
	while (!zcbor_array_at_end(states)) {
		if (!zcbor_tstr_decode(states, &key)) {
			return -EBADMSG;
		}
		if (key.len == 2U && memcmp(key.value, "rc", 2U) == 0) {
			if (!zcbor_int32_decode(states, &response_rc)) {
				return -EBADMSG;
			}
			have_rc = true;
		} else if (!zcbor_any_skip(states, NULL)) {
			return -EBADMSG;
		}
	}
	if (!zcbor_map_end_decode(states)) {
		return -EBADMSG;
	}
	return have_rc ? response_rc : 0;
}

static int decode_image_list(const uint8_t *payload, size_t payload_length,
			     struct dfu_image_list *list)
{
	zcbor_state_t states[8];
	struct zcbor_string key;
	bool images_found = false;

	memset(list, 0, sizeof(*list));
	zcbor_new_decode_state(states, ARRAY_SIZE(states), payload, payload_length, 1, NULL, 0);
	if (!zcbor_map_start_decode(states)) {
		return -EBADMSG;
	}
	while (!zcbor_array_at_end(states)) {
		if (!zcbor_tstr_decode(states, &key)) {
			return -EBADMSG;
		}
		if (key.len != 6U || memcmp(key.value, "images", 6U) != 0) {
			if (!zcbor_any_skip(states, NULL)) {
				return -EBADMSG;
			}
			continue;
		}
		images_found = true;
		if (!zcbor_list_start_decode(states)) {
			return -EBADMSG;
		}
		while (!zcbor_array_at_end(states)) {
			struct zcbor_string hash = { 0 };
			struct dfu_image *image;
			uint32_t image_number = 0U;
			uint32_t slot = UINT32_MAX;
			bool bootable = false;
			bool pending = false;
			bool confirmed = false;
			bool active = false;
			bool permanent = false;
			bool have_hash = false;
			bool have_slot = false;

			if (list->count >= ARRAY_SIZE(list->images) ||
			    !zcbor_map_start_decode(states)) {
				return -EMSGSIZE;
			}
			while (!zcbor_array_at_end(states)) {
				if (!zcbor_tstr_decode(states, &key)) {
					return -EBADMSG;
				}
				if (key.len == 4U && memcmp(key.value, "hash", 4U) == 0) {
					if (!zcbor_bstr_decode(states, &hash)) {
						return -EBADMSG;
					}
					have_hash = true;
				} else if (key.len == 4U && memcmp(key.value, "slot", 4U) == 0) {
					if (!zcbor_uint32_decode(states, &slot)) {
						return -EBADMSG;
					}
					have_slot = true;
				} else if (key.len == 5U && memcmp(key.value, "image", 5U) == 0) {
					if (!zcbor_uint32_decode(states, &image_number)) {
						return -EBADMSG;
					}
				} else if (key.len == 8U && memcmp(key.value, "bootable", 8U) == 0) {
					if (!zcbor_bool_decode(states, &bootable)) {
						return -EBADMSG;
					}
				} else if (key.len == 7U && memcmp(key.value, "pending", 7U) == 0) {
					if (!zcbor_bool_decode(states, &pending)) {
						return -EBADMSG;
					}
				} else if (key.len == 9U && memcmp(key.value, "confirmed", 9U) == 0) {
					if (!zcbor_bool_decode(states, &confirmed)) {
						return -EBADMSG;
					}
				} else if (key.len == 6U && memcmp(key.value, "active", 6U) == 0) {
					if (!zcbor_bool_decode(states, &active)) {
						return -EBADMSG;
					}
				} else if (key.len == 9U && memcmp(key.value, "permanent", 9U) == 0) {
					if (!zcbor_bool_decode(states, &permanent)) {
						return -EBADMSG;
					}
				} else if (!zcbor_any_skip(states, NULL)) {
					return -EBADMSG;
				}
			}
			if (!zcbor_map_end_decode(states) || !have_hash || !have_slot ||
			    hash.len != MSENSE_DFU_HASH_BYTES) {
				return -EBADMSG;
			}
			image = &list->images[list->count++];
			image->image = image_number;
			image->slot = slot;
			memcpy(image->hash, hash.value, sizeof(image->hash));
			image->bootable = bootable;
			image->pending = pending;
			image->confirmed = confirmed;
			image->active = active;
			image->permanent = permanent;
		}
		if (!zcbor_list_end_decode(states)) {
			return -EBADMSG;
		}
	}
	if (!zcbor_map_end_decode(states) || !images_found) {
		return -EBADMSG;
	}
	return list->count == 0U ? -ENOENT : 0;
}

static int smp_image_list(struct dfu_image_list *list)
{
	struct dfu_smp_packet *packet = &smp_scratch.packet;
	uint8_t *response = smp_scratch.response;
	const uint8_t *payload;
	zcbor_state_t states[2];
	size_t response_length;
	size_t payload_length;
	size_t encoded_length;
	uint8_t request_sequence;
	int error;

	memset(packet, 0, sizeof(*packet));
	zcbor_new_encode_state(states, ARRAY_SIZE(states), packet->payload, sizeof(packet->payload),
			       0);
	if (!zcbor_map_start_encode(states, 1) || !zcbor_map_end_encode(states, 1)) {
		return -ENOMEM;
	}
	encoded_length = states->payload - packet->payload;
	error = smp_send(packet, encoded_length, MSENSE_MGMT_GROUP_IMAGE,
			 MSENSE_IMG_ID_STATE, MSENSE_MGMT_OP_READ, response, &response_length,
			 &request_sequence);
	if (error != 0) {
		return error;
	}
	error = response_payload(response, response_length, MSENSE_MGMT_GROUP_IMAGE,
				 MSENSE_IMG_ID_STATE, MSENSE_MGMT_OP_READ, request_sequence,
				 &payload, &payload_length);
	if (error != 0) {
		return error;
	}
	return decode_image_list(payload, payload_length, list);
}

static int smp_state_write(const uint8_t hash[MSENSE_DFU_HASH_BYTES], bool confirm)
{
	struct dfu_smp_packet *packet = &smp_scratch.packet;
	uint8_t *response = smp_scratch.response;
	const uint8_t *payload;
	zcbor_state_t states[3];
	size_t response_length;
	size_t payload_length;
	size_t encoded_length;
	uint8_t request_sequence;
	int error;
	bool ok;

	memset(packet, 0, sizeof(*packet));
	zcbor_new_encode_state(states, ARRAY_SIZE(states), packet->payload, sizeof(packet->payload),
			       0);
	ok = zcbor_map_start_encode(states, hash == NULL ? 2 : 4) &&
	     zcbor_tstr_put_lit(states, "confirm") && zcbor_bool_put(states, confirm);
	if (ok && hash != NULL) {
		ok = zcbor_tstr_put_lit(states, "hash") &&
		     zcbor_bstr_encode_ptr(states, hash, MSENSE_DFU_HASH_BYTES);
	}
	if (ok) {
		ok = zcbor_map_end_encode(states, hash == NULL ? 2 : 4);
	}
	if (!ok) {
		return -ENOMEM;
	}
	encoded_length = states->payload - packet->payload;
	error = smp_send(packet, encoded_length, MSENSE_MGMT_GROUP_IMAGE,
			 MSENSE_IMG_ID_STATE, MSENSE_MGMT_OP_WRITE, response, &response_length,
			 &request_sequence);
	if (error != 0) {
		return error;
	}
	error = response_payload(response, response_length, MSENSE_MGMT_GROUP_IMAGE,
				 MSENSE_IMG_ID_STATE, MSENSE_MGMT_OP_WRITE, request_sequence,
				 &payload, &payload_length);
	if (error != 0) {
		return error;
	}
	return decode_response_rc(payload, payload_length);
}

static int smp_image_erase(uint32_t slot)
{
	struct dfu_smp_packet *packet = &smp_scratch.packet;
	uint8_t *response = smp_scratch.response;
	const uint8_t *payload;
	zcbor_state_t states[3];
	size_t response_length;
	size_t payload_length;
	size_t encoded_length;
	uint8_t request_sequence;
	int error;

	memset(packet, 0, sizeof(*packet));
	zcbor_new_encode_state(states, ARRAY_SIZE(states), packet->payload, sizeof(packet->payload),
			       0);
	if (!zcbor_map_start_encode(states, 2) || !zcbor_tstr_put_lit(states, "slot") ||
	    !zcbor_uint32_put(states, slot) || !zcbor_map_end_encode(states, 2)) {
		return -ENOMEM;
	}
	encoded_length = states->payload - packet->payload;
	error = smp_send(packet, encoded_length, MSENSE_MGMT_GROUP_IMAGE,
			 MSENSE_IMG_ID_ERASE, MSENSE_MGMT_OP_WRITE, response, &response_length,
			 &request_sequence);
	if (error != 0) {
		return error;
	}
	error = response_payload(response, response_length, MSENSE_MGMT_GROUP_IMAGE,
				 MSENSE_IMG_ID_ERASE, MSENSE_MGMT_OP_WRITE, request_sequence,
				 &payload, &payload_length);
	if (error != 0) {
		return error;
	}
	return decode_response_rc(payload, payload_length);
}

static int smp_reset(void)
{
	struct dfu_smp_packet *packet = &smp_scratch.packet;
	uint8_t *response = smp_scratch.response;
	const uint8_t *payload;
	zcbor_state_t states[2];
	size_t response_length;
	size_t payload_length;
	size_t encoded_length;
	uint8_t request_sequence;
	int error;

	memset(packet, 0, sizeof(*packet));
	zcbor_new_encode_state(states, ARRAY_SIZE(states), packet->payload, sizeof(packet->payload),
			       0);
	if (!zcbor_map_start_encode(states, 1) || !zcbor_map_end_encode(states, 1)) {
		return -ENOMEM;
	}
	encoded_length = states->payload - packet->payload;
	error = smp_send(packet, encoded_length, MSENSE_MGMT_GROUP_OS, MSENSE_OS_ID_RESET,
			 MSENSE_MGMT_OP_WRITE, response, &response_length, &request_sequence);
	if (error == -ENOTCONN) {
		/* A peer can reset before its write response reaches the Central. */
		return 0;
	}
	if (error != 0) {
		return error;
	}
	error = response_payload(response, response_length, MSENSE_MGMT_GROUP_OS,
				 MSENSE_OS_ID_RESET, MSENSE_MGMT_OP_WRITE, request_sequence,
				 &payload, &payload_length);
	if (error != 0) {
		return error;
	}
	return decode_response_rc(payload, payload_length);
}

static int decode_upload_offset(const uint8_t *payload, size_t payload_length,
				uint32_t *offset)
{
	zcbor_state_t states[4];
	struct zcbor_string key;
	size_t decoded_offset = SIZE_MAX;
	int32_t response_rc = 0;
	bool have_offset = false;

	zcbor_new_decode_state(states, ARRAY_SIZE(states), payload, payload_length, 1, NULL, 0);
	if (!zcbor_map_start_decode(states)) {
		return -EBADMSG;
	}
	while (!zcbor_array_at_end(states)) {
		if (!zcbor_tstr_decode(states, &key)) {
			return -EBADMSG;
		}
		if (key.len == 3U && memcmp(key.value, "off", 3U) == 0) {
			if (!zcbor_size_decode(states, &decoded_offset)) {
				return -EBADMSG;
			}
			have_offset = true;
		} else if (key.len == 2U && memcmp(key.value, "rc", 2U) == 0) {
			if (!zcbor_int32_decode(states, &response_rc)) {
				return -EBADMSG;
			}
		} else if (!zcbor_any_skip(states, NULL)) {
			return -EBADMSG;
		}
	}
	if (!zcbor_map_end_decode(states) || !have_offset || decoded_offset > UINT32_MAX) {
		return -EBADMSG;
	}
	if (response_rc != 0) {
		return response_rc;
	}
	*offset = (uint32_t)decoded_offset;
	return 0;
}

static int smp_upload(const struct msense_dfu_frame *frame, uint32_t *target_offset)
{
	struct dfu_smp_packet *packet = &smp_scratch.packet;
	uint8_t *response = smp_scratch.response;
	const uint8_t *payload;
	zcbor_state_t states[4];
	size_t response_length;
	size_t payload_length;
	size_t encoded_length;
	uint8_t request_sequence;
	int error;
	bool ok;

	memset(packet, 0, sizeof(*packet));
	zcbor_new_encode_state(states, ARRAY_SIZE(states), packet->payload, sizeof(packet->payload),
			       0);
	ok = zcbor_map_start_encode(states, frame->offset == 0U ? 10 : 6) &&
	     zcbor_tstr_put_lit(states, "image") && zcbor_uint32_put(states, 0U) &&
	     zcbor_tstr_put_lit(states, "data") &&
	     zcbor_bstr_encode_ptr(states, frame->payload, frame->payload_length) &&
	     zcbor_tstr_put_lit(states, "off") && zcbor_size_put(states, frame->offset);
	if (ok && frame->offset == 0U) {
		ok = zcbor_tstr_put_lit(states, "len") && zcbor_size_put(states, dfu.image_size) &&
		     zcbor_tstr_put_lit(states, "sha") &&
		     zcbor_bstr_encode_ptr(states, dfu.upload_hash, sizeof(dfu.upload_hash));
	}
	if (ok) {
		ok = zcbor_map_end_encode(states, frame->offset == 0U ? 10 : 6);
	}
	if (!ok) {
		return -EMSGSIZE;
	}
	encoded_length = states->payload - packet->payload;
	error = smp_send(packet, encoded_length, MSENSE_MGMT_GROUP_IMAGE,
			 MSENSE_IMG_ID_UPLOAD, MSENSE_MGMT_OP_WRITE, response, &response_length,
			 &request_sequence);
	if (error != 0) {
		return error;
	}
	error = response_payload(response, response_length, MSENSE_MGMT_GROUP_IMAGE,
				 MSENSE_IMG_ID_UPLOAD, MSENSE_MGMT_OP_WRITE, request_sequence,
				 &payload, &payload_length);
	if (error != 0) {
		return error;
	}
	return decode_upload_offset(payload, payload_length, target_offset);
}

static bool image_matches(const struct dfu_image *image,
			  const uint8_t hash[MSENSE_DFU_HASH_BYTES])
{
	return image->image == 0U && memcmp(image->hash, hash, MSENSE_DFU_HASH_BYTES) == 0;
}

static const struct dfu_image *find_matching_image(const struct dfu_image_list *list,
					    const uint8_t hash[MSENSE_DFU_HASH_BYTES],
					    bool active)
{
	size_t index;

	for (index = 0U; index < list->count; index++) {
		if (image_matches(&list->images[index], hash) && list->images[index].active == active) {
			return &list->images[index];
		}
	}
	return NULL;
}

static void emit_image_list(uint32_t transaction, const struct dfu_image_list *list)
{
	size_t index;

	emit_event("DFU_LIST_BEGIN tx=%u count=%u", transaction, (uint32_t)list->count);
	for (index = 0U; index < list->count; index++) {
		const struct dfu_image *image = &list->images[index];
		char hash[65];

		hash_to_hex(image->hash, hash);
		emit_event("DFU_IMAGE tx=%u image=%u slot=%u hash=%s bootable=%u pending=%u "
			   "confirmed=%u active=%u permanent=%u",
			   transaction, image->image, image->slot, hash, image->bootable,
			   image->pending, image->confirmed, image->active, image->permanent);
	}
	emit_event("DFU_LIST_END tx=%u", transaction);
}

static bool peer_ready_for_dfu(void)
{
	if (!atomic_get(&dfu.peer_smp_ready)) {
		return false;
	}
	return !dfu.platform.peer_requires_nus(dfu.platform.context) ||
	       atomic_get(&dfu.peer_nus_ready);
}

static bool wait_for_reconnect(void)
{
	if (peer_ready_for_dfu()) {
		return true;
	}
	k_sem_reset(&dfu_peer_ready);
	if (peer_ready_for_dfu()) {
		return true;
	}
	if (k_sem_take(&dfu_peer_ready, K_MSEC(MSENSE_DFU_RECONNECT_TIMEOUT_MS)) != 0) {
		return false;
	}
	return peer_ready_for_dfu();
}

static bool wait_for_epoch_change(atomic_t *epoch, uint32_t previous,
				  struct k_sem *event)
{
	int64_t start_ms = k_uptime_get();

	while ((uint32_t)atomic_get(epoch) == previous) {
		int64_t elapsed_ms = k_uptime_get() - start_ms;
		int64_t remaining_ms;

		if (elapsed_ms >= MSENSE_DFU_RECONNECT_TIMEOUT_MS) {
			return false;
		}
		remaining_ms = MSENSE_DFU_RECONNECT_TIMEOUT_MS - elapsed_ms;
		(void)k_sem_take(event, K_MSEC(remaining_ms));
	}
	return true;
}

static bool wait_for_reset_reconnect(uint32_t previous_disconnect_epoch,
				     uint32_t previous_peer_ready_epoch)
{
	int64_t start_ms;

	if (!wait_for_epoch_change(&dfu.disconnect_epoch, previous_disconnect_epoch,
				   &dfu_disconnect_ready)) {
		return false;
	}
	emit_event("DFU_RESET_DISCONNECTED tx=%u", dfu.transaction);
	start_ms = k_uptime_get();
	while (true) {
		int64_t elapsed_ms;
		int64_t remaining_ms;

		if ((uint32_t)atomic_get(&dfu.peer_ready_epoch) != previous_peer_ready_epoch &&
		    peer_ready_for_dfu()) {
			return true;
		}
		elapsed_ms = k_uptime_get() - start_ms;
		if (elapsed_ms >= MSENSE_DFU_POST_RESET_RECONNECT_TIMEOUT_MS) {
			return false;
		}
		remaining_ms = MSENSE_DFU_POST_RESET_RECONNECT_TIMEOUT_MS - elapsed_ms;
		(void)k_sem_take(&dfu_peer_ready, K_MSEC(remaining_ms));
	}
}

static bool recover_connection(void)
{
	set_state(MSENSE_DFU_RECONNECTING);
	emit_event("DFU_RECONNECT tx=%u", dfu.transaction);
	dfu.platform.request_reconnect(dfu.platform.context);
	if (!wait_for_reconnect()) {
		return false;
	}
	set_state(MSENSE_DFU_RECEIVING);
	emit_event("DFU_RECONNECTED tx=%u", dfu.transaction);
	return true;
}

static void emit_credit(void)
{
	atomic_set(&dfu.credit_active, 1);
	k_sem_reset(&dfu_frame_ready);
	emit_event("DFU_CREDIT tx=%u off=%u max=%u", dfu.transaction, dfu.offset,
		   dfu.credit_payload_max);
}

static int wait_for_frame(struct msense_dfu_frame **frame)
{
	uint32_t seen_wire_errors = (uint32_t)atomic_get(&dfu.wire_errors);
	int64_t start_ms = k_uptime_get();

	while (true) {
		int64_t elapsed_ms;
		int64_t remaining_ms;
		int64_t wait_ms;

		if (atomic_get(&dfu.abort_requested)) {
			return -ECANCELED;
		}
		if (!atomic_get(&dfu.peer_connected)) {
			return -ENOTCONN;
		}
		if (atomic_get(&dfu.binary_rx_stopped)) {
			return -EIO;
		}
		if ((uint32_t)atomic_get(&dfu.wire_errors) != seen_wire_errors) {
			emit_event("DFU_RETRY tx=%u off=%u reason=wire", dfu.transaction, dfu.offset);
			return -EAGAIN;
		}
		elapsed_ms = k_uptime_get() - start_ms;
		if (elapsed_ms >= MSENSE_DFU_HOST_FRAME_TIMEOUT_MS) {
			return -ETIMEDOUT;
		}
		remaining_ms = MSENSE_DFU_HOST_FRAME_TIMEOUT_MS - elapsed_ms;
		wait_ms = MIN(remaining_ms, 100);
		if (k_sem_take(&dfu_frame_ready, K_MSEC(wait_ms)) == 0) {
			if (k_msgq_get(&dfu_frame_queue, frame, K_NO_WAIT) == 0) {
				return 0;
			}
		}
	}
}

static int verify_secondary_image(void)
{
	struct dfu_image_list list;
	const struct dfu_image *image;
	int error;

	error = smp_image_list(&list);
	if (error != 0) {
		return error;
	}
	emit_image_list(dfu.transaction, &list);
	image = find_matching_image(&list, dfu.tlv_hash, false);
	if (image == NULL || !image->bootable) {
		return -ENOEXEC;
	}
	return 0;
}

static int verify_secondary_pending(void)
{
	struct dfu_image_list list;
	const struct dfu_image *image;
	int error;

	error = smp_image_list(&list);
	if (error != 0) {
		return error;
	}
	emit_image_list(dfu.transaction, &list);
	image = find_matching_image(&list, dfu.tlv_hash, false);
	if (image == NULL || !image->bootable || !image->pending) {
		return -EAGAIN;
	}
	return 0;
}

static int verify_active_confirmed(bool require_confirmed)
{
	struct dfu_image_list list;
	const struct dfu_image *image;
	int error;

	error = smp_image_list(&list);
	if (error != 0) {
		return error;
	}
	emit_image_list(dfu.transaction, &list);
	image = find_matching_image(&list, dfu.tlv_hash, true);
	if (image == NULL) {
		return -ENOEXEC;
	}
	if (require_confirmed && !image->confirmed) {
		return -EAGAIN;
	}
	return 0;
}

static int complete_upgrade(void)
{
	uint32_t previous_disconnect_epoch;
	uint32_t previous_peer_ready_epoch;
	int error;

	set_state(MSENSE_DFU_VERIFYING);
	error = verify_secondary_image();
	if (error != 0) {
		return error;
	}
	emit_event("DFU_SECONDARY_VERIFIED tx=%u", dfu.transaction);

	set_state(MSENSE_DFU_TESTING);
	error = smp_state_write(dfu.tlv_hash, false);
	if (error != 0) {
		return error;
	}
	emit_event("DFU_TESTED tx=%u", dfu.transaction);
	error = verify_secondary_pending();
	if (error != 0) {
		return error;
	}
	emit_event("DFU_PENDING_VERIFIED tx=%u", dfu.transaction);

	set_state(MSENSE_DFU_RESETTING);
	/* Mark the reconnect expectation before reset can remove the connection. */
	previous_disconnect_epoch = (uint32_t)atomic_get(&dfu.disconnect_epoch);
	previous_peer_ready_epoch = (uint32_t)atomic_get(&dfu.peer_ready_epoch);
	k_sem_reset(&dfu_disconnect_ready);
	k_sem_reset(&dfu_peer_ready);
	dfu.platform.request_reconnect(dfu.platform.context);
	error = smp_reset();
	if (error != 0) {
		return error;
	}

	set_state(MSENSE_DFU_RECONNECTING);
	if (!wait_for_reset_reconnect(previous_disconnect_epoch, previous_peer_ready_epoch)) {
		return -ETIMEDOUT;
	}
	emit_event("DFU_POSTBOOT_READY tx=%u", dfu.transaction);

	set_state(MSENSE_DFU_VERIFYING);
	error = verify_active_confirmed(false);
	if (error != 0) {
		return error;
	}
	emit_event("DFU_ACTIVE_VERIFIED tx=%u", dfu.transaction);

	set_state(MSENSE_DFU_CONFIRMING);
	/* The active image was verified by exact hash above. Confirm the image that
	 * is currently running, then require that exact hash to report confirmed. */
	error = smp_state_write(NULL, true);
	if (error != 0) {
		return error;
	}
	error = verify_active_confirmed(true);
	if (error != 0) {
		return error;
	}
	return 0;
}

static int run_upload(void)
{
	int error;

	set_state(MSENSE_DFU_RECEIVING);
	while (dfu.offset < dfu.image_size) {
		struct msense_dfu_frame *frame = NULL;
		uint32_t target_offset;
		uint16_t frame_length;

		emit_credit();
		error = wait_for_frame(&frame);
		if (error == -ECANCELED) {
			return error;
		}
		if (error == -EAGAIN) {
			continue;
		}
		if (error == -ENOTCONN) {
			if (!recover_connection()) {
				return -ENOTCONN;
			}
			continue;
		}
		if (error != 0) {
			return error;
		}
		if (frame->transaction != dfu.transaction || frame->offset != dfu.offset ||
		    frame->payload_length == 0U || frame->payload_length > dfu.credit_payload_max ||
		    frame->payload_length > dfu.image_size - dfu.offset) {
			emit_event("DFU_RETRY tx=%u off=%u reason=wrong_frame", dfu.transaction,
				   dfu.offset);
			k_mem_slab_free(&dfu_frame_slab, (void *)frame);
			continue;
		}

		frame_length = frame->payload_length;
		target_offset = dfu.offset;
		error = smp_upload(frame, &target_offset);
		if (error == -ENOTCONN) {
			k_mem_slab_free(&dfu_frame_slab, (void *)frame);
			if (!recover_connection()) {
				return -ENOTCONN;
			}
			continue;
		}
		k_mem_slab_free(&dfu_frame_slab, (void *)frame);
		if (error != 0) {
			return error;
		}
		if (target_offset > dfu.image_size) {
			return -ERANGE;
		}
		if (target_offset != dfu.offset + frame_length) {
			if (target_offset == 0U && dfu.offset != 0U) {
				emit_event("DFU_RESTART tx=%u previous_off=%u", dfu.transaction,
					   dfu.offset);
			}
			dfu.offset = target_offset;
			emit_event("DFU_RETRY tx=%u off=%u reason=target_offset", dfu.transaction,
				   dfu.offset);
			continue;
		}
		dfu.offset = target_offset;
		emit_event("DFU_PROGRESS tx=%u off=%u total=%u", dfu.transaction, dfu.offset,
			   dfu.image_size);
	}

	return complete_upgrade();
}

static int begin_upload(const struct dfu_job *job)
{
	struct dfu_image_list list;
	const struct dfu_image *active;
	int error;

	set_state(MSENSE_DFU_PREPARING);
	if (!dfu.platform.smp_ready(dfu.platform.context)) {
		return -ENOTCONN;
	}
	if (!dfu.platform.security_ok(dfu.platform.context)) {
		return -EACCES;
	}
	dfu.credit_payload_max = calculate_credit_payload_max();
	if (dfu.credit_payload_max == 0U) {
		return -EMSGSIZE;
	}
	atomic_set(&dfu.binary_rx_stopped, 0);
	error = dfu.platform.claim_binary_port(dfu.platform.context);
	if (error != 0) {
		return error;
	}
	dfu.binary_port_claimed = true;
	if (dfu.platform.set_reconnect_enabled != NULL) {
		dfu.platform.set_reconnect_enabled(dfu.platform.context, true);
	}

	error = smp_image_list(&list);
	if (error != 0) {
		return error;
	}
	emit_image_list(dfu.transaction, &list);
	active = find_matching_image(&list, dfu.tlv_hash, true);
	if (active != NULL && !job->allow_same) {
		return -EALREADY;
	}
	emit_event("DFU_BEGIN_READY tx=%u bytes=%u chunk_max=%u", dfu.transaction,
		   dfu.image_size, dfu.credit_payload_max);
	return run_upload();
}

static void run_list_job(void)
{
	struct dfu_image_list list;
	int error;

	set_state(MSENSE_DFU_LISTING);
	if (!dfu.platform.smp_ready(dfu.platform.context)) {
		fail_job("list", -ENOTCONN, "smp_not_ready");
		return;
	}
	error = smp_image_list(&list);
	if (error != 0) {
		fail_job("list", error, "smp_list");
		return;
	}
	emit_image_list(dfu.transaction, &list);
	emit_event("DFU_DONE tx=%u operation=list", dfu.transaction);
	finish_job(MSENSE_DFU_COMPLETE);
}

static void run_erase_job(const struct dfu_job *job)
{
	int error;

	set_state(MSENSE_DFU_VERIFYING);
	if (!dfu.platform.smp_ready(dfu.platform.context)) {
		fail_job("erase", -ENOTCONN, "smp_not_ready");
		return;
	}
	error = smp_image_erase(job->slot);
	if (error != 0) {
		fail_job("erase", error, "smp_erase");
		return;
	}
	emit_event("DFU_DONE tx=%u operation=erase slot=%u", dfu.transaction, job->slot);
	finish_job(MSENSE_DFU_COMPLETE);
}

static void run_state_job(const struct dfu_job *job, bool confirm)
{
	int error;

	set_state(confirm ? MSENSE_DFU_CONFIRMING : MSENSE_DFU_TESTING);
	if (!dfu.platform.smp_ready(dfu.platform.context)) {
		fail_job(confirm ? "confirm" : "test", -ENOTCONN, "smp_not_ready");
		return;
	}
	error = smp_state_write(job->tlv_hash, confirm);
	if (error != 0) {
		fail_job(confirm ? "confirm" : "test", error, "smp_state");
		return;
	}
	emit_event("DFU_DONE tx=%u operation=%s", dfu.transaction,
		   confirm ? "confirm" : "test");
	finish_job(MSENSE_DFU_COMPLETE);
}

static void run_reset_job(void)
{
	int error;

	set_state(MSENSE_DFU_RESETTING);
	if (!dfu.platform.smp_ready(dfu.platform.context)) {
		fail_job("reset", -ENOTCONN, "smp_not_ready");
		return;
	}
	error = smp_reset();
	if (error != 0) {
		fail_job("reset", error, "smp_reset");
		return;
	}
	emit_event("DFU_DONE tx=%u operation=reset", dfu.transaction);
	finish_job(MSENSE_DFU_COMPLETE);
}

static void dfu_thread(void *arg1, void *arg2, void *arg3)
{
	struct dfu_job job;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		if (k_msgq_get(&dfu_job_queue, &job, K_FOREVER) != 0) {
			continue;
		}
		dfu.transaction = job.transaction;
		dfu.image_size = job.image_size;
		dfu.offset = 0U;
		memcpy(dfu.upload_hash, job.upload_hash, sizeof(dfu.upload_hash));
		memcpy(dfu.tlv_hash, job.tlv_hash, sizeof(dfu.tlv_hash));
		atomic_set(&dfu.abort_requested, 0);

		switch (job.type) {
		case DFU_JOB_LIST:
			run_list_job();
			break;
		case DFU_JOB_BEGIN: {
			int error = begin_upload(&job);

			if (error == -ECANCELED) {
				abort_job();
			} else if (error != 0) {
				const char *reason = "workflow";

				if (error == -EALREADY) {
					reason = "same_active_image";
				} else if (error == -EBUSY) {
					reason = "binary_port_busy";
				} else if (error == -ETIMEDOUT) {
					reason = "timeout";
				} else if (error == -ENOTCONN) {
					reason = "disconnected";
				} else if (error == -EACCES) {
					reason = "security_required";
				} else if (error == -EIO) {
					reason = "binary_rx_stopped";
				}
				fail_job("upgrade", error, reason);
			} else {
				emit_event("DFU_SUCCESS tx=%u bytes=%u", dfu.transaction,
					   dfu.image_size);
				finish_job(MSENSE_DFU_COMPLETE);
			}
			break;
		}
		case DFU_JOB_ERASE:
			run_erase_job(&job);
			break;
		case DFU_JOB_CONFIRM:
			run_state_job(&job, true);
			break;
		case DFU_JOB_TEST:
			run_state_job(&job, false);
			break;
		case DFU_JOB_RESET:
			run_reset_job();
			break;
		default:
			fail_job("unknown", -EINVAL, "internal");
			break;
		}
	}
}

K_THREAD_DEFINE(msense_dfu_thread_id, 4096, dfu_thread, NULL, NULL, NULL, 7, 0, 0);

int msense_dfu_engine_init(const struct msense_dfu_platform *platform,
			   msense_dfu_event_cb event_cb, void *event_context)
{
	if (platform == NULL || platform->claim_binary_port == NULL ||
	    platform->release_binary_port == NULL || platform->smp_ready == NULL ||
	    platform->security_ok == NULL || platform->att_mtu == NULL ||
	    platform->peer_requires_nus == NULL || platform->request_reconnect == NULL) {
		return -EINVAL;
	}
	memset(&dfu, 0, sizeof(dfu));
	dfu.platform = *platform;
	dfu.event_cb = event_cb;
	dfu.event_context = event_context;
	k_sem_reset(&dfu_frame_ready);
	k_sem_reset(&dfu_peer_ready);
	k_sem_reset(&dfu_disconnect_ready);
	set_state(MSENSE_DFU_IDLE);
	atomic_set(&dfu.accepting_job, 1);
	return 0;
}

static int queue_job(const struct dfu_job *job)
{
	if (!atomic_cas(&dfu.accepting_job, 1, 0)) {
		return -EBUSY;
	}
	if (k_msgq_put(&dfu_job_queue, job, K_NO_WAIT) != 0) {
		atomic_set(&dfu.accepting_job, 1);
		return -ENOMEM;
	}
	return 0;
}

int msense_dfu_engine_request_list(uint32_t transaction)
{
	struct dfu_job job = {
		.type = DFU_JOB_LIST,
		.transaction = transaction,
	};

	return transaction == 0U ? -EINVAL : queue_job(&job);
}

int msense_dfu_engine_request_begin(uint32_t transaction, uint32_t image_size,
				    const uint8_t upload_hash[32],
				    const uint8_t tlv_hash[32], bool allow_same)
{
	struct dfu_job job = {
		.type = DFU_JOB_BEGIN,
		.transaction = transaction,
		.image_size = image_size,
		.allow_same = allow_same,
	};

	if (transaction == 0U || image_size == 0U || upload_hash == NULL || tlv_hash == NULL) {
		return -EINVAL;
	}
	memcpy(job.upload_hash, upload_hash, sizeof(job.upload_hash));
	memcpy(job.tlv_hash, tlv_hash, sizeof(job.tlv_hash));
	return queue_job(&job);
}

int msense_dfu_engine_request_erase(uint32_t transaction, uint32_t slot)
{
	struct dfu_job job = {
		.type = DFU_JOB_ERASE,
		.transaction = transaction,
		.slot = slot,
	};

	return transaction == 0U ? -EINVAL : queue_job(&job);
}

static int queue_hash_job(enum dfu_job_type type, uint32_t transaction, const uint8_t hash[32])
{
	struct dfu_job job = {
		.type = type,
		.transaction = transaction,
	};

	if (transaction == 0U || hash == NULL) {
		return -EINVAL;
	}
	memcpy(job.tlv_hash, hash, sizeof(job.tlv_hash));
	return queue_job(&job);
}

int msense_dfu_engine_request_confirm(uint32_t transaction, const uint8_t hash[32])
{
	return queue_hash_job(DFU_JOB_CONFIRM, transaction, hash);
}

int msense_dfu_engine_request_test(uint32_t transaction, const uint8_t hash[32])
{
	return queue_hash_job(DFU_JOB_TEST, transaction, hash);
}

int msense_dfu_engine_request_reset(uint32_t transaction)
{
	struct dfu_job job = {
		.type = DFU_JOB_RESET,
		.transaction = transaction,
	};

	return transaction == 0U ? -EINVAL : queue_job(&job);
}

int msense_dfu_engine_abort(uint32_t transaction)
{
	if (transaction == 0U || transaction != dfu.transaction ||
	    get_state() == MSENSE_DFU_IDLE || atomic_get(&dfu.accepting_job)) {
		return -ENOENT;
	}
	atomic_set(&dfu.abort_requested, 1);
	k_sem_give(&dfu_frame_ready);
	return 0;
}

void msense_dfu_engine_receive_frame(const struct msense_dfu_frame *frame)
{
	struct msense_dfu_frame *stored_frame;

	if (frame == NULL || get_state() != MSENSE_DFU_RECEIVING ||
	    !atomic_cas(&dfu.credit_active, 1, 0)) {
		atomic_inc(&dfu.dropped_frames);
		return;
	}
	if (k_mem_slab_alloc(&dfu_frame_slab, (void **)&stored_frame, K_NO_WAIT) != 0) {
		atomic_set(&dfu.credit_active, 1);
		atomic_inc(&dfu.dropped_frames);
		return;
	}
	memcpy(stored_frame, frame, sizeof(*stored_frame));
	if (k_msgq_put(&dfu_frame_queue, &stored_frame, K_NO_WAIT) != 0) {
		k_mem_slab_free(&dfu_frame_slab, (void *)stored_frame);
		atomic_set(&dfu.credit_active, 1);
		atomic_inc(&dfu.dropped_frames);
		return;
	}
	k_sem_give(&dfu_frame_ready);
}

void msense_dfu_engine_note_wire_error(enum msense_dfu_wire_error error)
{
	ARG_UNUSED(error);
	atomic_inc(&dfu.wire_errors);
	if (get_state() == MSENSE_DFU_RECEIVING) {
		k_sem_give(&dfu_frame_ready);
	}
}

void msense_dfu_engine_connection_lost(void)
{
	atomic_set(&dfu.peer_connected, 0);
	atomic_set(&dfu.peer_nus_ready, 0);
	atomic_set(&dfu.peer_smp_ready, 0);
	atomic_inc(&dfu.disconnect_epoch);
	k_sem_give(&dfu_disconnect_ready);
	k_sem_give(&dfu_frame_ready);
}

void msense_dfu_engine_peer_ready(bool nus_ready, bool smp_ready)
{
	atomic_set(&dfu.peer_connected, 1);
	atomic_set(&dfu.peer_nus_ready, nus_ready ? 1 : 0);
	atomic_set(&dfu.peer_smp_ready, smp_ready ? 1 : 0);
	atomic_inc(&dfu.peer_ready_epoch);
	k_sem_give(&dfu_peer_ready);
}

void msense_dfu_engine_binary_rx_stopped(void)
{
	atomic_set(&dfu.binary_rx_stopped, 1);
	if (get_state() == MSENSE_DFU_RECEIVING) {
		k_sem_give(&dfu_frame_ready);
	}
}

void msense_dfu_engine_get_status(struct msense_dfu_status *status)
{
	if (status == NULL) {
		return;
	}
	status->state = get_state();
	status->transaction = dfu.transaction;
	status->offset = dfu.offset;
	status->image_size = dfu.image_size;
	status->credit_payload_max = dfu.credit_payload_max;
	status->dropped_frames = (uint32_t)atomic_get(&dfu.dropped_frames);
	status->binary_port_claimed = dfu.binary_port_claimed;
}
