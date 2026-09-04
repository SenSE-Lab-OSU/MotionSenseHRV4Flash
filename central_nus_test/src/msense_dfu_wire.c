/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>

#include <stdbool.h>
#include <string.h>

#include "msense_dfu_wire.h"

static const uint8_t mdfu_magic[] = { 'M', 'D', 'F', 'U' };

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		uint8_t bit;

		crc ^= data[index];
		for (bit = 0U; bit < 8U; bit++) {
			crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
		}
	}

	return crc;
}

uint32_t msense_dfu_wire_crc32(const uint8_t *data, size_t length)
{
	return crc32_update(UINT32_MAX, data, length) ^ UINT32_MAX;
}

static uint32_t frame_crc32(const uint8_t *frame, uint16_t payload_length)
{
	uint32_t crc;

	crc = crc32_update(UINT32_MAX, &frame[4], 16U);
	crc = crc32_update(crc, &frame[MSENSE_DFU_WIRE_HEADER_BYTES], payload_length);
	return crc ^ UINT32_MAX;
}

static void parser_drop_prefix(struct msense_dfu_wire_parser *parser, size_t count)
{
	if (count >= parser->used) {
		parser->used = 0U;
		return;
	}

	memmove(parser->buffer, &parser->buffer[count], parser->used - count);
	parser->used -= count;
}

static bool parser_process_one(struct msense_dfu_wire_parser *parser,
			       msense_dfu_wire_frame_cb frame_cb,
			       msense_dfu_wire_error_cb error_cb, void *context)
{
	uint16_t header_size;
	uint16_t payload_length;
	uint16_t reserved;
	uint32_t received_crc;
	size_t frame_length;

	if (parser->used < sizeof(mdfu_magic)) {
		return false;
	}
	if (memcmp(parser->buffer, mdfu_magic, sizeof(mdfu_magic)) != 0) {
		parser_drop_prefix(parser, 1U);
		return true;
	}
	if (parser->used < MSENSE_DFU_WIRE_HEADER_BYTES) {
		return false;
	}

	header_size = sys_get_le16(&parser->buffer[6]);
	payload_length = sys_get_le16(&parser->buffer[16]);
	reserved = sys_get_le16(&parser->buffer[18]);
	if (parser->buffer[4] != MSENSE_DFU_WIRE_VERSION ||
	    parser->buffer[5] != MSENSE_DFU_WIRE_TYPE_DATA ||
	    header_size != MSENSE_DFU_WIRE_HEADER_BYTES ||
	    payload_length > MSENSE_DFU_WIRE_MAX_PAYLOAD || reserved != 0U) {
		if (error_cb != NULL) {
			error_cb(MSENSE_DFU_WIRE_ERROR_HEADER, context);
		}
		parser_drop_prefix(parser, 1U);
		return true;
	}

	frame_length = MSENSE_DFU_WIRE_HEADER_BYTES + payload_length;
	if (parser->used < frame_length) {
		return false;
	}

	received_crc = sys_get_le32(&parser->buffer[20]);
	if (received_crc != frame_crc32(parser->buffer, payload_length)) {
		if (error_cb != NULL) {
			error_cb(MSENSE_DFU_WIRE_ERROR_CRC, context);
		}
		parser_drop_prefix(parser, 1U);
		return true;
	}

	parser->frame.transaction = sys_get_le32(&parser->buffer[8]);
	parser->frame.offset = sys_get_le32(&parser->buffer[12]);
	parser->frame.payload_length = payload_length;
	memcpy(parser->frame.payload, &parser->buffer[MSENSE_DFU_WIRE_HEADER_BYTES],
	       payload_length);
	if (frame_cb != NULL) {
		frame_cb(&parser->frame, context);
	}
	parser_drop_prefix(parser, frame_length);
	return true;
}

void msense_dfu_wire_parser_init(struct msense_dfu_wire_parser *parser)
{
	memset(parser, 0, sizeof(*parser));
}

void msense_dfu_wire_parser_feed(struct msense_dfu_wire_parser *parser,
				 const uint8_t *data, size_t length,
				 msense_dfu_wire_frame_cb frame_cb,
				 msense_dfu_wire_error_cb error_cb, void *context)
{
	size_t index;

	for (index = 0U; index < length; index++) {
		if (parser->used == sizeof(parser->buffer)) {
			if (error_cb != NULL) {
				error_cb(MSENSE_DFU_WIRE_ERROR_HEADER, context);
			}
			parser_drop_prefix(parser, 1U);
		}
		parser->buffer[parser->used++] = data[index];
		while (parser_process_one(parser, frame_cb, error_cb, context)) {
			/* Consume every complete or invalid frame before accepting more input. */
		}
	}
}
