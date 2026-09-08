/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_DFU_WIRE_H_
#define MSENSE_DFU_WIRE_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSENSE_DFU_WIRE_VERSION 1U
#define MSENSE_DFU_WIRE_TYPE_DATA 1U
#define MSENSE_DFU_WIRE_HEADER_BYTES 24U
#define MSENSE_DFU_WIRE_MAX_PAYLOAD 384U
#define MSENSE_DFU_WIRE_MAX_FRAME_BYTES \
	(MSENSE_DFU_WIRE_HEADER_BYTES + MSENSE_DFU_WIRE_MAX_PAYLOAD)

enum msense_dfu_wire_error {
	MSENSE_DFU_WIRE_ERROR_HEADER,
	MSENSE_DFU_WIRE_ERROR_CRC,
};

struct msense_dfu_frame {
	uint32_t transaction;
	uint32_t offset;
	uint16_t payload_length;
	uint8_t payload[MSENSE_DFU_WIRE_MAX_PAYLOAD];
};

typedef void (*msense_dfu_wire_frame_cb)(const struct msense_dfu_frame *frame,
					 void *context);
typedef void (*msense_dfu_wire_error_cb)(enum msense_dfu_wire_error error,
					 void *context);

struct msense_dfu_wire_parser {
	uint8_t buffer[MSENSE_DFU_WIRE_MAX_FRAME_BYTES];
	size_t used;
	struct msense_dfu_frame frame;
};

uint32_t msense_dfu_wire_crc32(const uint8_t *data, size_t length);

void msense_dfu_wire_parser_init(struct msense_dfu_wire_parser *parser);

void msense_dfu_wire_parser_feed(struct msense_dfu_wire_parser *parser,
				 const uint8_t *data, size_t length,
				 msense_dfu_wire_frame_cb frame_cb,
				 msense_dfu_wire_error_cb error_cb, void *context);

#ifdef __cplusplus
}
#endif

#endif /* MSENSE_DFU_WIRE_H_ */
