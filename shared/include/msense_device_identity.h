/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_DEVICE_IDENTITY_H_
#define MSENSE_DEVICE_IDENTITY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MSENSE_DEVICE_ID_LEN 8U
#define MSENSE_DEVICE_ID_HEX_LEN (MSENSE_DEVICE_ID_LEN * 2U)
#define MSENSE_DEVICE_NAME_SUFFIX_LEN 5U
#define MSENSE_DEVICE_NAME_MAX_LEN 16U

struct msense_device_identity_config {
	const char *ble_name_prefix;
	size_t ble_name_prefix_len;
	size_t ble_name_len;
	const char *dis_model;
};

struct msense_device_identity {
	uint8_t device_id[MSENSE_DEVICE_ID_LEN];
	char device_id_hex[MSENSE_DEVICE_ID_HEX_LEN + 1U];
	char device_name[MSENSE_DEVICE_NAME_MAX_LEN + 1U];
	const char *dis_model;
	size_t device_name_len;
	bool initialized;
};

int msense_device_identity_init(struct msense_device_identity *identity,
				const struct msense_device_identity_config *config);

const uint8_t *msense_device_identity_bytes(const struct msense_device_identity *identity);

const char *msense_device_identity_hex(const struct msense_device_identity *identity);

const char *msense_device_identity_name(const struct msense_device_identity *identity);

size_t msense_device_identity_name_len(const struct msense_device_identity *identity);

const char *msense_device_identity_model(const struct msense_device_identity *identity);

#endif /* MSENSE_DEVICE_IDENTITY_H_ */
