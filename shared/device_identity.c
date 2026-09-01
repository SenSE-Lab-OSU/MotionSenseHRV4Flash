/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/crc.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "msense_device_identity.h"

#define MSENSE_DEVICE_ID_SUFFIX_BITS 25U
#define MSENSE_DEVICE_ID_SUFFIX_MASK ((1UL << MSENSE_DEVICE_ID_SUFFIX_BITS) - 1UL)

static const char crockford_base32[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

static bool device_id_is_invalid(const uint8_t *id)
{
	uint8_t i;

	for (i = 0U; i < MSENSE_DEVICE_ID_LEN; i++) {
		if (id[i] != 0xFFU) {
			return false;
		}
	}

	return true;
}

static void format_digest(struct msense_device_identity *identity, uint32_t crc,
			  size_t prefix_len)
{
	uint32_t digest = crc & MSENSE_DEVICE_ID_SUFFIX_MASK;
	uint8_t i;

	for (i = 0U; i < MSENSE_DEVICE_NAME_SUFFIX_LEN; i++) {
		uint8_t shift = (MSENSE_DEVICE_NAME_SUFFIX_LEN - 1U - i) * 5U;

		identity->device_name[prefix_len + i] =
			crockford_base32[(digest >> shift) & 0x1FU];
	}
	identity->device_name[identity->device_name_len] = '\0';
}

int msense_device_identity_init(struct msense_device_identity *identity,
				const struct msense_device_identity_config *config)
{
	ssize_t id_len;
	uint32_t crc;
	int written;

	if (identity == NULL || config == NULL || config->ble_name_prefix == NULL ||
	    config->dis_model == NULL || config->ble_name_prefix_len == 0U ||
	    config->ble_name_len > MSENSE_DEVICE_NAME_MAX_LEN ||
	    config->ble_name_prefix_len + MSENSE_DEVICE_NAME_SUFFIX_LEN !=
		config->ble_name_len) {
		return -EINVAL;
	}

	if (identity->initialized) {
		return 0;
	}

	id_len = hwinfo_get_device_id(identity->device_id, sizeof(identity->device_id));
	if (id_len < 0) {
		return (int)id_len;
	}
	if (id_len != MSENSE_DEVICE_ID_LEN) {
		return -EMSGSIZE;
	}
	if (device_id_is_invalid(identity->device_id)) {
		return -ENODEV;
	}

	written = snprintf(identity->device_id_hex, sizeof(identity->device_id_hex),
			   "%02X%02X%02X%02X%02X%02X%02X%02X", identity->device_id[0],
			   identity->device_id[1], identity->device_id[2], identity->device_id[3],
			   identity->device_id[4], identity->device_id[5], identity->device_id[6],
			   identity->device_id[7]);
	if (written != MSENSE_DEVICE_ID_HEX_LEN) {
		return -EIO;
	}

	identity->device_name_len = config->ble_name_len;
	identity->dis_model = config->dis_model;
	memcpy(identity->device_name, config->ble_name_prefix, config->ble_name_prefix_len);
	crc = crc32_ieee(identity->device_id, sizeof(identity->device_id));
	format_digest(identity, crc, config->ble_name_prefix_len);
	identity->initialized = true;

	return 0;
}

const uint8_t *msense_device_identity_bytes(const struct msense_device_identity *identity)
{
	return identity->device_id;
}

const char *msense_device_identity_hex(const struct msense_device_identity *identity)
{
	return identity->device_id_hex;
}

const char *msense_device_identity_name(const struct msense_device_identity *identity)
{
	return identity->device_name;
}

size_t msense_device_identity_name_len(const struct msense_device_identity *identity)
{
	return identity->device_name_len;
}

const char *msense_device_identity_model(const struct msense_device_identity *identity)
{
	return identity->dis_model;
}
