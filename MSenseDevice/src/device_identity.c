#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/crc.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include "device_identity.h"

#define MSENSE_BLE_NAME_PREFIX "MSense4PPG-"
#define MSENSE_BLE_NAME_PREFIX_LEN (sizeof(MSENSE_BLE_NAME_PREFIX) - 1U)
#define MSENSE_DEVICE_ID_SUFFIX_BITS 25U
#define MSENSE_DEVICE_ID_SUFFIX_MASK ((1UL << MSENSE_DEVICE_ID_SUFFIX_BITS) - 1UL)

static const char crockford_base32[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

static uint8_t device_id[MSENSE_DEVICE_ID_LEN];
static char device_id_hex[MSENSE_DEVICE_ID_HEX_LEN + 1U];
static char device_name[MSENSE_BLE_NAME_LEN + 1U];
static bool identity_initialized;

BUILD_ASSERT(MSENSE_BLE_NAME_PREFIX_LEN + 5U == MSENSE_BLE_NAME_LEN,
             "BLE name prefix and suffix must each match their lengths");

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

static void format_digest(uint32_t crc)
{
	uint32_t digest = crc & MSENSE_DEVICE_ID_SUFFIX_MASK;
	uint8_t i;

	for (i = 0U; i < 5U; i++) {
		uint8_t shift = (4U - i) * 5U;

		device_name[MSENSE_BLE_NAME_PREFIX_LEN + i] =
			crockford_base32[(digest >> shift) & 0x1FU];
	}
	device_name[MSENSE_BLE_NAME_LEN] = '\0';
}

int msense_device_identity_init(void)
{
	ssize_t id_len;
	uint32_t crc;
	int written;

	if (identity_initialized) {
		return 0;
	}

	id_len = hwinfo_get_device_id(device_id, sizeof(device_id));
	if (id_len < 0) {
		return (int)id_len;
	}
	if (id_len != MSENSE_DEVICE_ID_LEN) {
		return -EMSGSIZE;
	}
	if (device_id_is_invalid(device_id)) {
		return -ENODEV;
	}

	written = snprintf(device_id_hex, sizeof(device_id_hex),
			   "%02X%02X%02X%02X%02X%02X%02X%02X",
			   device_id[0], device_id[1], device_id[2], device_id[3],
			   device_id[4], device_id[5], device_id[6], device_id[7]);
	if (written != MSENSE_DEVICE_ID_HEX_LEN) {
		return -EIO;
	}

	snprintf(device_name, sizeof(device_name), "%s", MSENSE_BLE_NAME_PREFIX);
	crc = crc32_ieee(device_id, sizeof(device_id));
	format_digest(crc);
	identity_initialized = true;

	return 0;
}

const uint8_t *msense_device_identity_bytes(void)
{
	return device_id;
}

const char *msense_device_identity_hex(void)
{
	return device_id_hex;
}

const char *msense_device_identity_name(void)
{
	return device_name;
}
