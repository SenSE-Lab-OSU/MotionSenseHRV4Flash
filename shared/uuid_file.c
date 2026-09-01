/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/fs/fs.h>

#include <errno.h>
#include <string.h>

#include "msense_uuid_file.h"

#define UUID_CONTENTS_MAX_SIZE 640U
#define UUID_BLE_ADDRESS_LINE_LEN 26U
#define UUID_BLE_ADDRESS_CONTENTS_MAX_SIZE \
	(UUID_CONTENTS_MAX_SIZE + UUID_BLE_ADDRESS_LINE_LEN + 1U)

static bool uuid_is_hex_digit(char value)
{
	return (value >= '0' && value <= '9') ||
	       (value >= 'A' && value <= 'F') ||
	       (value >= 'a' && value <= 'f');
}

static bool uuid_has_ble_address_line(const char *contents, size_t contents_len)
{
	size_t i;

	if (contents == NULL || contents_len < UUID_BLE_ADDRESS_LINE_LEN ||
	    memcmp(&contents[17U], " (random)", 9U) != 0) {
		return false;
	}
	if (contents_len > UUID_BLE_ADDRESS_LINE_LEN &&
	    contents[UUID_BLE_ADDRESS_LINE_LEN] != '\n' &&
	    contents[UUID_BLE_ADDRESS_LINE_LEN] != '\r') {
		return false;
	}

	for (i = 0U; i < 17U; i++) {
		if (i == 2U || i == 5U || i == 8U || i == 11U || i == 14U) {
			if (contents[i] != ':') {
				return false;
			}
		} else if (!uuid_is_hex_digit(contents[i])) {
			return false;
		}
	}

	return true;
}

static int read_uuid_contents(const char *uuid_name, char *contents,
			      size_t contents_size, size_t *contents_len)
{
	struct fs_dirent entry;
	struct fs_file_t name_file;
	int close_rc;
	int rc;
	ssize_t bytes_read;

	if (uuid_name == NULL || contents == NULL || contents_len == NULL) {
		return -EINVAL;
	}

	*contents_len = 0U;
	rc = fs_stat(uuid_name, &entry);
	if (rc != 0) {
		return rc;
	}
	if (entry.size > contents_size) {
		return -EFBIG;
	}

	fs_file_t_init(&name_file);
	rc = fs_open(&name_file, uuid_name, FS_O_READ);
	if (rc != 0) {
		return rc;
	}

	bytes_read = fs_read(&name_file, contents, entry.size);
	if (bytes_read < 0) {
		rc = (int)bytes_read;
	} else if (bytes_read != (ssize_t)entry.size) {
		rc = -EIO;
	} else {
		*contents_len = (size_t)bytes_read;
		rc = 0;
	}

	close_rc = fs_close(&name_file);
	if (rc == 0 && close_rc != 0) {
		rc = close_rc;
	}

	return rc;
}

static int write_uuid_contents(const char *uuid_name, const char *contents,
			       size_t contents_len, fs_mode_t flags)
{
	struct fs_file_t name_file;
	int close_rc;
	int rc;
	int sync_rc = 0;
	ssize_t bytes_written;

	if (uuid_name == NULL || contents == NULL) {
		return -EINVAL;
	}

	fs_file_t_init(&name_file);
	rc = fs_open(&name_file, uuid_name, flags);
	if (rc != 0) {
		return rc;
	}

	bytes_written = fs_write(&name_file, contents, contents_len);
	if (bytes_written < 0) {
		rc = (int)bytes_written;
	} else if (bytes_written != (ssize_t)contents_len) {
		rc = -EIO;
	} else {
		rc = 0;
	}
	if (rc == 0) {
		sync_rc = fs_sync(&name_file);
	}
	close_rc = fs_close(&name_file);
	if (rc == 0 && sync_rc != 0) {
		rc = sync_rc;
	}
	if (rc == 0 && close_rc != 0) {
		rc = close_rc;
	}

	return rc;
}

int msense_uuid_file_ble_address_present(const char *uuid_name,
					 bool *ble_address_present)
{
	char uuid_contents[UUID_CONTENTS_MAX_SIZE];
	int rc;
	size_t uuid_contents_len;

	if (ble_address_present == NULL) {
		return -EINVAL;
	}
	*ble_address_present = false;

	rc = read_uuid_contents(uuid_name, uuid_contents, sizeof(uuid_contents),
				&uuid_contents_len);
	if (rc == 0) {
		*ble_address_present = uuid_has_ble_address_line(uuid_contents,
							  uuid_contents_len);
	}

	return rc;
}

int msense_uuid_file_create(const char *uuid_name, const char *contents,
			    size_t contents_len)
{
	struct fs_dirent entry;
	int rc;

	if (uuid_name == NULL || contents == NULL) {
		return -EINVAL;
	}

	rc = fs_stat(uuid_name, &entry);
	if (rc == 0) {
		return -EEXIST;
	}
	if (rc != -ENOENT) {
		return rc;
	}

	return write_uuid_contents(uuid_name, contents, contents_len,
				   FS_O_CREATE | FS_O_WRITE);
}

int msense_uuid_file_prepend_ble_address(const char *uuid_name,
					 const char *ble_address)
{
	char uuid_contents[UUID_BLE_ADDRESS_CONTENTS_MAX_SIZE];
	int rc;
	size_t ble_address_len;
	size_t uuid_contents_len;

	if (ble_address == NULL) {
		return -EINVAL;
	}

	ble_address_len = strlen(ble_address);
	if (ble_address_len != UUID_BLE_ADDRESS_LINE_LEN ||
	    !uuid_has_ble_address_line(ble_address, ble_address_len)) {
		return -EINVAL;
	}

	rc = read_uuid_contents(uuid_name, uuid_contents, UUID_CONTENTS_MAX_SIZE,
				&uuid_contents_len);
	if (rc != 0) {
		return rc;
	}
	if (uuid_has_ble_address_line(uuid_contents, uuid_contents_len)) {
		return 0;
	}
	if (ble_address_len + 1U + uuid_contents_len > sizeof(uuid_contents)) {
		return -ENOSPC;
	}

	memmove(uuid_contents + ble_address_len + 1U, uuid_contents, uuid_contents_len);
	memcpy(uuid_contents, ble_address, ble_address_len);
	uuid_contents[ble_address_len] = '\n';

	return write_uuid_contents(uuid_name, uuid_contents,
				   ble_address_len + 1U + uuid_contents_len,
				   FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
}
