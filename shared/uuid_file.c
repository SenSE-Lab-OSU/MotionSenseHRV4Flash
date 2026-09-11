/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/fs/fs.h>

#include <errno.h>

#include "msense_uuid_file.h"

static int write_uuid_contents(const char *uuid_name, const char *contents,
			       size_t contents_len)
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
	rc = fs_open(&name_file, uuid_name, FS_O_CREATE | FS_O_WRITE);
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

int msense_uuid_file_ensure(const char *uuid_name, const char *contents,
			    size_t contents_len)
{
	struct fs_dirent entry;
	int rc;

	if (uuid_name == NULL || contents == NULL) {
		return -EINVAL;
	}

	rc = fs_stat(uuid_name, &entry);
	if (rc == 0) {
		return 0;
	}
	if (rc != -ENOENT) {
		return rc;
	}

	return write_uuid_contents(uuid_name, contents, contents_len);
}
