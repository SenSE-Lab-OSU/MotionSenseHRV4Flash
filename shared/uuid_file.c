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
		/* Do not close/sync a handle after a possibly partial NAND write. */
		return (int)bytes_written;
	} else if (bytes_written != (ssize_t)contents_len) {
		return -EIO;
	}
	close_rc = fs_close(&name_file);
	return close_rc;
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
