/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_UUID_FILE_H_
#define MSENSE_UUID_FILE_H_

#include <stddef.h>

/* Creates uuid.txt when absent without changing an existing file. */
int msense_uuid_file_ensure(const char *uuid_name, const char *contents,
			    size_t contents_len);

#endif /* MSENSE_UUID_FILE_H_ */
