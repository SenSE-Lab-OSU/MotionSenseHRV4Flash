/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_UUID_FILE_H_
#define MSENSE_UUID_FILE_H_

#include <stdbool.h>
#include <stddef.h>

/* Reads uuid.txt without modifying it. */
int msense_uuid_file_ble_address_present(const char *uuid_name,
					 bool *ble_address_present);

/* Creates uuid.txt after the caller has established that it is absent. */
int msense_uuid_file_create(const char *uuid_name, const char *contents,
			    size_t contents_len);

/* Prepends the historic static-random BLE address line if it is absent. */
int msense_uuid_file_prepend_ble_address(const char *uuid_name,
					 const char *ble_address);

#endif /* MSENSE_UUID_FILE_H_ */
