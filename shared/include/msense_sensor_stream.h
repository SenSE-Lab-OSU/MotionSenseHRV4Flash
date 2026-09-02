/*
 * Copyright (c) 2026 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSENSE_SENSOR_STREAM_H_
#define MSENSE_SENSOR_STREAM_H_

#include <stddef.h>
#include <stdint.h>

#include "msense_sensor_stream_protocol.h"

struct msense_sensor_stream_config {
	uint8_t device_type;
	uint8_t record_format_version;
	uint16_t record_size;
	uint32_t record_rate_numerator;
	uint32_t record_rate_denominator;
	uint32_t history_record_count;
	uint32_t forward_record_count;
	const uint8_t *device_id;
	const char *device_name;
	size_t device_name_len;
	const char *git_commit;
	const char *git_tree_state;
};

/**
 * Initialize the shared NUS stream endpoint with immutable product metadata.
 * The pointed-to values are copied during this call.
 */
int msense_sensor_stream_init(const struct msense_sensor_stream_config *config);

/** Notify the stream module that authoritative primary recording has started. */
void msense_sensor_stream_recording_started(void);

/** Notify the stream module that authoritative primary recording has stopped. */
void msense_sensor_stream_recording_stopped(void);

/**
 * Mirror a record accepted by the storage pipeline. This call is bounded,
 * nonblocking, allocation-free, and safe from producer contexts.
 */
int msense_sensor_stream_accept_record(const void *record, size_t record_size);

/** Notify the stream module that primary storage entered a terminal fault. */
void msense_sensor_stream_storage_failed(int error);

#endif /* MSENSE_SENSOR_STREAM_H_ */
