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
	uint16_t record_size;
	uint32_t history_record_count;
	uint32_t forward_record_count;
};

/**
 * Initialize the shared NUS stream endpoint with immutable record geometry.
 */
int msense_sensor_stream_init(const struct msense_sensor_stream_config *config);

/** Notify the stream module that authoritative primary recording has started. */
void msense_sensor_stream_recording_started(void);

/** Notify the stream module that authoritative primary recording has stopped. */
void msense_sensor_stream_recording_stopped(void);

/**
 * Mirror one finalized whole record. The stream copies it before returning;
 * this call is bounded, nonblocking, allocation-free, and producer-safe.
 */
int msense_sensor_stream_accept_record(const void *record, size_t record_size);

/** Notify the stream module that primary storage entered a terminal fault. */
void msense_sensor_stream_storage_failed(int error);

/** Notify the ECG stream that acquisition or another non-storage path failed. */
void msense_sensor_stream_recording_failed(int error);

#endif /* MSENSE_SENSOR_STREAM_H_ */
