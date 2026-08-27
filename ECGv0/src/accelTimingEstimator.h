#ifndef ACCEL_TIMING_ESTIMATOR_H_
#define ACCEL_TIMING_ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>

#define ACCEL_TIMING_ESTIMATOR_WINDOW 32U

struct accel_timing_observation {
	uint64_t sample_sequence;
	uint32_t rtc_ticks;
};

struct accel_timing_estimator {
	struct accel_timing_observation observations[ACCEL_TIMING_ESTIMATOR_WINDOW];
	uint8_t count;
	uint64_t period_q32;
};

void accel_timing_estimator_reset(struct accel_timing_estimator *estimator);
int accel_timing_estimator_observe(
	struct accel_timing_estimator *estimator,
	const struct accel_timing_observation *observation);
bool accel_timing_estimator_ready(const struct accel_timing_estimator *estimator);
int accel_timing_estimator_estimate(
	const struct accel_timing_estimator *estimator, uint64_t sample_sequence,
	uint32_t *estimated_rtc_ticks, uint32_t *estimated_error_q16_ticks);

#endif /* ACCEL_TIMING_ESTIMATOR_H_ */
