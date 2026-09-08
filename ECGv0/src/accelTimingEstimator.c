#include "accelTimingEstimator.h"

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 512 / (562.5 * 1.10) and 512 / (562.5 * 0.90), respectively, in
 * RTC ticks per sample. Keeping these as rational values avoids a floating
 * point dependency in the FIFO consumer.
 */
#define ACCEL_TIMING_PERIOD_MIN_NUMERATOR 2048ULL
#define ACCEL_TIMING_PERIOD_MIN_DENOMINATOR 2475ULL
#define ACCEL_TIMING_PERIOD_MAX_NUMERATOR 2048ULL
#define ACCEL_TIMING_PERIOD_MAX_DENOMINATOR 2025ULL

#define ACCEL_TIMING_Q32_ONE (1ULL << 32)
#define ACCEL_TIMING_Q32_HALF (1ULL << 31)

static uint64_t accel_timing_period_bound_q32(uint64_t numerator,
					       uint64_t denominator)
{
	return (numerator << 32) / denominator;
}

static int64_t accel_timing_round_q32(int64_t value)
{
	if (value >= 0) {
		return (value + (int64_t)ACCEL_TIMING_Q32_HALF) >> 32;
	}

	return -(((-value) + (int64_t)ACCEL_TIMING_Q32_HALF) >> 32);
}

void accel_timing_estimator_reset(struct accel_timing_estimator *estimator)
{
	if (estimator != NULL) {
		memset(estimator, 0, sizeof(*estimator));
	}
}

int accel_timing_estimator_observe(
	struct accel_timing_estimator *estimator,
	const struct accel_timing_observation *observation)
{
	const struct accel_timing_observation *oldest;
	const struct accel_timing_observation *newest;
	uint64_t sample_delta;
	uint32_t tick_delta;
	uint64_t period_q32;

	if ((estimator == NULL) || (observation == NULL)) {
		return -EINVAL;
	}

	if ((estimator->count != 0U) &&
	    (observation->sample_sequence <=
	     estimator->observations[estimator->count - 1U].sample_sequence)) {
		return -EINVAL;
	}

	if (estimator->count == ACCEL_TIMING_ESTIMATOR_WINDOW) {
		memmove(&estimator->observations[0], &estimator->observations[1],
			(sizeof(estimator->observations[0]) *
			 (ACCEL_TIMING_ESTIMATOR_WINDOW - 1U)));
		estimator->count--;
	}

	estimator->observations[estimator->count++] = *observation;
	if (estimator->count < 2U) {
		return 0;
	}

	oldest = &estimator->observations[0];
	newest = &estimator->observations[estimator->count - 1U];
	sample_delta = newest->sample_sequence - oldest->sample_sequence;
	if (sample_delta == 0U) {
		return -EINVAL;
	}

	tick_delta = newest->rtc_ticks - oldest->rtc_ticks;
	period_q32 = ((uint64_t)tick_delta << 32) / sample_delta;
	if ((period_q32 < accel_timing_period_bound_q32(
				      ACCEL_TIMING_PERIOD_MIN_NUMERATOR,
				      ACCEL_TIMING_PERIOD_MIN_DENOMINATOR)) ||
	    (period_q32 > accel_timing_period_bound_q32(
				      ACCEL_TIMING_PERIOD_MAX_NUMERATOR,
				      ACCEL_TIMING_PERIOD_MAX_DENOMINATOR))) {
		return -ERANGE;
	}

	estimator->period_q32 = period_q32;
	return 0;
}

bool accel_timing_estimator_ready(const struct accel_timing_estimator *estimator)
{
	return (estimator != NULL) && (estimator->count >= 2U) &&
	       (estimator->period_q32 != 0U);
}

int accel_timing_estimator_estimate(
	const struct accel_timing_estimator *estimator, uint64_t sample_sequence,
	uint32_t *estimated_rtc_ticks, uint32_t *estimated_error_q16_ticks)
{
	const struct accel_timing_observation *reference = NULL;
	uint64_t closest_distance_x2 = UINT64_MAX;
	uint64_t window_samples;
	uint64_t period_max_q32;
	uint64_t error_q32;
	int64_t target_x2;
	int64_t reference_x2;
	int64_t delta_x2;
	int64_t offset_q32;
	int64_t rounded_offset_ticks;

	if ((estimated_rtc_ticks == NULL) || !accel_timing_estimator_ready(estimator)) {
		return -ENODATA;
	}
	if (sample_sequence > ((uint64_t)INT64_MAX / 2U)) {
		return -ERANGE;
	}

	target_x2 = (int64_t)(sample_sequence * 2U);
	for (uint8_t i = 0U; i < estimator->count; i++) {
		int64_t observation_x2;
		int64_t distance_x2;
		uint64_t absolute_distance_x2;

		if (estimator->observations[i].sample_sequence >
		    ((uint64_t)INT64_MAX / 2U)) {
			return -ERANGE;
		}
		observation_x2 =
			(int64_t)(estimator->observations[i].sample_sequence * 2U) - 1;
		distance_x2 = target_x2 - observation_x2;
		absolute_distance_x2 = (distance_x2 < 0) ?
			(uint64_t)(-distance_x2) : (uint64_t)distance_x2;
		if (absolute_distance_x2 < closest_distance_x2) {
			closest_distance_x2 = absolute_distance_x2;
			reference = &estimator->observations[i];
		}
	}

	if (reference == NULL) {
		return -ENODATA;
	}

	reference_x2 = (int64_t)(reference->sample_sequence * 2U) - 1;
	delta_x2 = target_x2 - reference_x2;
	offset_q32 = (delta_x2 * (int64_t)estimator->period_q32) / 2;
	rounded_offset_ticks = accel_timing_round_q32(offset_q32);
	*estimated_rtc_ticks = reference->rtc_ticks + (uint32_t)rounded_offset_ticks;

	if (estimated_error_q16_ticks != NULL) {
		window_samples = estimator->observations[estimator->count - 1U].sample_sequence -
			estimator->observations[0].sample_sequence;
		period_max_q32 = accel_timing_period_bound_q32(
			ACCEL_TIMING_PERIOD_MAX_NUMERATOR,
			ACCEL_TIMING_PERIOD_MAX_DENOMINATOR);
		/* Half a sample phase, endpoint-period uncertainty, and tick rounding. */
		error_q32 = (period_max_q32 / 2U) + ACCEL_TIMING_Q32_HALF;
		if (window_samples != 0U) {
			error_q32 += (closest_distance_x2 * period_max_q32) /
				(2U * window_samples);
		}
		*estimated_error_q16_ticks = (uint32_t)(error_q32 >> 16);
	}

	return 0;
}
