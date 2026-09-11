#include "icm20948_fifo_count.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>

int icm20948_fifo_count_read_stable(icm20948_fifo_count_read_t read_count,
				    icm20948_fifo_count_wait_t wait,
				    void *context, uint16_t *count_bytes,
				    struct icm20948_fifo_count_observation *observation)
{
	static const uint16_t retry_waits_us[] = {0U, 25U, 50U, 100U, 200U};
	uint16_t previous_count;
	int ret;

	if ((read_count == NULL) || (wait == NULL) || (count_bytes == NULL) ||
	    (observation == NULL)) {
		return -EINVAL;
	}

	memset(observation, 0, sizeof(*observation));
	/* COUNTH makes the two count bytes coherent, not necessarily sample-aligned. */
	ret = read_count(context, count_bytes);
	observation->read_count = 1U;
	observation->read_error = ret;
	if (ret != 0) {
		return ret;
	}

	observation->initial_count_bytes = *count_bytes;
	observation->last_count_bytes = *count_bytes;
	if (*count_bytes == 0U) {
		return 0;
	}
	if (*count_bytes > ICM20948_FIFO_COUNT_CAPACITY_BYTES) {
		return -ERANGE;
	}
	if ((*count_bytes % ICM20948_FIFO_COUNT_SAMPLE_BYTES) == 0U) {
		return 0;
	}

	previous_count = *count_bytes;
	for (size_t i = 0U; i < sizeof(retry_waits_us) / sizeof(retry_waits_us[0]); i++) {
		wait(context, retry_waits_us[i]);
		ret = read_count(context, count_bytes);
		observation->read_count++;
		observation->read_error = ret;
		if (ret != 0) {
			return ret;
		}

		observation->last_count_bytes = *count_bytes;
		if ((*count_bytes > ICM20948_FIFO_COUNT_CAPACITY_BYTES) ||
		    (*count_bytes < previous_count)) {
			return -ERANGE;
		}
		if ((*count_bytes != 0U) &&
		    ((*count_bytes % ICM20948_FIFO_COUNT_SAMPLE_BYTES) == 0U)) {
			return 0;
		}
		previous_count = *count_bytes;
	}

	return -EBADMSG;
}
