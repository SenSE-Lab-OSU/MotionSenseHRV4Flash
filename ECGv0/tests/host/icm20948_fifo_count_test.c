#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "icm20948_fifo_count.h"

struct stub_context {
	uint16_t counts[6];
	int errors[6];
	uint16_t waits_us[5];
	size_t count_length;
	size_t read_index;
	size_t wait_count;
};

static int stub_read(void *context, uint16_t *count_bytes)
{
	struct stub_context *stub = context;
	size_t index = stub->read_index++;

	assert(index < stub->count_length);
	*count_bytes = stub->counts[index];
	return stub->errors[index];
}

static void stub_wait(void *context, uint32_t wait_us)
{
	struct stub_context *stub = context;

	assert(stub->wait_count < 5U);
	stub->waits_us[stub->wait_count++] = (uint16_t)wait_us;
}

static int run_sequence(struct stub_context *stub,
			struct icm20948_fifo_count_observation *observation,
			uint16_t *count_bytes)
{
	return icm20948_fifo_count_read_stable(stub_read, stub_wait, stub,
					       count_bytes, observation);
}

int main(void)
{
	struct icm20948_fifo_count_observation observation;
	struct stub_context stub;
	uint16_t count;
	int ret;

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 600U;
	stub.count_length = 1U;
	assert(run_sequence(&stub, &observation, &count) == 0);
	assert((count == 600U) && (stub.wait_count == 0U));

	memset(&stub, 0, sizeof(stub));
	stub.count_length = 1U;
	assert(run_sequence(&stub, &observation, &count) == 0);
	assert((count == 0U) && (stub.wait_count == 0U));

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 601U;
	stub.counts[1] = 602U;
	stub.counts[2] = 606U;
	stub.count_length = 3U;
	assert(run_sequence(&stub, &observation, &count) == 0);
	assert((count == 606U) && (observation.read_count == 3U));
	assert((stub.waits_us[0] == 0U) && (stub.waits_us[1] == 25U));

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 601U;
	stub.counts[1] = 3000U;
	stub.count_length = 2U;
	assert(run_sequence(&stub, &observation, &count) == 0);

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 601U;
	stub.counts[1] = 600U;
	stub.count_length = 2U;
	assert(run_sequence(&stub, &observation, &count) == -ERANGE);

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 4097U;
	stub.count_length = 1U;
	assert(run_sequence(&stub, &observation, &count) == -ERANGE);

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 601U;
	stub.counts[1] = 4097U;
	stub.count_length = 2U;
	assert(run_sequence(&stub, &observation, &count) == -ERANGE);

	memset(&stub, 0, sizeof(stub));
	stub.count_length = 1U;
	stub.errors[0] = -EIO;
	assert(run_sequence(&stub, &observation, &count) == -EIO);

	memset(&stub, 0, sizeof(stub));
	stub.counts[0] = 601U;
	stub.counts[1] = 602U;
	stub.errors[1] = -EIO;
	stub.count_length = 2U;
	assert(run_sequence(&stub, &observation, &count) == -EIO);

	memset(&stub, 0, sizeof(stub));
	for (size_t i = 0U; i < 6U; i++) {
		stub.counts[i] = 601U + (uint16_t)(i * 6U);
	}
	stub.count_length = 6U;
	ret = run_sequence(&stub, &observation, &count);
	assert(ret == -EBADMSG);
	assert(stub.wait_count == 5U);
	assert((stub.waits_us[0] == 0U) && (stub.waits_us[1] == 25U) &&
	       (stub.waits_us[2] == 50U) && (stub.waits_us[3] == 100U) &&
	       (stub.waits_us[4] == 200U));

	puts("icm20948 fifo count tests passed");
	return 0;
}
