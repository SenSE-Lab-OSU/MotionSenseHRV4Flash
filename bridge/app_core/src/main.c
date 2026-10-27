#include <zephyr/kernel.h>

int main(void)
{
	/* Keep RAM and GPIO ownership for future Bridge board services. */
	k_sleep(K_FOREVER);
	return 0;
}
