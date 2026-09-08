#ifndef IMU_FSYNC_TIMING_H_
#define IMU_FSYNC_TIMING_H_

#include <stdint.h>

#include <zephyr/kernel.h>

struct imu_fsync_edge {
	uint32_t rtc_ticks;
	uint32_t ordinal;
	uint8_t level;
};

int imu_fsync_timing_init(void);
int imu_fsync_timing_start(void);
void imu_fsync_timing_stop(void);
int imu_fsync_timing_take_edge(struct imu_fsync_edge *edge);
uint32_t imu_fsync_timing_edge_count_get(void);
int imu_fsync_timing_wait_for_edge_after(uint32_t ordinal, k_timeout_t timeout);
void imu_fsync_timing_rtc_compare_isr(void);

#endif /* IMU_FSYNC_TIMING_H_ */
