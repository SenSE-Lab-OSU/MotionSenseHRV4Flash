/*
 * ECGv0 retains these BLE-visible configuration values for compatibility with
 * its existing GATT contract. They do not initialize or access MAX86141
 * hardware; the PPG acquisition implementation remains unlinked from ECGv0.
 */

#include "ppgSensor.h"

struct ppg_configData ppgConfig = {
	.isEnabled = true,
	.sample_avg = PPG_FIXED_SAMPLE_AVG,
	.green_intensity = 0x30,
	.infraRed_intensity = 0x12,
	.sampling_time = 0x28,
	.numCounts = PPG_FIXED_NUM_COUNTS,
	.txPacketEnable = false,
};

bool use_fixed_ppg_brightness = false;
