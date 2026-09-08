/*
 * Copyright (c) 2025 The Ohio State University SENSE Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>
#include <zephyr/../../drivers/sensor/ti/bq274xx/bq274xx.h>
#include "batterymonitordt.h"
#include "BLEService.h"

LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_INF);

int battery_level = 100;

/* Flags() bits used by the application. */
#define BQ27441_FLAG_ITPOR (1U << 8)

enum battery_power_flow {
	BATTERY_POWER_FLOW_UNKNOWN,
	BATTERY_POWER_FLOW_CHARGING,
	BATTERY_POWER_FLOW_DISCHARGING,
	BATTERY_POWER_FLOW_IDLE,
};

static bool gauge_reset_warning_reported;
static enum battery_power_flow last_power_flow = BATTERY_POWER_FLOW_UNKNOWN;

static const char *battery_power_flow_name(enum battery_power_flow flow)
{
	switch (flow) {
	case BATTERY_POWER_FLOW_CHARGING:
		return "charging";
	case BATTERY_POWER_FLOW_DISCHARGING:
		return "discharging";
	case BATTERY_POWER_FLOW_IDLE:
		return "idle";
	default:
		return "unknown";
	}
}

/* Copied from the Zephyr BQ274xx driver because that helper has static scope. */
static int bq274xx_cmd_reg_read(const struct device *dev, uint8_t reg_addr,
						 int16_t *val)
{
	const struct bq274xx_config *config = dev->config;
	uint8_t i2c_data[2];
	int ret;

	ret = i2c_burst_read_dt(&config->i2c, reg_addr, i2c_data, sizeof(i2c_data));
	if (ret < 0) {
		LOG_ERR("Unable to read fuel-gauge register 0x%02x: %d", reg_addr, ret);
		return -EIO;
	}

	*val = sys_get_le16(i2c_data);
	return 0;
}

static int bq274xx_read_flags(const struct device *dev, uint16_t *flags)
{
	int16_t raw_flags;
	int ret;

	ret = bq274xx_cmd_reg_read(dev, BQ274XX_COMMAND_FLAGS, &raw_flags);
	if (ret < 0) {
		return ret;
	}

	*flags = (uint16_t)raw_flags;
	return 0;
}

static int bq274xx_read_channel(const struct device *dev, enum sensor_channel channel,
						struct sensor_value *value, const char *name)
{
	int ret;

	ret = sensor_sample_fetch_chan(dev, channel);
	if (ret < 0) {
		LOG_ERR("Unable to fetch %s: %d", name, ret);
		return ret;
	}

	ret = sensor_channel_get(dev, channel, value);
	if (ret < 0) {
		LOG_ERR("Unable to read %s: %d", name, ret);
	}

	return ret;
}

void dt_update_battery(const struct device *dev, bool log_summary)
{
	uint16_t flags_value;
	bool estimate_may_be_unreliable;
	bool current_battery_charging;
	int64_t average_current_ma;
	enum battery_power_flow current_power_flow;
	struct sensor_value voltage;
	struct sensor_value current;
	struct sensor_value state_of_charge;
	struct sensor_value avg_power;
	struct sensor_value full_charge_capacity;
	struct sensor_value remaining_charge_capacity;
	int status;

	/*
	 * sensor_sample_fetch_chan() retries bq274xx_gauge_configure() when the
	 * driver's previous configuration attempt failed. Do this before testing
	 * ITPOR so the periodic maintenance cycle can recover the gauge.
	 */
	if (bq274xx_read_channel(dev, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage,
					"battery voltage") < 0) {
		return;
	}

	status = bq274xx_read_flags(dev, &flags_value);
	if (status < 0) {
		LOG_ERR("Unable to read fuel-gauge flags: %d", status);
		return;
	}

	/*
	 * A POR restores the gauge's RAM configuration from ROM. Continue to
	 * publish its readings while the configuration is incomplete, but identify
	 * them in the periodic log as potentially unreliable.
	 */
	estimate_may_be_unreliable = (flags_value & BQ27441_FLAG_ITPOR) != 0U;
	if (estimate_may_be_unreliable) {
		if (!gauge_reset_warning_reported) {
			LOG_WRN("Fuel gauge reset (Flags: 0x%04x); battery estimates may be unreliable until configured",
				flags_value);
			gauge_reset_warning_reported = true;
		}
	} else if (gauge_reset_warning_reported) {
		LOG_INF("Fuel-gauge configuration restored");
		gauge_reset_warning_reported = false;
	}

	if (bq274xx_read_channel(dev, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current,
					"average current") < 0 ||
		bq274xx_read_channel(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,
					&state_of_charge, "state of charge") < 0 ||
		bq274xx_read_channel(dev, SENSOR_CHAN_GAUGE_AVG_POWER, &avg_power,
					"average power") < 0) {
		return;
	}

	battery_level = state_of_charge.val1;
	bt_bas_set_battery_level(battery_level);

	/* The BQ27441 specifies positive AveragePower() as charging. */
	current_battery_charging = (avg_power.val1 > 0) ||
		(avg_power.val1 == 0 && avg_power.val2 > 0);
	if (current_battery_charging != battery_charging) {
		battery_charging = current_battery_charging;
		status_reg_ble_notification();
	}

	if (current_battery_charging) {
		current_power_flow = BATTERY_POWER_FLOW_CHARGING;
	} else if (avg_power.val1 < 0 ||
		   (avg_power.val1 == 0 && avg_power.val2 < 0)) {
		current_power_flow = BATTERY_POWER_FLOW_DISCHARGING;
	} else {
		current_power_flow = BATTERY_POWER_FLOW_IDLE;
	}

	if (current_power_flow != last_power_flow) {
		LOG_INF("Battery power flow: %s",
			battery_power_flow_name(current_power_flow));
		last_power_flow = current_power_flow;
	}

	if (!log_summary) {
		return;
	}

	if (bq274xx_read_channel(dev, SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY,
					&full_charge_capacity, "full charge capacity") < 0 ||
		bq274xx_read_channel(dev, SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY,
					&remaining_charge_capacity, "remaining charge capacity") < 0) {
		return;
	}

	average_current_ma = sensor_value_to_milli(&current);
	if (estimate_may_be_unreliable) {
		LOG_WRN("Battery estimate may be unreliable (Flags: 0x%04x; configuration incomplete)",
			flags_value);
	}
	LOG_INF("Battery: %d.%03d V, SoC %d%%, %d mA, %s, %d mW; %d/%d mAh",
		voltage.val1, voltage.val2 / 1000, state_of_charge.val1,
		(int)average_current_ma, battery_power_flow_name(current_power_flow),
		avg_power.val1, remaining_charge_capacity.val1,
		full_charge_capacity.val1);
}
