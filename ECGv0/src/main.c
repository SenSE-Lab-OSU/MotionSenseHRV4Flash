/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/atomic.h>
#include <nrfx.h>
#include <nrfx_timer.h>
#include <nrfx_uarte.h>
#include <helpers/nrfx_reset_reason.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include "batterymonitordt.h"
#include "ppgSensor.h"
#include "accelRecorder.h"
#include "imuFsyncTiming.h"
#include "icm20948_accel.h"
#include "common.h"
#include "BLEService.h"
#include "ecgRecorder.h"
#include "device_identity.h"
#include "zephyrfilesystem.h"
#include "msense_msc_media.h"
#if CONFIG_DISK_DRIVER_RAW_NAND
#include "nand_disk.h"
#include "spi_nand.h"
#endif
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <string.h>


LOG_MODULE_REGISTER(main, 3);

BUILD_ASSERT(IS_ENABLED(CONFIG_USB_MASS_STORAGE),
	     "ECGv0 requires legacy USB MSC support");



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 6000
#define BATTERY_LOG_INTERVAL_MAINTENANCE_CYCLES 4U

/* The devicetree node identifier for the "led0" alias. */
#define LED_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define PPG_POWER_NODE DT_ALIAS(led2) 
#define BUTTON0_NODE DT_ALIAS(user_button)
#define ECG_NODE DT_ALIAS(ecg)
#define ECG_BUS_NODE DT_ALIAS(ecg_bus)
#define BATTERY_GAUGE_NODE DT_ALIAS(battery_gauge)
#define BATTERY_BUS_NODE DT_ALIAS(battery_bus)
#define GPIO0_NODE DT_ALIAS(gpio0)
#define GPIO1_NODE DT_ALIAS(gpio1)
#define BUTTON0_LONG_PRESS_MS 5000

enum ship_mode_state {
  SHIP_MODE_ACTIVE,
  SHIP_MODE_WAITING_FOR_BUTTON,
  SHIP_MODE_STARTING,
};

enum ecg_storage_action {
	ECG_STORAGE_ACTION_NONE = 0,
	ECG_STORAGE_ACTION_REBOOT,
	ECG_STORAGE_ACTION_ERASE,
	ECG_STORAGE_ACTION_RESET_BAD_BLOCKS,
	ECG_STORAGE_ACTION_TEST_FILES_100,
	ECG_STORAGE_ACTION_TEST_FILES_500,
};

// define our red and green leds
#define LED_PIN DT_GPIO_PIN(LED_NODE, gpios)
#define LED1_PIN DT_GPIO_PIN(LED1_NODE, gpios)

#define LED_FLAGS DT_GPIO_FLAGS(LED_NODE, gpios)



#define PPG_POWER_PIN DT_GPIO_PIN(PPG_POWER_NODE, gpios)
#define PPG_POWER_FLAGS DT_GPIO_FLAGS(PPG_POWER_NODE, gpios)

const struct device* gpio0_device;
const struct device* gpio1_device;

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button0_callback;
/*
 * True means the legacy USB/MSC instance may still be active.  It is cleared
 * only after usb_disable() confirms containment, so fault handling reasserts
 * MSC medium absence whenever this remains true.
 */
static bool usb_enabled;
static bool filesystem_workqueue_started;
static atomic_t msc_ownership_faulted = ATOMIC_INIT(0);
/* Runtime requests stay closed until the boot UUID ownership transaction ends. */
static atomic_t ecg_storage_runtime_ready = ATOMIC_INIT(0);
static bool button0_pressed;
static int64_t button0_pressed_time_ms;
K_MUTEX_DEFINE(collection_mode_lock);
static K_SEM_DEFINE(ecg_collection_transition_sem, 0, 1);
static atomic_t ecg_collection_transition_requested = ATOMIC_INIT(0);
static atomic_t ecg_storage_action_requested = ATOMIC_INIT(ECG_STORAGE_ACTION_NONE);
static atomic_t ship_mode = ATOMIC_INIT(SHIP_MODE_WAITING_FOR_BUTTON);
static K_SEM_DEFINE(ship_mode_exit_sem, 0, 1);

static void button0_work_handler(struct k_work *work);
static void button0_pressed_handler(const struct device *port,
                                    struct gpio_callback *cb,
                                    uint32_t pins);
int enter_ecg_collection_mode(void);
int exit_ecg_collection_mode(void);
void storage_clear_led(void);
static void ecg_storage_transition_fault(const char *operation, int error);
static int execute_ecg_storage_action(enum ecg_storage_action action);
static void ecg_collection_transition_thread(void *arg1, void *arg2,
					     void *arg3);

K_WORK_DEFINE(button0_work, button0_work_handler);
K_THREAD_DEFINE(ecg_collection_transition_thread_id, 2048,
		ecg_collection_transition_thread, NULL, NULL, NULL, 7, 0, 0);


/* SPI Definitions */


/*
------------------------------------------------------------------------------------
SPI Mode    CPOL 	CPHA 	Clock Polarity  Clock Phase Used to
                                in Idle State 	Sample and/or Shift the Data
------------------------------------------------------------------------------------
0               0         0 	Logic low 	Data sampled on rising edge and
                                                shifted out on the falling edge
1               0         1 	Logic low 	Data sampled on the falling edge and
                                                shifted out on the rising edge
2               1         1 	Logic high 	Data sampled on the falling edge and
                                                shifted out on the rising edge
3               1         0 	Logic high 	Data sampled on the rising edge and
                                                shifted out on the falling edge
-------------------------------------------------------------------------------------
*/
// SPI Mode-3 PPG
struct spi_config spi_cfg_ppg = {
    .frequency = DT_PROP(ECG_NODE, spi_max_frequency),
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = DT_REG_ADDR(ECG_NODE),
    .cs = SPI_CS_CONTROL_INIT(ECG_NODE, 0),
};




const struct device *spi_dev_ppg;
const struct device *i2c_dev;





#define DIS_FW_REV_STR CONFIG_BT_DIS_FW_REV_STR
#define DIS_FW_REV_STR_LEN (sizeof(DIS_FW_REV_STR))

#define DIS_HW_REV_STR CONFIG_BT_DIS_HW_REV_STR
#define DIS_HW_REV_STR_LEN (sizeof(DIS_HW_REV_STR))

#define DIS_MANUF CONFIG_BT_DIS_MANUF
#define DIS_MANUF_LEN (sizeof(DIS_MANUF))

#define DIS_MODEL CONFIG_BT_DIS_MODEL
#define DIS_MODEL_LEN (sizeof(DIS_MODEL))


static K_SEM_DEFINE(ble_init_ok, 0, 1);

uint32_t global_counter;

#define AD_FIELD_OVERHEAD 2U
#define AD_FLAGS_ENCODED_LEN (AD_FIELD_OVERHEAD + 1U)
#define AD_SERVICE_DATA_LEN (BT_UUID_SIZE_128 + MSENSE_DEVICE_ID_LEN)
#define AD_SERVICE_DATA_ENCODED_LEN (AD_FIELD_OVERHEAD + AD_SERVICE_DATA_LEN)
#define SCAN_RESPONSE_NAME_ENCODED_LEN (AD_FIELD_OVERHEAD + MSENSE_BLE_NAME_LEN)

static uint8_t advertising_service_data[AD_SERVICE_DATA_LEN] = {
    CONTROL_SERVICE_UUID,
};

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_SVC_DATA128, advertising_service_data,
            sizeof(advertising_service_data)),
};

static struct bt_data sd[] = {
    {
        .type = BT_DATA_NAME_COMPLETE,
        .data = NULL,
        .data_len = 0U,
    },
};

BUILD_ASSERT(MSENSE_DEVICE_ID_LEN == 8U, "Device ID must be 64 bits");
BUILD_ASSERT(MSENSE_BLE_NAME_LEN == 16U, "BLE name length changed unexpectedly");
BUILD_ASSERT(sizeof(advertising_service_data) == 24U,
             "Service data must contain UUID and device ID");
BUILD_ASSERT(AD_FLAGS_ENCODED_LEN + AD_SERVICE_DATA_ENCODED_LEN <=
             BT_GAP_ADV_MAX_ADV_DATA_LEN,
             "Primary advertising data exceeds the legacy limit");
BUILD_ASSERT(SCAN_RESPONSE_NAME_ENCODED_LEN <= BT_GAP_ADV_MAX_ADV_DATA_LEN,
             "Scan response name exceeds the legacy limit");

static int shell_request_ecg_reboot(const struct shell *shell, size_t argc,
                                    char **argv)
{
  ARG_UNUSED(shell);
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  return request_ecg_storage_reboot();
}

static int shell_request_ecg_full_reset(const struct shell *shell, size_t argc,
                                        char **argv)
{
  ARG_UNUSED(shell);
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  return request_ecg_storage_reset(false);
}

// Shell commands queue non-normal storage work on the ECG transition owner.
SHELL_CMD_REGISTER(reset, NULL, "Queues a safe device reset", shell_request_ecg_reboot);
SHELL_CMD_REGISTER(full_reset, NULL, "Queues a safe storage erase and reset",
                   shell_request_ecg_full_reset);





// Setting up the device information service
static int settings_runtime_load(void)
{

/*
  settings_runtime_set("bt/dis/model",
                       DIS_MODEL, DIS_MODEL_LEN);
  settings_runtime_set("bt/dis/manuf",
                       DIS_MANUF, DIS_MANUF_LEN);
*/
  return 0;
}


void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param){
    
  LOG_INF("USB Status: %d", status);

}

static int enable_usb_msc_host_media(void)
{
	int claim_ret;
	int disable_ret;
	int ret;

	ret = usb_enable(usb_status_cb);
	if (ret != 0) {
		/* A nonzero result, including -EALREADY, leaves USB state unproven. */
		usb_enabled = true;
		disable_ret = usb_disable();
		if (disable_ret == 0) {
			usb_enabled = false;
		} else {
			LOG_ERR("USB disable after enable failure returned %d", disable_ret);
		}
		return ret;
	}
	usb_enabled = true;

	ret = msense_msc_media_initialize_absent();
	if (ret != 0) {
		LOG_ERR("MSC safe initialization failed: %d", ret);
		claim_ret = msense_msc_media_claim_for_firmware();
		if (claim_ret != 0) {
			LOG_ERR("MSC absence recovery failed: %d", claim_ret);
		}
		goto disable_usb;
	}

	ret = msense_msc_media_publish_to_host();
	if (ret == 0) {
		return 0;
	}

	LOG_ERR("MSC host publication failed: %d", ret);
	claim_ret = msense_msc_media_claim_for_firmware();
	if (claim_ret != 0) {
		LOG_ERR("MSC absence recovery failed: %d", claim_ret);
	}

disable_usb:
	/* Whole-device USB disable is boot-failure containment only. */
	disable_ret = usb_disable();
	if (disable_ret == 0) {
		usb_enabled = false;
	} else {
		LOG_ERR("USB disable after MSC failure returned %d", disable_ret);
	}
	return ret;
}

static int publish_msc_host_media(void)
{
	int claim_ret;
	int ret;

	if (atomic_get(&msc_ownership_faulted) != 0) {
		return -EIO;
	}
	if (filesystem_is_mounted()) {
		return -EBUSY;
	}

	ret = msense_msc_media_publish_to_host();
	if (ret == 0) {
		return 0;
	}

	LOG_ERR("MSC host publication failed: %d", ret);
	claim_ret = msense_msc_media_claim_for_firmware();
	if (claim_ret == 0) {
		return ret;
	}
	LOG_ERR("MSC absence recovery failed: %d", claim_ret);
	return ret;
}

static void set_firmware_disk_read_only(void)
{
#if CONFIG_DISK_DRIVER_RAW_NAND
	set_read_only(true);
#endif
}

static void set_firmware_disk_writable(void)
{
#if CONFIG_DISK_DRIVER_RAW_NAND
	set_read_only(false);
#endif
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
  struct bt_conn_info info;
  char addr[BT_ADDR_LE_STR_LEN];

  if (bt_conn_get_info(conn, &info))
    printk("Could not parse connection info\n");
  else
  {
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Connection parameters updated!	\n\
      Connected to: %s						\n\
      New Connection Interval: %u				\n\
      New Slave Latency: %u					\n\
      New Connection Supervisory Timeout: %u	\n",
           addr, info.le.interval, info.le.latency, info.le.timeout);
  }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    //.le_param_req = le_param_req,
    .le_param_updated = le_param_updated};

static void bt_ready(int err)
{
  if (err)
  {
    LOG_ERR("BLE initialization callback failed: %d", err);
    return;
  }
  else
    LOG_INF("BLE initialized");

  if (atomic_get(&msc_ownership_faulted) != 0) {
    LOG_ERR("Storage ownership fault; BLE advertising remains disabled");
    return;
  }

  #if CONFIG_BT_SETTINGS
    settings_load();
  #endif


  // Configure connection callbacks
  bt_conn_cb_register(&conn_callbacks);

  err = bt_set_name(msense_device_identity_name());
  if (err) {
    LOG_ERR("Unable to set generated BLE name: %d", err);
    return;
  }

  memcpy(&advertising_service_data[BT_UUID_SIZE_128],
         msense_device_identity_bytes(), MSENSE_DEVICE_ID_LEN);
  sd[0].data = msense_device_identity_name();
  sd[0].data_len = MSENSE_BLE_NAME_LEN;

  // Boot storage ownership is complete before Bluetooth is initialized.
  const struct bt_le_adv_param v = {
      .id = BT_ID_DEFAULT,
      .sid = 0,
      .secondary_max_skip = 0,
      .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
      .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
      .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
      .peer = NULL};

  // if this doesn't work we can use
  /* 
  static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONNECTABLE|BT_LE_ADV_OPT_USE_IDENTITY), 
                800, //Min Advertising Interval 500ms (800*0.625ms) 
                801, //Max Advertising Interval 500.625ms (801*0.625ms)
                NULL); // Set to NULL for undirected advertising
  */
  err = bt_le_adv_start(&v, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("BLE advertising failed to start: %d", err);
    ecg_storage_transition_fault("BLE advertising start", err);
    return;
  }

  atomic_set(&ecg_storage_runtime_ready, 1);
  k_sem_give(&ble_init_ok);
  LOG_INF("BLE advertising started");
}

// Initialize BLE
static void ble_init(void)
{
  int err;

  
  err = bt_enable(bt_ready);
  if (err)
  {
    LOG_ERR("Unable to start BLE initialization: %d", err);
  }
}

// Timer handler that periodically executes commands with a period,
// which is defined by the macro-variable TIMER_MS
static void app_spi_init(void)
{
  
  // device_get_binding is used for runtime aquisition of a device object. We can still use it but we have to be carefull to select the right names
  const char *const spiName_ppg = "spi@c000";

  spi_dev_ppg = DEVICE_DT_GET(ECG_BUS_NODE);
  //spi_dev_ppg = device_get_binding(spiName_ppg);

  if (spi_dev_ppg == NULL || !device_is_ready(spi_dev_ppg))
  {
    printk("Could not get %s \n", spiName_ppg);
    return;
  }
  
}

static void i2c_init(void)
{

  printk("The I2C Init started\n");
  i2c_dev = DEVICE_DT_GET(BATTERY_BUS_NODE);
  if (!device_is_ready(i2c_dev))
  {
    printk("Binding failed to i2c.");
    return;
  }
  // Previously we used to set values, but these are now set by device tree.
}



#define WORKQUEUE_PRIORITY 8
#define WORKQUEUE_STACK_SIZE 20048
K_THREAD_STACK_DEFINE(my_stack_area, WORKQUEUE_STACK_SIZE);



void battery_maintenance()
{
  static uint8_t maintenance_cycles;
  const struct device *const dev = DEVICE_DT_GET(BATTERY_GAUGE_NODE);
  bool log_summary = (maintenance_cycles % BATTERY_LOG_INTERVAL_MAINTENANCE_CYCLES) == 0U;

  maintenance_cycles++;
  dt_update_battery(dev, log_summary);
  
  //battery_lvl = bt_bas_get_battery_level();
  #ifndef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  if (battery_level < 5){
    // if this is our first time
    if (!battery_low){
      battery_low = true;
      LOG_WRN("battery low, turning off file logs and data collection.");
      LOG_INF("logs and data collection will resume once battery is sufficiently charged (>15 percent)");
    }
    
  }
  else if (battery_level > 15){
    if (battery_low){
    LOG_INF("resuming log after battery improved");
    }
    battery_low = false;
    
  } 

  if (collecting_data || host_wants_collection){
        if (battery_low && collecting_data){
            
            request_ecg_collection_mode(false);
        }
        else if (!battery_low && host_wants_collection && !collecting_data){

            request_ecg_collection_mode(true);
            
        }
  }
  #endif
    
}



void blink_led(gpio_pin_t pin){
  if (atomic_get(&ship_mode) != SHIP_MODE_ACTIVE) {
    return;
  }

  gpio_pin_set(gpio0_device, pin, 1);
  k_sleep(K_MSEC(200));
  gpio_pin_set(gpio0_device, pin, 0);
}

static void set_mode_leds(bool led0_on, bool led1_on)
{
  if (atomic_get(&ship_mode) != SHIP_MODE_ACTIVE) {
    return;
  }

  gpio_pin_set(gpio0_device, LED_PIN, led0_on ? 1 : 0);
  gpio_pin_set(gpio0_device, LED1_PIN, led1_on ? 1 : 0);
}

static void blink_collection_mode_pattern(void)
{
  for (int i = 0; i < 3; i++) {
    set_mode_leds(true, false);
    k_sleep(K_MSEC(120));
    set_mode_leds(false, true);
    k_sleep(K_MSEC(120));
  }
  set_mode_leds(false, false);
}

static void blink_usb_mode_pattern(void)
{
  for (int i = 0; i < 2; i++) {
    set_mode_leds(true, true);
    k_sleep(K_MSEC(200));
    set_mode_leds(false, false);
    k_sleep(K_MSEC(200));
  }
}

static void blink_flash_format_pattern(void)
{
  for (int i = 0; i < 8; i++) {
    set_mode_leds(true, true);
    k_sleep(K_MSEC(80));
    set_mode_leds(false, false);
    k_sleep(K_MSEC(80));
  }

  set_mode_leds(true, true);
  k_sleep(K_MSEC(500));
  set_mode_leds(false, false);
}

static void filesystem_workqueue_init(void)
{
  struct k_work_queue_config cfg = {
    .name = "file_sys",
    .no_yield = false
  };

  if (filesystem_workqueue_started) {
    return;
  }

  k_work_queue_init(&my_work_q);
  k_work_queue_start(&my_work_q, my_stack_area,
                     K_THREAD_STACK_SIZEOF(my_stack_area), WORKQUEUE_PRIORITY, &cfg);

  k_work_init(&ppg_work_item.work, work_write);
  ppg_work_item.sensor = ppg;

  k_work_init(&ecg_work_item.work, work_write);
  ecg_work_item.sensor = ecg;

  k_work_init(&log_work_item.work, work_write);
  log_work_item.sensor = customlog;

  filesystem_workqueue_started = true;
}

static uint64_t collection_session_id(void)
{
	uint64_t uptime_ms = (uint64_t)k_uptime_get();

	return (get_current_unix_time() * 1000U) + (uptime_ms % 1000U);
}

static void accel_record_fault_handler(void *context)
{
	ARG_UNUSED(context);
	request_ecg_storage_fault();
}

void request_ecg_storage_fault(void)
{
	/* Safe from producer and log callbacks: only latch, gate, and wake. */
	atomic_set(&msc_ownership_faulted, 1);
	atomic_clear(&ecg_storage_runtime_ready);
	atomic_clear(&ecg_collection_transition_requested);
	k_sem_give(&ecg_collection_transition_sem);
}

void request_ecg_collection_mode(bool enable)
{
	if (atomic_get(&ecg_storage_runtime_ready) == 0) {
		LOG_WRN("Rejecting ECG collection request before storage readiness");
		return;
	}
	if (enable && (atomic_get(&msc_ownership_faulted) != 0 ||
		       atomic_get(&ecg_storage_action_requested) !=
			       ECG_STORAGE_ACTION_NONE)) {
		LOG_ERR("Rejecting ECG collection restart while storage is unavailable");
		return;
	}

	atomic_set(&ecg_collection_transition_requested, enable ? 1 : 0);
	k_sem_give(&ecg_collection_transition_sem);
}

static int request_ecg_storage_action(enum ecg_storage_action action)
{
	if (atomic_get(&ecg_storage_runtime_ready) == 0) {
		return -EAGAIN;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		return -EIO;
	}
	/* Actions require initialized MSC so absence can be established explicitly. */
	if (!usb_enabled || !filesystem_workqueue_started) {
		return -EAGAIN;
	}
	if (collecting_data || host_wants_collection ||
	    atomic_get(&ecg_collection_transition_requested) != 0) {
		return -EBUSY;
	}
	if (!atomic_cas(&ecg_storage_action_requested,
			ECG_STORAGE_ACTION_NONE, action)) {
		return -EBUSY;
	}

	k_sem_give(&ecg_collection_transition_sem);
	return 0;
}

int request_ecg_storage_reboot(void)
{
	return request_ecg_storage_action(ECG_STORAGE_ACTION_REBOOT);
}

int request_ecg_storage_reset(bool reset_bad_blocks)
{
	return request_ecg_storage_action(
		reset_bad_blocks ? ECG_STORAGE_ACTION_RESET_BAD_BLOCKS :
				   ECG_STORAGE_ACTION_ERASE);
}

int request_ecg_manual_test_file(uint8_t command)
{
	enum ecg_storage_action action;

	switch (command) {
	case 130:
		action = ECG_STORAGE_ACTION_TEST_FILES_100;
		break;
	case 150:
		action = ECG_STORAGE_ACTION_TEST_FILES_500;
		break;
	default:
		return -EINVAL;
	}

	return request_ecg_storage_action(action);
}

static void ecg_collection_transition_thread(void *arg1, void *arg2,
					     void *arg3)
{
	enum ecg_storage_action action;
	int ret;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&ecg_collection_transition_sem, K_FOREVER);
		if (atomic_get(&msc_ownership_faulted) == 0 &&
		    atomic_get(&ecg_storage_runtime_ready) == 0) {
			continue;
		}
		action = (enum ecg_storage_action)atomic_get(
			&ecg_storage_action_requested);
		if (action != ECG_STORAGE_ACTION_NONE) {
			ret = execute_ecg_storage_action(action);
			atomic_set(&ecg_storage_action_requested,
				   ECG_STORAGE_ACTION_NONE);
			if (ret != 0) {
				LOG_ERR("ECG non-normal storage action failed: %d", ret);
			}
		}

		if (atomic_get(&msc_ownership_faulted) != 0) {
			if (collecting_data) {
				(void)exit_ecg_collection_mode();
			} else {
				k_mutex_lock(&collection_mode_lock, K_FOREVER);
				ecg_storage_transition_fault("ECG deferred storage fault", -EIO);
				k_mutex_unlock(&collection_mode_lock);
			}
		} else if (atomic_get(&ecg_collection_transition_requested) == 0) {
			(void)exit_ecg_collection_mode();
		} else {
			(void)enter_ecg_collection_mode();
		}
	}
}

static int finalize_ecg_filesystem_for_host(void)
{
	int ret;

	if (!filesystem_workqueue_started) {
		return -ENODEV;
	}

	ret = filesystem_drain_pending_work();
	if (ret != 0) {
		return ret;
	}

	ret = flush_data_buffer(ecg);
	if (ret != 0) {
		return ret;
	}
	ret = flush_data_buffer(customlog);
	if (ret != 0) {
		return ret;
	}

	ret = filesystem_gate_and_drain();
	if (ret != 0) {
		return ret;
	}

	ret = shutdown_filesystem();
	if (ret != 0) {
		return ret;
	}

	return filesystem_is_mounted() ? -EBUSY : 0;
}

static int shutdown_ecg_filesystem_after_start_failure(bool collection_mount_ready)
{
	int ret;

	if (collection_mount_ready) {
		return finalize_ecg_filesystem_for_host();
	}
	if (!filesystem_workqueue_started) {
		return -ENODEV;
	}

	ret = filesystem_gate_and_drain();
	if (ret != 0) {
		return ret;
	}
	if (filesystem_is_mounted()) {
		ret = shutdown_filesystem();
		if (ret != 0) {
			return ret;
		}
	}

	return filesystem_is_mounted() ? -EBUSY : 0;
}

static void ecg_storage_transition_fault(const char *operation, int error)
{
	int claim_ret;
	int gate_ret;

	/*
	 * Reassert the ownership boundary even when the operation that was meant
	 * to establish it returned an error.  A failure here cannot be recovered
	 * by publishing or restarting; the latched fault keeps the collection
	 * state unavailable until an explicit recovery path is established.
	 */
	if (usb_enabled) {
		claim_ret = msense_msc_media_claim_for_firmware();
		if (claim_ret != 0) {
			LOG_ERR("MSC absence recovery returned %d after %s", claim_ret,
				operation);
		}
	}

	ecg_filesystem_log_disable_and_wait();
	if (filesystem_workqueue_started) {
		gate_ret = filesystem_gate_and_drain();
		if (gate_ret != 0) {
			LOG_ERR("Filesystem fault gate returned %d after %s", gate_ret,
				operation);
		}
	}
	set_firmware_disk_read_only();
	atomic_set(&msc_ownership_faulted, 1);
	atomic_clear(&ecg_storage_runtime_ready);
	atomic_clear(&ecg_collection_transition_requested);
	LOG_ERR("%s failed: %d; keeping MSC medium absent", operation, error);
}

/* Called only by the ECG transition owner while collection_mode_lock is held. */
static int teardown_ecg_filesystem_for_storage_action(void)
{
	int ret;

	if (!filesystem_workqueue_started) {
		return -ENODEV;
	}

	ret = filesystem_gate_and_drain();
	if (ret != 0) {
		return ret;
	}
	if (filesystem_is_mounted()) {
		ret = shutdown_filesystem();
		if (ret != 0) {
			return ret;
		}
	}

	return filesystem_is_mounted() ? -EBUSY : 0;
}

/* Called only by the ECG transition owner while collection_mode_lock is held. */
static int prepare_ecg_storage_action(void)
{
	int ret;
	int stop_ret;

	if (collecting_data || host_wants_collection ||
	    atomic_get(&ecg_collection_transition_requested) != 0) {
		return -EBUSY;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		return -EIO;
	}

	reset_lock = true;
	ecg_filesystem_log_disable_and_wait();

	/* Rejoin any stale recorder sessions before changing storage ownership. */
	stop_ret = ecg_recorder_stop();
	ret = accel_recorder_stop();
	if (stop_ret == 0 && ret != 0) {
		stop_ret = ret;
	}
	if (stop_ret != 0) {
		ecg_storage_transition_fault("ECG non-normal producer stop", stop_ret);
		return stop_ret;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ecg_storage_transition_fault("ECG non-normal producer fault", -EIO);
		return -EIO;
	}

	/* The request precondition guarantees that legacy MSC is initialized. */
	ret = msense_msc_media_claim_for_firmware();
	if (ret != 0) {
		ecg_storage_transition_fault("ECG non-normal MSC firmware claim", ret);
		return ret;
	}

	ret = teardown_ecg_filesystem_for_storage_action();
	if (ret != 0) {
		set_firmware_disk_read_only();
		ecg_storage_transition_fault("ECG non-normal storage teardown", ret);
		return ret;
	}

	return 0;
}

/* Called only by the ECG transition owner after prepare_ecg_storage_action(). */
static int reset_ecg_storage_and_reboot(bool reset_bad_blocks)
{
	const struct device *flash_device = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	int ret;

	if (!device_is_ready(flash_device)) {
		ret = -ENODEV;
		goto erase_failed;
	}

	blink_flash_format_pattern();
	set_firmware_disk_writable();
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		goto erase_failed;
	}
#if CONFIG_DISK_DRIVER_RAW_NAND
	if (reset_bad_blocks) {
		LOG_WRN("Erasing bad block table");
		ret = spi_nand_multi_chip_reset_bad_block(flash_device);
	} else {
		LOG_INF("Performing chip erase");
		ret = spi_nand_multi_chip_erase(flash_device);
	}
#else
#if !DT_NODE_HAS_PROP(DT_ALIAS(spi_flash0), size)
#error "flash needs size property in order to be erased"
#endif
	ret = flash_erase(flash_device, 0,
			  DT_PROP(DT_ALIAS(spi_flash0), size) / 8);
#endif
	if (ret != 0) {
		goto erase_failed;
	}

	set_firmware_disk_read_only();
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		goto erase_failed;
	}
	LOG_INF("Storage erase complete; resetting while MSC remains absent");
	NVIC_SystemReset();
	ret = -EIO;

erase_failed:
	set_firmware_disk_read_only();
	ecg_storage_transition_fault("ECG storage erase/reset", ret);
	return ret;
}

/* Called only by the ECG transition owner after prepare_ecg_storage_action(). */
static int reboot_ecg_after_storage_teardown(void)
{
	set_firmware_disk_read_only();
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ecg_storage_transition_fault("ECG reboot", -EIO);
		return -EIO;
	}
	LOG_INF("Resetting while MSC remains absent");
	NVIC_SystemReset();
	ecg_storage_transition_fault("ECG reboot", -EIO);
	return -EIO;
}

/* Called only by the ECG transition owner after prepare_ecg_storage_action(). */
static int run_ecg_manual_test_file_action(enum ecg_storage_action action)
{
	int ret;
	int teardown_ret;

	set_firmware_disk_writable();
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		goto test_failed;
	}
	ret = setup_disk();
	if (ret != 0) {
		if (filesystem_is_mounted()) {
			teardown_ret = teardown_ecg_filesystem_for_storage_action();
			if (teardown_ret != 0) {
				LOG_ERR("Test setup teardown failed: %d", teardown_ret);
			}
		}
		goto test_failed;
	}

	storage_clear_led();
	switch (action) {
	case ECG_STORAGE_ACTION_TEST_FILES_100:
		ret = create_test_files(100);
		break;
	case ECG_STORAGE_ACTION_TEST_FILES_500:
		ret = create_test_files(500);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	/* Always attempt the checked close/unmount sequence after a mounted test. */
	teardown_ret = teardown_ecg_filesystem_for_storage_action();
	if (ret == 0 && teardown_ret != 0) {
		ret = teardown_ret;
	}
	if (ret != 0) {
		goto test_failed;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		goto test_failed;
	}

	set_firmware_disk_read_only();
	ret = publish_msc_host_media();
	if (ret != 0) {
		goto test_failed;
	}

	filesystem_clear_collection_id();
	reset_lock = false;
	collecting_data = false;
	host_wants_collection = false;
	return 0;

test_failed:
	set_firmware_disk_read_only();
	ecg_storage_transition_fault("ECG manual test-file operation", ret);
	return ret;
}

static int execute_ecg_storage_action(enum ecg_storage_action action)
{
	int ret;

	k_mutex_lock(&collection_mode_lock, K_FOREVER);
	if (atomic_get(&ecg_storage_runtime_ready) == 0) {
		ret = -EAGAIN;
		goto out;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		goto out;
	}

	ret = prepare_ecg_storage_action();
	if (ret != 0) {
		goto out;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		ecg_storage_transition_fault("ECG non-normal storage action", ret);
		goto out;
	}

	switch (action) {
	case ECG_STORAGE_ACTION_REBOOT:
		ret = reboot_ecg_after_storage_teardown();
		break;
	case ECG_STORAGE_ACTION_ERASE:
		ret = reset_ecg_storage_and_reboot(false);
		break;
	case ECG_STORAGE_ACTION_RESET_BAD_BLOCKS:
		ret = reset_ecg_storage_and_reboot(true);
		break;
	case ECG_STORAGE_ACTION_TEST_FILES_100:
	case ECG_STORAGE_ACTION_TEST_FILES_500:
		ret = run_ecg_manual_test_file_action(action);
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	k_mutex_unlock(&collection_mode_lock);
	return ret;
}

int enter_ecg_collection_mode(void)
{
	int cleanup_ret;
	int ret;
	int start_ret;
	uint64_t session_id;
	bool accel_started = false;
	bool collection_mount_ready = false;
	bool ecg_start_submitted = false;
	bool ecg_started = false;
	bool fsync_started = false;
	bool icm_started = false;
	bool rtc_started = false;

	k_mutex_lock(&collection_mode_lock, K_FOREVER);
	if (atomic_get(&ecg_storage_runtime_ready) == 0) {
		k_mutex_unlock(&collection_mode_lock);
		return -EAGAIN;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		k_mutex_unlock(&collection_mode_lock);
		return -EIO;
	}
	if (collecting_data) {
		k_mutex_unlock(&collection_mode_lock);
		return 0;
	}

	LOG_INF("Entering ECG collection mode");
	blink_collection_mode_pattern();

	if (usb_enabled) {
		ret = msense_msc_media_claim_for_firmware();
		if (ret != 0) {
			ecg_storage_transition_fault("MSC firmware claim", ret);
			k_mutex_unlock(&collection_mode_lock);
			return ret;
		}
	}

	set_firmware_disk_writable();
	ret = setup_disk();
	if (ret != 0) {
		LOG_ERR("Failed to mount ECG filesystem: %d", ret);
		goto start_failed;
	}
	collection_mount_ready = true;
	session_id = collection_session_id();
	filesystem_set_collection_id(session_id);
	if (!filesystem_workqueue_started) {
		ret = -ENODEV;
		goto start_failed;
	}
	k_work_queue_unplug(&my_work_q);

	ret = rtc0_collection_counter_start();
	if (ret != 0) {
		LOG_ERR("Failed to start RTC0 collection counter: %d", ret);
		goto start_failed;
	}
	rtc_started = true;

	ecg_start_submitted = true;
	ret = ecg_recorder_start();
	if (ret != 0) {
		LOG_ERR("Failed to start ECG recorder: %d", ret);
		goto start_failed;
	}
	ecg_started = true;

	ret = accel_recorder_start(session_id);
	if (ret != 0) {
		LOG_ERR("Failed to start accelerometer recorder: %d", ret);
		goto start_failed;
	}
	accel_started = true;

	ret = icm20948_accel_set_fifo_consumer(accel_recorder_consume_fifo, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to register ICM-20948 FIFO consumer: %d", ret);
		goto start_failed;
	}

	ret = icm20948_accel_start();
	if (ret != 0) {
		LOG_ERR("Failed to start ICM-20948 accelerometer: %d", ret);
		goto start_failed;
	}
	icm_started = true;

	ret = imu_fsync_timing_start();
	if (ret != 0) {
		LOG_ERR("Failed to start IMU FSYNC timing: %d", ret);
		goto start_failed;
	}
	fsync_started = true;

	ret = rtc0_collection_notification_start();
	if (ret != 0) {
		LOG_ERR("Failed to start RTC0 BLE timing notifications: %d", ret);
		goto start_failed;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ret = -EIO;
		goto start_failed;
	}

	collecting_data = true;
	host_wants_collection = true;
	ecg_filesystem_log_enable();
	k_mutex_unlock(&collection_mode_lock);
	return 0;

start_failed:
	start_ret = ret;
	ecg_filesystem_log_disable_and_wait();
	cleanup_ret = 0;
	if (fsync_started) {
		imu_fsync_timing_stop();
	}
	if (icm_started) {
		ret = icm20948_accel_stop();
		if (cleanup_ret == 0 && ret != 0) {
			cleanup_ret = ret;
		}
	}
	if (accel_started) {
		ret = icm20948_accel_set_fifo_consumer(NULL, NULL);
		if (cleanup_ret == 0 && ret != 0) {
			cleanup_ret = ret;
		}
		ret = accel_recorder_stop();
		if (cleanup_ret == 0 && ret != 0) {
			cleanup_ret = ret;
		}
	}
	if (ecg_started) {
		ret = ecg_recorder_stop();
		if (cleanup_ret == 0 && ret != 0) {
			cleanup_ret = ret;
		}
	}
	/*
	 * A failed ecg_recorder_start() may have submitted a start token even
	 * though it never returned success.  Its failure path must consume the
	 * terminal stopped acknowledgement; otherwise no filesystem teardown is
	 * safe.  Do not infer this from ecg_record_active.
	 */
	if (ecg_start_submitted && !ecg_recorder_shutdown_confirmed()) {
		if (cleanup_ret == 0) {
			cleanup_ret = (start_ret != 0) ? start_ret : -EIO;
		}
	}
	if (rtc_started) {
		rtc0_collection_counter_stop();
	}
	if (cleanup_ret == 0) {
		cleanup_ret = shutdown_ecg_filesystem_after_start_failure(
			collection_mount_ready);
	}

	set_firmware_disk_read_only();
	host_wants_collection = false;
	collecting_data = false;
	if (cleanup_ret == 0 && !filesystem_is_mounted()) {
		filesystem_clear_collection_id();
		if (usb_enabled) {
			cleanup_ret = publish_msc_host_media();
		}
	}
	if (cleanup_ret != 0) {
		ecg_storage_transition_fault("ECG collection start unwind", cleanup_ret);
		ret = cleanup_ret;
	} else {
		ret = start_ret;
	}

	k_mutex_unlock(&collection_mode_lock);
	return ret;
}

int exit_ecg_collection_mode(void)
{
	int ret;
	int stop_ret = 0;
	uint32_t fsync_edge_count;

	k_mutex_lock(&collection_mode_lock, K_FOREVER);
	if (!collecting_data) {
		ret = (atomic_get(&msc_ownership_faulted) != 0) ? -EIO : 0;
		k_mutex_unlock(&collection_mode_lock);
		return ret;
	}

	LOG_INF("Leaving ECG collection mode");
	host_wants_collection = false;
	ecg_filesystem_log_disable_and_wait();

	/* Keep the stream alive until its second marker, then stop all producers. */
	while (imu_fsync_timing_edge_count_get() < 2U) {
		fsync_edge_count = imu_fsync_timing_edge_count_get();
		ret = imu_fsync_timing_wait_for_edge_after(fsync_edge_count,
							K_MSEC(500));
		if (ret != 0) {
			LOG_WRN("Timed out waiting for IMU FSYNC startup edges: %d", ret);
			break;
		}
	}
	if (imu_fsync_timing_edge_count_get() >= 2U) {
		k_sleep(K_MSEC(4));
	}

	ret = icm20948_accel_stop();
	if (ret != 0) {
		LOG_WRN("ICM-20948 accelerometer stop returned %d", ret);
		stop_ret = ret;
	}
	imu_fsync_timing_stop();
	ret = icm20948_accel_set_fifo_consumer(NULL, NULL);
	if (ret != 0) {
		LOG_WRN("ICM-20948 FIFO consumer clear returned %d", ret);
		if (stop_ret == 0) {
			stop_ret = ret;
		}
	}

	ret = accel_recorder_stop();
	if (ret != 0) {
		LOG_WRN("Accelerometer recorder stop returned %d", ret);
		if (stop_ret == 0) {
			stop_ret = ret;
		}
	}

	ret = ecg_recorder_stop();
	if (ret != 0) {
		LOG_WRN("ECG recorder stop returned %d", ret);
		if (stop_ret == 0) {
			stop_ret = ret;
		}
	}
	rtc0_collection_counter_stop();

	if (stop_ret != 0) {
		ecg_storage_transition_fault("ECG producer stop", stop_ret);
		k_mutex_unlock(&collection_mode_lock);
		return stop_ret;
	}
	if (atomic_get(&msc_ownership_faulted) != 0) {
		ecg_storage_transition_fault("ECG recorder fault", -EIO);
		k_mutex_unlock(&collection_mode_lock);
		return -EIO;
	}

	ret = finalize_ecg_filesystem_for_host();
	if (ret != 0) {
		ecg_storage_transition_fault("ECG filesystem teardown", ret);
		k_mutex_unlock(&collection_mode_lock);
		return ret;
	}

	set_firmware_disk_read_only();
	if (usb_enabled) {
		ret = publish_msc_host_media();
		if (ret != 0) {
			ecg_storage_transition_fault("ECG MSC publication", ret);
			k_mutex_unlock(&collection_mode_lock);
			return ret;
		}
	}

	filesystem_clear_collection_id();
	collecting_data = false;
	blink_usb_mode_pattern();
	k_mutex_unlock(&collection_mode_lock);
	return 0;
}

static void button0_work_handler(struct k_work *work)
{
  int button_state;
  int ret;
  int64_t pressed_duration_ms;

  ARG_UNUSED(work);

  button_state = gpio_pin_get_dt(&button0);
  if (button_state < 0) {
    LOG_WRN("button0 read failed: %d", button_state);
    return;
  }

  if (button_state > 0) {
    button0_pressed = true;
    button0_pressed_time_ms = k_uptime_get();
    return;
  }

  if (!button0_pressed) {
    return;
  }

  button0_pressed = false;
  pressed_duration_ms = k_uptime_get() - button0_pressed_time_ms;

  if (atomic_get(&ship_mode) != SHIP_MODE_ACTIVE) {
    if (atomic_cas(&ship_mode, SHIP_MODE_WAITING_FOR_BUTTON,
                   SHIP_MODE_STARTING)) {
      LOG_INF("Exiting ship mode");
      k_sem_give(&ship_mode_exit_sem);
    }
    return;
  }

  if (pressed_duration_ms >= BUTTON0_LONG_PRESS_MS) {
    ret = request_ecg_storage_reset(false);
    if (ret != 0) {
      LOG_ERR("Long-press storage erase request rejected: %d", ret);
    } else {
      LOG_WRN("Long-press storage erase queued through ECG transition owner");
    }
    return;
  }

  if (collecting_data) {
    request_ecg_collection_mode(false);
  } else {
    request_ecg_collection_mode(true);
  }
}

static void button0_pressed_handler(const struct device *port,
                                    struct gpio_callback *cb,
                                    uint32_t pins)
{
  ARG_UNUSED(port);
  ARG_UNUSED(cb);
  ARG_UNUSED(pins);

  (void)k_work_submit(&button0_work);
}

static int button0_init(void)
{
  int ret;

  if (!gpio_is_ready_dt(&button0)) {
    LOG_ERR("button0 GPIO is not ready");
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
  if (ret != 0) {
    return ret;
  }

  gpio_init_callback(&button0_callback, button0_pressed_handler,
                     BIT(button0.pin));

  ret = gpio_add_callback(button0.port, &button0_callback);
  if (ret != 0) {
    return ret;
  }

  return gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
}



void storage_clear_led(){
  if (atomic_get(&ship_mode) != SHIP_MODE_ACTIVE) {
    return;
  }

  gpio_pin_set(gpio0_device, LED1_PIN, 1);
}

int main(void)
{
  int ret;
  int identity_err;
	int uuid_ret = 0;
	bool uuid_write_ready = false;
  uint32_t reset_reason = nrfx_reset_reason_get();

  /* RESETREAS is cumulative until acknowledged. Capture this boot's reason
   * before clearing it, so the next boot is not reported with stale flags. */
  nrfx_reset_reason_clear(reset_reason);

  printk("Starting Application... \n");
  LOG_WRN("Boot reset reason: 0x%08x", (unsigned int)reset_reason);
  LOG_INF("Starting Logging...");

  ret = button0_init();
  if (ret != 0)
  {
    LOG_ERR("Failed to initialize button0: %d", ret);
    return ret;
  }

  identity_err = msense_device_identity_init();
  if (identity_err) {
    LOG_ERR("Unable to initialize factory device identity: %d", identity_err);
  }
  

  // Setup our Flash Filesystem
  ret = setup_disk();
  if (ret != 0) {
    LOG_ERR("Startup filesystem setup failed: %d", ret);
    set_firmware_disk_read_only();
    return ret;
  }
  filesystem_workqueue_init();

	if (identity_err == 0) {
		uuid_ret = write_device_info_file(msense_device_identity_name(),
						  msense_device_identity_hex());
		if (uuid_ret == 0) {
			uuid_write_ready = true;
		} else {
			LOG_ERR("Unable to write boot uuid.txt: %d", uuid_ret);
		}
	}

  k_sleep(K_SECONDS(1));

  #ifdef CONFIG_MSENSE_USB_SECURITY
    security_lock = true;
  #endif

  ret = filesystem_gate_and_drain();
  if (ret == 0) {
    ret = shutdown_filesystem();
  }
  set_firmware_disk_read_only();
  if (ret != 0 || filesystem_is_mounted()) {
    if (ret == 0) {
      ret = -EBUSY;
    }
    ecg_storage_transition_fault("Startup filesystem teardown", ret);
    LOG_ERR("Startup filesystem teardown failed: %d", ret);
  } else if (identity_err != 0) {
    LOG_ERR("Factory identity is unavailable; keeping MSC medium absent");
  } else if (!uuid_write_ready) {
    ecg_storage_transition_fault("Startup uuid.txt write", uuid_ret);
  } else if (atomic_get(&msc_ownership_faulted) != 0) {
    LOG_ERR("Startup storage ownership fault; keeping MSC medium absent");
  } else if (!security_lock) {
    ret = enable_usb_msc_host_media();
    if (ret != 0) {
      ecg_storage_transition_fault("Startup USB MSC publication", ret);
      LOG_ERR("Failed to publish USB MSC medium: %d", ret);
    }
  }

  k_sleep(K_SECONDS(2));
  

  gpio0_device = DEVICE_DT_GET(GPIO0_NODE);
  gpio1_device = DEVICE_DT_GET(GPIO1_NODE);
  
  // Initialize our 2 LED pins and 5V PPG Power Pin
  ret = gpio_pin_configure(gpio0_device, LED_PIN, GPIO_OUTPUT_INACTIVE | LED_FLAGS);
  ret = gpio_pin_configure(gpio0_device, LED1_PIN, GPIO_OUTPUT_INACTIVE | LED_FLAGS);
  ret = gpio_pin_configure(gpio1_device, PPG_POWER_PIN, GPIO_OUTPUT_ACTIVE | PPG_POWER_FLAGS);
  ret = icm20948_accel_init();
  if (ret != 0)
  {
    LOG_ERR("ICM-20948 accelerometer initialization failed: %d", ret);
  }
  ret = imu_fsync_timing_init();
  if (ret != 0)
  {
    LOG_ERR("IMU FSYNC timing initialization failed: %d", ret);
  }
  accel_recorder_set_fault_handler(accel_record_fault_handler, NULL);
  ecg_recorder_set_fault_handler(accel_record_fault_handler, NULL);
  
  // Init, verify ID and config sensors
  
  //app_spi_init();
  //spi_verify_sensor_ids();

  //i2c_init();

  // Shutdown our PPG sensor until we need to get data from it
  //ppg_sleep();

  LOG_INF("Ship mode active; press button0 to enable BLE and status LEDs");
  ret = k_sem_take(&ship_mode_exit_sem, K_FOREVER);
  if (ret != 0) {
    LOG_ERR("Failed to exit ship mode: %d", ret);
    return ret;
  }

  if (identity_err == 0) {
    ble_init();
  } else {
    LOG_ERR("BLE advertising disabled because device identity is unavailable");
  }
  atomic_set(&ship_mode, SHIP_MODE_ACTIVE);

  //get_storage_percent_full();
  
  // we set global update at 9 so that when we are entering the while loop, we will check the storage & battery.
  int global_update = 9;
  int update_time = SLEEP_TIME_MS;
  
  collecting_data = false;
  host_wants_collection = false;

  while (1)
  {
    
    //printk("%d %d\n", connectedFlag, collecting_data);

    global_update++;
    if (global_update >= 100){
      global_update = 0;
    }

    if (global_update % 5 == 0){
      battery_maintenance();
      //get_current_unix_time();
    }

    if (!connectedFlag){
    // blink the LED while we aren't connected.
      blink_led(LED_PIN);
    }
    else {
      // When Connected, LED instead only blinks once every 10 cycles
      if (global_update % 10 == 0){
        blink_led(LED_PIN);
      }
    }
      
    
    if (collecting_data){
        blink_led(LED1_PIN);
    }

    k_msleep(update_time);
    
  }
  return 0;
}
