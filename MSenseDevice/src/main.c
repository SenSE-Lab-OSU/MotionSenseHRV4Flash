/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <nrfx.h>
#include <nrfx_timer.h>
#include <nrfx_uarte.h>
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
#if CONFIG_DISK_DRIVER_RAW_NAND
#include "drivers/nand/nand_disk.h"
#endif
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <string.h>


LOG_MODULE_REGISTER(main, 3);



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 6000

/* The devicetree node identifier for the "led0" alias. */
#define LED_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define PPG_POWER_NODE DT_ALIAS(led2) 
#define BUTTON0_NODE DT_NODELABEL(button0)
#define BUTTON0_LONG_PRESS_MS 5000

enum ship_mode_state {
  SHIP_MODE_ACTIVE,
  SHIP_MODE_WAITING_FOR_BUTTON,
  SHIP_MODE_STARTING,
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
static bool usb_enabled;
static bool filesystem_workqueue_started;
static bool button0_pressed;
static int64_t button0_pressed_time_ms;
static struct k_mutex collection_mode_lock;
static K_SEM_DEFINE(accel_record_fault_sem, 0, 1);
static atomic_t ship_mode = ATOMIC_INIT(SHIP_MODE_WAITING_FOR_BUTTON);
static K_SEM_DEFINE(ship_mode_exit_sem, 0, 1);

static void button0_work_handler(struct k_work *work);
static void button0_pressed_handler(const struct device *port,
                                    struct gpio_callback *cb,
                                    uint32_t pins);
void enter_ecg_collection_mode(void);
void exit_ecg_collection_mode(void);
static void accel_record_fault_thread(void *arg1, void *arg2, void *arg3);

K_WORK_DEFINE(button0_work, button0_work_handler);
K_THREAD_DEFINE(accel_record_fault_thread_id, 2048,
		accel_record_fault_thread, NULL, NULL, NULL, 7, 0, 0);


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
    .frequency = 4000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    /* only in version 2.5: .cs= {
  .delay = 0,
  .gpio = {.pin = 15, .dt_flags=GPIO_ACTIVE_LOW, }
  },
  */
};

// Now we define the cs pins

struct spi_cs_control ppg_cs = {
    .delay = 0,
    .gpio = {
        .pin = 9,
        .dt_flags = GPIO_ACTIVE_LOW,
    }};




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

// Shell Commands for entering in the terminal, in case a bluetooth command is not avalible.
SHELL_CMD_REGISTER(reset, NULL, "Resets Device", NVIC_SystemReset);
SHELL_CMD_REGISTER(full_reset, NULL, "Resets Storage and Device", reset_device);





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

static int set_usb_mass_storage_enabled(bool enable)
{
  int ret;

  if (usb_enabled == enable) {
    return 0;
  }

  ret = enable ? usb_enable(usb_status_cb) : usb_disable();
  if (ret == -EALREADY) {
    ret = 0;
  }

  if (ret == 0) {
    usb_enabled = enable;
  }

  return ret;
}

static void write_uuid_file(void)
{
  bt_addr_le_t address = {0};
  size_t count = 1;
  char addr_str[BT_ADDR_LE_STR_LEN];
  int result;

  bt_id_get(&address, &count);
  bt_addr_le_to_str(&address, addr_str, sizeof(addr_str));

  result = write_ble_uuid(addr_str, bt_get_name(),
                          msense_device_identity_hex());
  if (result < 0) {
    LOG_ERR("Unable to write uuid.txt: %d", result);
  }
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
    printk("BLE init failed with error code %d\n", err);
    return;
  }
  else
    printk("BLE init success\n");

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

  // Start advertising
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
  if (err)
    printk("Advertising failed to start (err %d)\n", err);
  else
  {
    printk("Advertising started\n");
  }

  k_sem_give(&ble_init_ok);

  write_uuid_file();
  #ifndef CONFIG_DEBUG
  
    if (!security_lock){
      usb_enable(usb_status_cb);
    }
    //k_sleep(K_SECONDS(10));
    #if CONFIG_DISK_DRIVER_RAW_NAND
    set_read_only(true);
  #endif
  #endif
}

// Initialize BLE
static void ble_init(void)
{
  int err;

  
  err = bt_enable(bt_ready);
  if (err)
  {
    printk("BLE initialization failed\n");
  }

  //err = bt_id_create(BT_ADDR_LE_ANY, NULL);
  if (!err)
    printk("Bluetooth initialized\n");
  else
  {
    printk("BLE initialization did not complete in time\n");
  }
  if (err)
    printk("Bluetooth init failed (err %d)\n", err);
}

// Timer handler that periodically executes commands with a period,
// which is defined by the macro-variable TIMER_MS
static void spi_init(void)
{
  
  // device_get_binding is used for runtime aquisition of a device object. We can still use it but we have to be carefull to select the right names
  const char *const spiName_ppg = "spi@c000";

  spi_dev_ppg = DEVICE_DT_GET(DT_NODELABEL(spi3));
  //spi_dev_ppg = device_get_binding(spiName_ppg);

  if (!device_is_ready(gpio1_device))
  {
    printk("Could not get GPIO_1\n");
    return;
  }
  if (spi_dev_ppg == NULL || !device_is_ready(spi_dev_ppg))
  {
    printk("Could not get %s \n", spiName_ppg);
    return;
  }
  
  ppg_cs.gpio.port = gpio1_device;
  
  
  spi_cfg_ppg.cs = ppg_cs; // version 2.5: .gpio.port = gpio1_device;
  
}

static void i2c_init(void)
{

  printk("The I2C Init started\n");
  i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
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
  const struct device *const dev = DEVICE_DT_GET_ONE(ti_bq274xx);
  dt_update_battery(dev, true);
  
  //battery_lvl = bt_bas_get_battery_level();
  #ifndef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  if (battery_level < 5){
    // if this is our first time
    if (!battery_low){
      battery_low = true;
      LOG_WRN("battery low, turning off file logs and data collection.");
      LOG_INF("logs and data collection will resume once battery is sufficiently charged (>15 percent)");
      if (!collecting_data){
        reset_log_file();
      }
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
            
            exit_ecg_collection_mode();
        }
        else if (!battery_low && host_wants_collection && !collecting_data){

            enter_ecg_collection_mode();
            
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
	k_sem_give(&accel_record_fault_sem);
}

static void accel_record_fault_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&accel_record_fault_sem, K_FOREVER);
		exit_ecg_collection_mode();
	}
}

/**
 * @brief Switch the device from USB mass-storage mode into ECG collection
 *        mode.
 *
 * Invoked by the button handler (short press while idle) and exposed to the
 * BLE service so a connected host can start a recording remotely. It blinks
 * the collection-mode LED pattern, removes the NAND filesystem from USB mass
 * storage, makes it writable, and starts the ECG recorder thread via
 * ecg_recorder_start().
 *
 * On success the global collecting_data / host_wants_collection flags are
 * set so the rest of the firmware (button logic, BLE status) knows a
 * recording is in progress. If the recorder fails to start, the filesystem
 * is returned to read-only and the mode change is abandoned, leaving the
 * device in its previous state.
 */
void enter_ecg_collection_mode(void)
{
  int ret = 0;
  uint64_t session_id;
  bool usb_was_enabled;
  bool icm_started = false;
  bool fsync_started = false;

  k_mutex_lock(&collection_mode_lock, K_FOREVER);
  if (collecting_data) {
    k_mutex_unlock(&collection_mode_lock);
    return;
  }

  LOG_INF("Entering ECG collection mode");
  blink_collection_mode_pattern();

  usb_was_enabled = usb_enabled;
  if (usb_was_enabled) {
    ret = set_usb_mass_storage_enabled(false);
  }
  if (ret != 0) {
    LOG_ERR("USB disable returned %d", ret);
    goto start_failed;
  }

  #if CONFIG_DISK_DRIVER_RAW_NAND
  set_read_only(false);
  #endif

  session_id = collection_session_id();
  filesystem_set_collection_id(session_id);

  ret = rtc0_collection_counter_start();
  if (ret != 0) {
    LOG_ERR("Failed to start RTC0 collection counter: %d", ret);
    goto start_failed;
  }

  ret = ecg_recorder_start();
  if (ret != 0) {
    LOG_ERR("Failed to start ECG recorder: %d", ret);
    goto start_failed;
  }

  ret = accel_recorder_start(session_id);
  if (ret != 0) {
    LOG_ERR("Failed to start accelerometer recorder: %d", ret);
    goto start_ecg_failed;
  }

  ret = icm20948_accel_set_fifo_consumer(accel_recorder_consume_fifo, NULL);
  if (ret != 0) {
    LOG_ERR("Failed to register ICM-20948 FIFO consumer: %d", ret);
    goto start_accel_failed;
  }

  ret = icm20948_accel_start();
  if (ret != 0) {
    LOG_ERR("Failed to start ICM-20948 accelerometer: %d", ret);
    goto start_accel_failed;
  }
  icm_started = true;

  ret = imu_fsync_timing_start();
  if (ret != 0) {
    LOG_ERR("Failed to start IMU FSYNC timing: %d", ret);
    goto start_accel_failed;
  }
  fsync_started = true;

  ret = rtc0_collection_notification_start();
  if (ret != 0) {
    LOG_ERR("Failed to start RTC0 BLE timing notifications: %d", ret);
    goto start_accel_failed;
  }

  host_wants_collection = true;
  collecting_data = true;
  k_mutex_unlock(&collection_mode_lock);
  return;

 start_accel_failed:
  if (fsync_started) {
    imu_fsync_timing_stop();
  }
  if (icm_started) {
    (void)icm20948_accel_stop();
  }
  (void)icm20948_accel_set_fifo_consumer(NULL, NULL);
  (void)accel_recorder_abort();
start_ecg_failed:
  (void)ecg_recorder_stop();
start_failed:
  rtc0_collection_counter_stop();
  filesystem_clear_collection_id();
  host_wants_collection = false;
  #if CONFIG_DISK_DRIVER_RAW_NAND
  set_read_only(true);
  #endif
  if (usb_was_enabled) {
    ret = set_usb_mass_storage_enabled(true);
    if (ret != 0) {
      LOG_WRN("USB restore returned %d", ret);
    }
  }
  k_mutex_unlock(&collection_mode_lock);
}

/**
 * @brief Switch the device from ECG collection mode back to USB mass-storage
 *        mode.
 *
 * The counterpart to enter_ecg_collection_mode(), invoked on a short button
 * press during recording or remotely over BLE. It first disables and drains
 * the IMU FIFO, finalizes the accelerometer file, then stops ECG recording.
 * After both streams are queued, the shared filesystem work queue is drained,
 * all files are closed, the NAND disk is set back to read-only, and USB
 * mass-storage is re-enabled.
 *
 * If the recorder does not confirm shutdown in time, the collection flags
 * are restored and the function bails out without touching the filesystem —
 * the device stays in collection mode rather than risking a USB host and
 * the recorder writing the disk at the same time.
 */
void exit_ecg_collection_mode(void)
{
  int ret;
  uint32_t fsync_edge_count;

  k_mutex_lock(&collection_mode_lock, K_FOREVER);
  if (!collecting_data) {
    k_mutex_unlock(&collection_mode_lock);
    return;
  }

  LOG_INF("Entering USB mass storage mode");
  blink_usb_mode_pattern();

  host_wants_collection = false;
  collecting_data = false;

  /*
   * A very short recording may not yet have enough transitions to estimate
   * the IMU period. Keep the stream alive until the second hardware marker
   * edge, then leave more than one worst-case sample period for it to enter
   * the FIFO before the final drain.
   */
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
  }
  imu_fsync_timing_stop();
  ret = icm20948_accel_set_fifo_consumer(NULL, NULL);
  if (ret != 0) {
    LOG_WRN("ICM-20948 FIFO consumer clear returned %d", ret);
  }

  ret = accel_recorder_stop();
  if (ret != 0) {
    LOG_WRN("Accelerometer recorder stop returned %d", ret);
  }

  ret = ecg_recorder_stop();
  if (ret != 0) {
    LOG_WRN("ECG recorder stop returned %d", ret);
    host_wants_collection = true;
    collecting_data = true;
    k_mutex_unlock(&collection_mode_lock);
    return;
  }

  rtc0_collection_counter_stop();
  flush_data_buffer(ecg);
  if (filesystem_workqueue_started) {
    (void)k_work_queue_drain(&my_work_q, true);
    k_work_queue_unplug(&my_work_q);
  }
  close_all_files();
  filesystem_clear_collection_id();

  #if CONFIG_DISK_DRIVER_RAW_NAND
  set_read_only(true);
  #endif

  ret = set_usb_mass_storage_enabled(true);
  if (ret != 0) {
    LOG_WRN("USB enable returned %d", ret);
  }
  k_mutex_unlock(&collection_mode_lock);
}

static void button0_work_handler(struct k_work *work)
{
  int button_state;
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
    LOG_WRN("Long button press detected, erasing flash");
    if (collecting_data) {
      exit_ecg_collection_mode();
      if (collecting_data) {
        LOG_ERR("Collection did not stop; refusing to erase active storage");
        return;
      }
    }

    reset_lock = true;
    (void)set_usb_mass_storage_enabled(false);

    #if CONFIG_DISK_DRIVER_RAW_NAND
    set_read_only(false);
    #endif

    shutdown_filesystem();
    blink_flash_format_pattern();
    reset_device(false);
    return;
  }

  if (collecting_data) {
    exit_ecg_collection_mode();
  } else {
    enter_ecg_collection_mode();
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

  printk("Starting Application... \n");
  LOG_INF("Starting Logging...\n");

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
  setup_disk();
  filesystem_workqueue_init();

  k_sleep(K_SECONDS(1));
  #if CONFIG_DISK_DRIVER_RAW_NAND
    set_read_only(true);
  #endif

  #ifdef CONFIG_MSENSE_USB_SECURITY
    security_lock = true;
  #endif

  ret = set_usb_mass_storage_enabled(true);
  if (ret != 0)
  {
    LOG_ERR("Failed to enable USB mass storage: %d", ret);
  }

  k_sleep(K_SECONDS(2));
  

  gpio0_device = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  gpio1_device = DEVICE_DT_GET(DT_NODELABEL(gpio1));
  
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
  
  // Init, verify ID and config sensors
  
  //spi_init();
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
      LOG_INF("filesystem work queue active: %d", filesystem_workqueue_started);
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
