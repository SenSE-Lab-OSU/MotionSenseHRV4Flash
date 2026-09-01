/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <nrfx.h>
#include <nrfx_timer.h>
#include <nrfx_uarte.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include "batterymonitordt.h"
#include "ppgSensor.h"
#include "imuSensor.h"
#include "common.h"
#include "BLEService.h"
#include "msense_device_identity.h"
#include "zephyrfilesystem.h"
#include "msense_msc_media.h"
#if CONFIG_DISK_DRIVER_RAW_NAND
#include "nand_disk.h"
#endif
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <errno.h>
#include <string.h>




LOG_MODULE_REGISTER(main);

BUILD_ASSERT(IS_ENABLED(CONFIG_USB_MASS_STORAGE),
	     "PPGv2 requires legacy USB MSC support");



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 6000
#define BATTERY_LOG_INTERVAL_MAINTENANCE_CYCLES 4U

/* The devicetree node identifier for the "led0" alias. */
#define LED_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define PPG_POWER_NODE DT_ALIAS(led2) 
#define IMU_NODE DT_ALIAS(imu)
#define PPG_NODE DT_ALIAS(ppg)
#define IMU_BUS_NODE DT_ALIAS(imu_bus)
#define PPG_BUS_NODE DT_ALIAS(ppg_bus)
#define BATTERY_GAUGE_NODE DT_ALIAS(battery_gauge)
#define BATTERY_BUS_NODE DT_ALIAS(battery_bus)
#define GPIO0_NODE DT_ALIAS(gpio0)
#define GPIO1_NODE DT_ALIAS(gpio1)

// define our red and green leds
#define LED_PIN DT_GPIO_PIN(LED_NODE, gpios)
#define LED1_PIN DT_GPIO_PIN(LED1_NODE, gpios)

#define LED_FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)



#define PPG_POWER_PIN DT_GPIO_PIN(PPG_POWER_NODE, gpios)
#define PPG_POWER_FLAGS DT_GPIO_FLAGS(PPG_POWER_NODE, gpios)

const struct device* gpio0_device;
const struct device* gpio1_device;


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
// SPI Mode-3 IMU
struct spi_config spi_cfg_imu =
{
    .frequency = DT_PROP(IMU_NODE, spi_max_frequency),
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = DT_REG_ADDR(IMU_NODE),
    .cs = SPI_CS_CONTROL_INIT(IMU_NODE, 0),
    };


// SPI Mode-3 PPG
struct spi_config spi_cfg_ppg = {
    .frequency = DT_PROP(PPG_NODE, spi_max_frequency),
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = DT_REG_ADDR(PPG_NODE),
    .cs = SPI_CS_CONTROL_INIT(PPG_NODE, 0),
};




const struct device *spi_dev_ppg, *spi_dev_imu;
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

#define MSENSE_PRODUCT_BLE_NAME_PREFIX "MSense4PPG-"
#define MSENSE_PRODUCT_BLE_NAME_LEN 16U

static struct msense_device_identity device_identity;
static const struct msense_device_identity_config device_identity_config = {
	.ble_name_prefix = MSENSE_PRODUCT_BLE_NAME_PREFIX,
	.ble_name_prefix_len = sizeof(MSENSE_PRODUCT_BLE_NAME_PREFIX) - 1U,
	.ble_name_len = MSENSE_PRODUCT_BLE_NAME_LEN,
	.dis_model = CONFIG_BT_DIS_MODEL,
};
static bool uuid_ble_address_update_needed;
static bool uuid_ble_address_msc_deferred;

#define AD_FIELD_OVERHEAD 2U
#define AD_FLAGS_ENCODED_LEN (AD_FIELD_OVERHEAD + 1U)
#define AD_SERVICE_DATA_LEN (BT_UUID_SIZE_128 + MSENSE_DEVICE_ID_LEN)
#define AD_SERVICE_DATA_ENCODED_LEN (AD_FIELD_OVERHEAD + AD_SERVICE_DATA_LEN)
#define SCAN_RESPONSE_NAME_ENCODED_LEN (AD_FIELD_OVERHEAD + MSENSE_PRODUCT_BLE_NAME_LEN)

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
BUILD_ASSERT(sizeof(MSENSE_PRODUCT_BLE_NAME_PREFIX) - 1U +
		     MSENSE_DEVICE_NAME_SUFFIX_LEN == MSENSE_PRODUCT_BLE_NAME_LEN,
		     "BLE name prefix and suffix lengths must match");
BUILD_ASSERT(MSENSE_PRODUCT_BLE_NAME_LEN <= MSENSE_DEVICE_NAME_MAX_LEN,
		     "BLE name exceeds shared identity storage");
BUILD_ASSERT(sizeof(CONFIG_BT_DEVICE_NAME) == sizeof(MSENSE_PRODUCT_BLE_NAME_PREFIX),
		     "Configured BLE prefix length changed unexpectedly");
BUILD_ASSERT(sizeof(advertising_service_data) == 24U,
             "Service data must contain UUID and device ID");
BUILD_ASSERT(AD_FLAGS_ENCODED_LEN + AD_SERVICE_DATA_ENCODED_LEN <=
             BT_GAP_ADV_MAX_ADV_DATA_LEN,
             "Primary advertising data exceeds the legacy limit");
BUILD_ASSERT(SCAN_RESPONSE_NAME_ENCODED_LEN <= BT_GAP_ADV_MAX_ADV_DATA_LEN,
             "Scan response name exceeds the legacy limit");

static int shell_request_ppg_reboot(const struct shell *shell, size_t argc,
                                    char **argv)
{
  ARG_UNUSED(shell);
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  return request_ppg_storage_reboot();
}

static int shell_request_ppg_full_reset(const struct shell *shell, size_t argc,
                                        char **argv)
{
  ARG_UNUSED(shell);
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  return request_ppg_storage_reset(false);
}

// Shell Commands for entering in the terminal, in case a bluetooth command is not avalible.
SHELL_CMD_REGISTER(reset, NULL, "Queues a safe device reset", shell_request_ppg_reboot);
SHELL_CMD_REGISTER(full_reset, NULL, "Queues a safe storage erase and reset",
                   shell_request_ppg_full_reset);





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
	ARG_UNUSED(param);

	switch (status) {
	case USB_DC_CONNECTED:
		LOG_INF("USB connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_INF("USB configured");
		break;
	case USB_DC_DISCONNECTED:
		LOG_INF("USB disconnected");
		break;
	case USB_DC_ERROR:
		LOG_ERR("USB controller error");
		break;
	default:
		break;
	}
}

static int enable_usb_msc_host_media(void)
{
	int claim_ret;
	int disable_ret;
	int ret;

	ret = usb_enable(usb_status_cb);
	if (ret != 0) {
		/* Boot-failure containment also covers an unprovable -EALREADY state. */
		disable_ret = usb_disable();
		if (disable_ret == 0) {
			ppg_collection_set_usb_msc_enabled(false);
		} else {
			ppg_collection_set_usb_msc_enabled(true);
			LOG_ERR("USB disable after enable failure returned %d", disable_ret);
		}
		return ret;
	}
	ppg_collection_set_usb_msc_enabled(true);

	ret = msense_msc_media_initialize_absent();
	if (ret != 0) {
		LOG_ERR("MSC safe initialization failed: %d", ret);
		goto disable_usb;
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

disable_usb:
	/* Do not leave the SDK's default writable MSC policy reachable on error. */
	disable_ret = usb_disable();
	if (disable_ret == 0) {
		ppg_collection_set_usb_msc_enabled(false);
	}
	if (disable_ret != 0) {
		LOG_ERR("USB disable after MSC failure returned %d", disable_ret);
	}
	return ret;
}

static int complete_pending_uuid_ble_address(void)
{
	bt_addr_le_t identity_address;
	char ble_address[BT_ADDR_LE_STR_LEN];
	size_t address_count = 1U;
	int teardown_ret;
	int ret;

	bt_id_get(&identity_address, &address_count);
	if (address_count != 1U || identity_address.type != BT_ADDR_LE_RANDOM ||
	    !BT_ADDR_IS_STATIC(&identity_address.a)) {
		return -ENODEV;
	}

	ret = bt_addr_le_to_str(&identity_address, ble_address, sizeof(ble_address));
	if (ret < 0 || (size_t)ret >= sizeof(ble_address)) {
		return -ENOSPC;
	}
	if (filesystem_is_mounted()) {
		return -EBUSY;
	}

#if CONFIG_DISK_DRIVER_RAW_NAND
	set_read_only(false);
#endif
	ret = setup_disk();
	if (ret == 0) {
		ret = write_device_info_ble_address(ble_address);
	}
	if (filesystem_is_mounted()) {
		teardown_ret = filesystem_gate_and_drain();
		if (teardown_ret == 0) {
			teardown_ret = shutdown_filesystem();
		}
		if (ret == 0 && teardown_ret != 0) {
			ret = teardown_ret;
		}
	}
#if CONFIG_DISK_DRIVER_RAW_NAND
	set_read_only(true);
#endif
	if (ret != 0 || filesystem_is_mounted()) {
		return ret != 0 ? ret : -EBUSY;
	}

	if (uuid_ble_address_msc_deferred) {
		ret = enable_usb_msc_host_media();
		if (ret != 0) {
			return ret;
		}
		uuid_ble_address_msc_deferred = false;
	}

	return 0;
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
    ppg_collection_latch_storage_fault();
    return;
  }
  else
    printk("BLE init success\n");

  #if CONFIG_BT_SETTINGS
    settings_load();
  #endif

  if (uuid_ble_address_update_needed) {
    err = complete_pending_uuid_ble_address();
    if (err) {
      LOG_ERR("Unable to update uuid.txt with BLE address: %d", err);
      ppg_collection_latch_storage_fault();
      return;
    }
    uuid_ble_address_update_needed = false;
  }


  // Configure connection callbacks
  bt_conn_cb_register(&conn_callbacks);

  err = bt_set_name(msense_device_identity_name(&device_identity));
  if (err) {
    LOG_ERR("Unable to set generated BLE name: %d", err);
    ppg_collection_latch_storage_fault();
    return;
  }

  memcpy(&advertising_service_data[BT_UUID_SIZE_128],
         msense_device_identity_bytes(&device_identity), MSENSE_DEVICE_ID_LEN);
  sd[0].data = msense_device_identity_name(&device_identity);
  sd[0].data_len = msense_device_identity_name_len(&device_identity);

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
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    ppg_collection_latch_storage_fault();
    return;
  }
  printk("Advertising started\n");

  k_sem_give(&ble_init_ok);

  ppg_collection_enable_runtime_transitions();
}

// Initialize BLE
static void ble_init(void)
{
  int err;

  
  err = bt_enable(bt_ready);
  if (err)
  {
    printk("BLE initialization failed\n");
    ppg_collection_latch_storage_fault();
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
  const char *const spiName_imu = "spi@9000";
  const char *const spiName_ppg = "spi@c000";

  spi_dev_imu = DEVICE_DT_GET(IMU_BUS_NODE);
  spi_dev_ppg = DEVICE_DT_GET(PPG_BUS_NODE);
  // device_get_binding(spiName_imu);
  //spi_dev_ppg = device_get_binding(spiName_ppg);

  if (spi_dev_imu == NULL || !device_is_ready(spi_dev_imu))
  {
    printk("Could not get %s \n", spiName_imu);
    return;
  }

  if (spi_dev_ppg == NULL || !device_is_ready(spi_dev_ppg))
  {
    printk("Could not get %s \n", spiName_ppg);
    return;
  }
  
}

void spi_verify_sensor_ids()
{
  
  if (device_is_ready(spi_dev_imu))
  {
    getIMUID();
  }
  else
  {
    LOG_WRN("IMU not ready, setup avoided");
  }
  
  k_sleep(K_SECONDS(1));

  if (device_is_ready(spi_dev_ppg))
  {
    read_ppg_chip_id();
  }
  else
  {
    LOG_WRN("ppg not ready, setup was avoided");
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
    request_ppg_collection_reconcile();
  }
  #endif
    
}



void blink_led(gpio_pin_t pin){
  gpio_pin_set(gpio0_device, pin, 1);
  k_sleep(K_MSEC(200));
  gpio_pin_set(gpio0_device, pin, 0);
}



void storage_clear_led(){
  gpio_pin_set(gpio0_device, LED1_PIN, 1);
}

int main(void)
{
	int identity_err;
	int ret;
	int teardown_ret;
	int uuid_ret = 0;
	bool boot_storage_ready = false;
	bool uuid_address_present = false;

  printk("Starting Application... \n");
  LOG_INF("Starting Logging...\n");

	identity_err = msense_device_identity_init(&device_identity,
						   &device_identity_config);
	if (identity_err) {
		LOG_ERR("Unable to initialize factory device identity: %d", identity_err);
	}
  
  
  

  // Setup our Flash Filesystem
  ret = setup_disk();
  if (ret != 0) {
    LOG_ERR("Startup filesystem setup failed: %d", ret);
    #if CONFIG_DISK_DRIVER_RAW_NAND
    set_read_only(true);
    #endif
    return ret;
  }
  k_sleep(K_SECONDS(1));
  //create_test_files(400);
  #ifdef CONFIG_MSENSE_USB_SECURITY
    security_lock = true;
  #endif


  k_sleep(K_SECONDS(2));
  

  gpio0_device = DEVICE_DT_GET(GPIO0_NODE);
  gpio1_device = DEVICE_DT_GET(GPIO1_NODE);
  
  // Initialize our 2 LED pins and 5V PPG Power Pin
  ret = gpio_pin_configure(gpio0_device, LED_PIN, GPIO_OUTPUT_INACTIVE | LED_FLAGS);
  ret = gpio_pin_configure(gpio0_device, LED1_PIN, GPIO_OUTPUT_INACTIVE | LED_FLAGS);
  ret = gpio_pin_configure(gpio1_device, PPG_POWER_PIN, GPIO_OUTPUT_ACTIVE | PPG_POWER_FLAGS);
  // initialize imu ground pin
  ret = gpio_pin_configure(gpio0_device, 27, GPIO_OUTPUT_INACTIVE);
  if (ret < 0)
  {
    printk("Error: Can't initialize LED");
    // return;
  }
  
  // Init, verify ID and config sensors
  
  spi_init();
  spi_verify_sensor_ids();

  i2c_init();

  // Shutdown our ppg and imu sensors until we need to get data from them
  ppg_sleep();
  motion_sleep();

  /*struct k_work_queue_config cfg = {
    .name = "my_custom_workq",
    .no_yield = false
  };*/

  // Start Threads for all our sensor tasks
  // This is the file system workqueue (workqueues are threads that process items in a queue), it processes uploading files to the filesystem. 
  k_work_queue_init(&my_work_q);
  k_work_queue_start(&my_work_q, my_stack_area,
                     K_THREAD_STACK_SIZEOF(my_stack_area), WORKQUEUE_PRIORITY, NULL);
  // Handles reading from the motion sensor and ppg sensor. This is the system workqueue, which zephyr creates by default
  // and is a different queue from the user created file system workqueue 
  k_work_init(&my_motionSensor.work, motion_data_timeout_handler);
  k_work_init(&my_ppgSensor.work, read_ppg_fifo_buffer);
  // sends enmo and accelerometer
  k_work_init(&my_motionData.work, motion_notify);
#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  // These items all handle sending data across bluetooth

  k_work_init(&my_ppgDataSensor.work, ppgData_notify);
#endif

  // these are our file system workqueue objects
  k_work_init(&ppg_work_item.work, work_write);
  ppg_work_item.sensor = ppg;

  k_work_init(&accel_work_item.work, work_write);
  accel_work_item.sensor = accelorometer;
  
  k_work_init(&log_work_item.work, work_write);
  log_work_item.sensor = customlog;
  ppg_collection_set_filesystem_workqueue_ready();
  k_thread_name_set(&my_work_q.thread, "file_sys");
  const char *name = k_thread_name_get(&my_work_q.thread);
  LOG_INF("file workqueue thead: %s", name);
  ret = get_storage_percent_full();
  if (ret < 0) {
    LOG_ERR("Startup storage query failed: %d", ret);
  }

  if (identity_err == 0) {
    uuid_ret = write_device_info_file(msense_device_identity_name(&device_identity),
                                      msense_device_identity_hex(&device_identity),
                                      msense_device_identity_model(&device_identity),
                                      &uuid_address_present);
    if (uuid_ret != 0) {
      LOG_ERR("Unable to write boot uuid.txt: %d", uuid_ret);
    } else {
      uuid_ble_address_update_needed = !uuid_address_present;
    }
  }

  teardown_ret = filesystem_gate_and_drain();
  if (teardown_ret == 0) {
    teardown_ret = shutdown_filesystem();
  }
  #if CONFIG_DISK_DRIVER_RAW_NAND
  set_read_only(true);
  #endif

  if (teardown_ret != 0 || filesystem_is_mounted()) {
    if (teardown_ret == 0) {
      teardown_ret = -EBUSY;
    }
    LOG_ERR("Startup filesystem teardown failed: %d", teardown_ret);
    ppg_collection_latch_storage_fault();
  } else if (identity_err != 0) {
    LOG_ERR("BLE advertising disabled because device identity is unavailable");
  } else if (uuid_ret != 0) {
    ppg_collection_latch_storage_fault();
  } else {
    #ifndef CONFIG_DEBUG
    if (!security_lock){
      if (uuid_ble_address_update_needed) {
        uuid_ble_address_msc_deferred = true;
        boot_storage_ready = true;
      } else {
        ret = enable_usb_msc_host_media();
        if (ret != 0) {
          LOG_ERR("Unable to publish USB MSC medium: %d", ret);
          ppg_collection_latch_storage_fault();
        } else {
          boot_storage_ready = true;
        }
      }
    } else {
      boot_storage_ready = true;
    }
    #else
    boot_storage_ready = true;
    #endif
  }

  if (boot_storage_ready) {
    ble_init();
  } else if (identity_err == 0 && uuid_ret == 0 &&
             teardown_ret == 0 && !filesystem_is_mounted() &&
             ppg_collection_faulted()) {
      LOG_ERR("BLE advertising disabled because boot storage is unavailable");
  }
  
  
  // we set global update at 9 so that when we are entering the while loop, we will check the storage & battery.
  int global_update = 9;
  int update_time = SLEEP_TIME_MS;
  
  
  while (1)
  {
    
    //printk("%d %d\n", connectedFlag, collecting_data);

    global_update++;
    if (global_update >= 100){
      global_update = 0;
    }

    if (global_update % 5 == 0){
      battery_maintenance();
      get_current_unix_time();
      LOG_INF("state: %d", k_work_busy_get(&accel_work_item.work));
      LOG_INF("connected: %d, collecting: %d", connectedFlag, collecting_data);
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
