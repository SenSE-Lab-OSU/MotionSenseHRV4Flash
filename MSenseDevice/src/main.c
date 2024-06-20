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
#include "batteryMonitor.h"
#include "ppgSensor.h"
#include "imuSensor.h"
#include "common.h"
#include "BLEService.h"
#include "zephyrfilesystem.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

// dfu configuration


#ifdef INCLUDE_DFU
#ifdef CONFIG_BOOTLOADER_MCUBOOT 

#include "os_mgmt/os_mgmt.h"
#include "img_mgmt/img_mgmt.h"
#include "stats/stats.h"
#include "stat_mgmt/stat_mgmt.h"

#include <mgmt/mcumgr/smp_bt.h>
#endif
#endif

LOG_MODULE_REGISTER(main);

#define READMASTER 0x80

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 3000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define PPG_POWER_NODE DT_ALIAS(led2) 

// #define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
// #define LED0 DEVICE_DT_NAME(LED0_NODE)
#define LED_PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define LED_FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

#define LED1_PIN DT_GPIO_PIN(LED1_NODE, gpios)

#define PPG_POWER_PIN DT_GPIO_PIN(PPG_POWER_NODE, gpios)
#define PPG_POWER_FLAGS DT_GPIO_FLAGS(PPG_POWER_NODE, gpios)

const struct device* gpio0_device;
const struct device* gpio1_device;

struct spi_cs_control imu_cs = {
    .delay = 0,
    .gpio = {.pin = 26, .dt_flags = GPIO_ACTIVE_LOW}};

// the gpio struct now requires a port? not sure what to do in this case
struct spi_cs_control ppg_cs = {
    .delay = 0,
    .gpio = {
        .pin = 9,
        .dt_flags = GPIO_ACTIVE_LOW,
    }};
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
    .frequency = 1000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    // this should work, but for some reason it doesn't. However, this might work without
    // the cs field anyway, apparently it is auto managed, so test first
    //.cs = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spi2), 0)
    };

//struct spi_config spi_cfg_imu2 = SPI_CONFIG_DT(DT_NODELABEL(spi2), spi_cfg_imu.operation, 0);

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


struct ppgData ppgData1;

const struct device *spi_dev_ppg, *spi_dev_imu;
const struct device *i2c_dev;

struct bq274xx_data batteryMonitor;
struct bq274xx_config batteryMonitorConfig;

uint8_t blePktTFMicro[ble_tfMicroPktLength];

struct accData accData1;
struct gyroData gyroData1;
struct magnetoData magnetoData1;

struct accel_config accelConfig;
struct gyro_config gyroConfig;
struct magneto_config magnetoConfig;
struct orientation_config orientationConfig;

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define DIS_FW_REV_STR CONFIG_BT_DIS_FW_REV_STR
#define DIS_FW_REV_STR_LEN (sizeof(DIS_FW_REV_STR))

#define DIS_HW_REV_STR CONFIG_BT_DIS_HW_REV_STR
#define DIS_HW_REV_STR_LEN (sizeof(DIS_HW_REV_STR))

#define DIS_MANUF CONFIG_BT_DIS_MANUF
#define DIS_MANUF_LEN (sizeof(DIS_MANUF))

#define DIS_MODEL CONFIG_BT_DIS_MODEL
#define DIS_MODEL_LEN (sizeof(DIS_MODEL))

#define RUN_STATUS_LED DK_LED1

static K_SEM_DEFINE(ble_init_ok, 0, 1);

uint16_t global_counter;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, TFMICRO_SERVICE_UUID),
};

struct bt_conn *my_connection;

// Setting up the device information service
static int settings_runtime_load(void)
{

/*#ifdef CONFIG_BOOTLOADER_MCUBOOT
  settings_runtime_set("bt/dis/model",
                       DIS_MODEL, DIS_MODEL_LEN);
  settings_runtime_set("bt/dis/manuf",
                       DIS_MANUF, DIS_MANUF_LEN);
  settings_runtime_set("bt/dis/fw",
                       DIS_FW_REV_STR, DIS_FW_REV_STR_LEN);
  settings_runtime_set("bt/dis/hw",
                       DIS_HW_REV_STR, DIS_HW_REV_STR_LEN);
#endif
*/
  return 0;
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
  // If acceptable params, return true, otherwise return false.
  if ((param->interval_min > 9) && (param->interval_max > 20))
    return false;
  else
    return true;
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
    .le_param_req = le_param_req,
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

  //settings_load();

  //settings_runtime_load();

  // Configure connection callbacks
  bt_conn_cb_register(&conn_callbacks);


  if (err)
    return;

  // Start advertising
  const struct bt_le_adv_param v = {
      .id = BT_ID_DEFAULT,
      .sid = 0,
      .secondary_max_skip = 0,
      .options = BT_LE_ADV_OPT_CONNECTABLE,
      .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
      .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
      .peer = NULL};

  err = bt_le_adv_start(&v, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err)
    printk("Advertising failed to start (err %d)\n", err);
  else
  {
    printk("Advertising successfully started\n");
  }

  k_sem_give(&ble_init_ok);
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
  uint32_t dataFlash;
  // device_get_binding is used for runtime aquisition of a device object. We can still use it but we have to be carefull to select the right names
  const char *const spiName_imu = "spi@9000";
  const char *const spiName_ppg = "spi@c000";

  spi_dev_imu = DEVICE_DT_GET(DT_NODELABEL(spi2)); // device_get_binding(spiName_imu);
  gpio0_device = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  spi_dev_ppg = DEVICE_DT_GET(DT_NODELABEL(spi3));
  //spi_dev_ppg = device_get_binding(spiName_ppg);
  gpio1_device = DEVICE_DT_GET(DT_NODELABEL(gpio1));

  if (!device_is_ready(gpio0_device))
  {
    printk("Could not get GPIO_0\n");
    return;
  }

  if (!device_is_ready(gpio1_device))
  {
    printk("Could not get GPIO_1\n");
    return;
  }
  if (spi_dev_imu == NULL || !device_is_ready(spi_dev_imu))
  {
    printk("Could not get %s device\n", spiName_imu);
    return;
  }

  if (spi_dev_ppg == NULL || !device_is_ready(spi_dev_ppg))
  {
    printk("Could not get %s device\n", spiName_ppg);
    return;
  }
  
  imu_cs.gpio.port = gpio0_device;
  ppg_cs.gpio.port = gpio1_device;
  
  struct spi_config* spi_test_config = spi_dev_imu->config;
  //spi_cfg_ppg.gpio.port = gpio1_device;
  spi_cfg_imu.cs = imu_cs;
  spi_cfg_ppg.cs = ppg_cs; // version 2.5: .gpio.port = gpio1_device;

  getIMUID();
  
  // high_pass_filter_init_25();

  // fileOpen();
  dataFlash = (((uint32_t)ppgConfig.green_intensity) << 8) + ((uint32_t)ppgConfig.infraRed_intensity);
  printk("data combo = %d,%d,%d\n", dataFlash, ((uint32_t)ppgConfig.green_intensity) << 8, (uint32_t)ppgConfig.infraRed_intensity);
  // fileWrite(dataFlash);
  // fileClose();

  accelConfig.isEnabled = true;
  accelConfig.txPacketEnable = true;
  accelConfig.sample_bw = ACCEL_DLPFCFG_12HZ;
  accelConfig.sensitivity = ACCEL_FS_SEL_4g;

  gyroConfig.isEnabled = true;
  gyroConfig.txPacketEnable = true;
  gyroConfig.tot_samples = 8;
  gyroConfig.sensitivity = GYRO_FS_SEL_250;
  orientationConfig.isEnabled = true;
  orientationConfig.txPacketEnable = true;

  magnetoConfig.isEnabled = false;
  magnetoConfig.txPacketEnable = true;
  //tfMicroCoonfig.isEnabled = true;
  //tfMicroCoonfig.txPacketEnable = true;

  configRead[1] = IMU_ENABLE | MAGNETOMETER_ENABLE | PPG_ENABLE |
                  ORIENTATION_ENABLE | TFMICRO_ENABLE;
  configRead[0] = MOTION_BLE_ENABLE | MAGNETOMETER_BLE_ENABLE |
                  PPG_BLE_ENABLE | ORIENTATION_BLE_ENABLE | TFMICRO_BLE_ENABLE;
  configRead[2] = ppgConfig.green_intensity;
  configRead[3] = ppgConfig.infraRed_intensity;
  configRead[4] = 0x12;
  configRead[5] = 0x44;
}

void spi_verify_sensor_ids()
{
  uint8_t tx_buffer[4], rx_buffer[4];
  tx_buffer[0] = READMASTER | 0x00;
  tx_buffer[1] = 0xFF;
  uint8_t txLen = 2, rxLen = 2;
  if (device_is_ready(spi_dev_imu))
  {

    spiReadWriteIMU(tx_buffer, txLen, rx_buffer, rxLen);
    LOG_INF("Chip ID from motion sensor=%x\n", rx_buffer[1]);
  }
  else
  {
    LOG_WRN("IMU not ready, setup was avoided");
  }

  tx_buffer[0] = PPG_CHIP_ID_1;
  tx_buffer[1] = READMASTER;
  tx_buffer[2] = 0x00;

  txLen = 3;
  rxLen = 3;
  k_sleep(K_SECONDS(1));
  if (device_is_ready(spi_dev_ppg))
  {
    spiReadWritePPG(tx_buffer, txLen, rx_buffer, rxLen);
    LOG_INF("Chip ID from ppg sensor=%x,%x,%x\n", rx_buffer[0], rx_buffer[1], rx_buffer[2]);
  }
  else
  {
    LOG_WRN("ppg not ready, setup was avoided");
  }
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
  /*
  i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

  batteryMonitorConfig.design_capacity = 0x00AA;   // 170 mAHour
  batteryMonitorConfig.taper_current = 0x0015;     // 21 mA
  batteryMonitorConfig.terminate_voltage = 0x0C1C; // 3100 mV
  bq274xx_gauge_init(&batteryMonitorConfig);
  */
}

// Timer handler that periodically executes commands with a period,
// which is defined by the macro-variable TIMER_MS

#define WORKQUEUE_PRIORITY -1
#define WORKQUEUE_STACK_SIZE 40048
K_THREAD_STACK_DEFINE(my_stack_area, WORKQUEUE_STACK_SIZE);

void battery_maintenance()
{
  const struct device *const dev = DEVICE_DT_GET_ONE(ti_bq274xx);
  do_main(dev);
  
  //battery_lvl = bt_bas_get_battery_level();

  if (collecting_data || host_wants_collection){
        if (battery_level < 5 && collecting_data){
            start_stop_device_collection(false);
        }
        else if (battery_level > 15 && host_wants_collection && !collecting_data){
            start_stop_device_collection(true);
        }
  }
    
}

void blink_led(gpio_pin_t pin){
  gpio_pin_set(gpio0_device, pin, 1);
  k_sleep(K_MSEC(200));
  gpio_pin_set(gpio0_device, pin, 0);
}

void storage_clear_led(){
  gpio_pin_set(gpio0_device, LED1_PIN, 1);
}

void main(void)
{

  printk("Starting Application... \n");
  LOG_INF("Starting Logging...\n");
  
  // Setup LEDs and Power Pins
  

  // Setup our Flash Filesystem
  setup_disk();

  usb_enable(usb_status_cb);
  k_sleep(K_SECONDS(1));

// this initializes FOTA
#ifdef INCLUDE_DFU
#ifdef CONFIG_BOOTLOADER_MCUBOOT
  os_mgmt_register_group();

  img_mgmt_register_group();
  smp_bt_register();
#endif
#endif

  bool led_is_on = true;
  bool led1_is_on = true;

  // the "1" is the timer priority
  IRQ_CONNECT(TIMER1_IRQn, 1,
              nrfx_timer_1_irq_handler, NULL, 0);


  // Init, verify ID and config sensors
  spi_init();
  int ret;

  // Initialize our 2 LED pins and 5V PPG Power Pin
  ret = gpio_pin_configure(gpio0_device, LED_PIN, GPIO_OUTPUT_INACTIVE | LED_FLAGS);
  ret = gpio_pin_configure(gpio0_device, LED1_PIN, GPIO_OUTPUT_INACTIVE | LED_FLAGS);
  ret = gpio_pin_configure(gpio1_device, PPG_POWER_PIN, GPIO_OUTPUT_ACTIVE | PPG_POWER_FLAGS);
  
  spi_verify_sensor_ids();

  

  i2c_init();

  ppg_turn_off();
  

  // Start Threads for all our sensor tasks
  k_work_queue_init(&my_work_q);
  k_work_queue_start(&my_work_q, my_stack_area,
                     K_THREAD_STACK_SIZEOF(my_stack_area), WORKQUEUE_PRIORITY, NULL);
  k_work_init(&my_motionSensor.work, motion_data_timeout_handler);
  k_work_init(&my_ppgSensor.work, read_ppg_fifo_buffer);
  k_work_init(&my_motionData.work, motion_notify);
#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  k_work_init(&my_magnetoSensor.work, magneto_notify);
  k_work_init(&my_orientaionSensor.work, orientation_notify);
  k_work_init(&my_ppgDataSensor.work, ppgData_notify);
#endif

  // TODO: make the file system init properly
  k_work_init(&work_item.work, work_write);

  ble_init();

  // dev = device_get_binding(LED0);
  // dev = DEVICE_DT_GET(LED0_NODE);
  // if (dev == NULL || !device_is_ready(dev)){
  
  
  if (ret < 0)
  {
    printk("Error: Can't initialize LED");
    // return;
  }
  int storage_update = 14;
  int global_update = 0;
  int update_time = SLEEP_TIME_MS;
  enable_read_only(true);
  
  
  while (1)
  {
    
    printk("%d %d\n", connectedFlag, collecting_data);

    global_update++;
    if (global_update >= 100){
      global_update = 0;
    }

    if (global_update % 10 == 0){
      battery_maintenance();
    }
    if (file_lock){
      update_time = 250;
    }

    if (!connectedFlag){
    // blink the LED while we aren't connected.
      blink_led(LED_PIN);
      
    }
    else
    {
      //When Connected, LED is always on for now, but we can change to 0 so tha it only blinks once every 15 cycles
      led_is_on = 1;
      if (!file_lock){
      storage_update++;
      // update how much storage we have left every 40 cycles 
      if (storage_update >= 40)
      { 
        get_storage_percent_full();
        get_current_unix_time();
        storage_update = 0;
      }
      if (global_update % 10 == 0){
        blink_led(LED_PIN);
      }
      }
      
    }
    if (collecting_data){
        blink_led(LED1_PIN);
    }
    
    
    k_msleep(update_time);
    
  }
}
