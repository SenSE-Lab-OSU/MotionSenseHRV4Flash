
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>
//#include <sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include "drivers/jdec_nor/custom_qspi.h"
#include <zephyr/drivers/flash.h>
#include "ppgSensor.h"
#include "imuSensor.h"
#include "batteryMonitor.h"
#include "zephyrfilesystem.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "zephyr/bluetooth/services/bas.h"
#include <nrfx_timer.h>
#include "BLEService.h"

#include <nrfx_rtc.h>

#if ((32768U % IMU_RTC_TICK_HZ) != 0)
#error "IMU_RTC_TICK_HZ must divide the 32768 Hz RTC clock exactly"
#endif

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "drivers/nand/spi_nand.h"
#include "drivers/nand/nand_disk.h"
#endif

#define CONFIG_NAME "Configure sensor"
#define TFMICRO_NAME "Micromarker Heart-rate"
#define PPG_NAME "PPG Sensor"
#define ACC_NAME "Acc and Gyroscope"
#define MAGNETO_NAME "Magnetometer"
#define ORIENTATION_NAME "Orientation vector"

static const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(0);


LOG_MODULE_REGISTER(user_bluetooth);


bool connectedFlag = false;
bool collecting_data = false;
bool host_wants_collection = false;
bool battery_low = false;
bool file_system_full = false;
bool file_system_malfunction = false;
bool battery_charging = false;

bool* status_registers[8] = {&connectedFlag, &collecting_data, &host_wants_collection, &battery_low, &file_system_full,  &file_system_malfunction, &battery_charging };
int num_of_status_registers = 7;
bool ble_status_register_send[8] = { 0 };

uint32_t uptime;

static ssize_t update_ble_status_register(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
void update_uptime(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
static ssize_t read_generic_one(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
static ssize_t read_generic_four(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
  static ssize_t read_generic_eight(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
  static ssize_t read_enmo_threshold(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
  static ssize_t bt_reset(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_write_patient_num(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_write_date_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t write_enable_value(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_reset(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_change_name(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_change_brightness(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
  uint16_t offset, uint8_t flags);




//#if CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
// Config Data Tx
// B_1 B_2 - 0x0001 PPG enabled
//         - 0x0002 IMU enabled
//         - 0x0004 orientation enabled
//         - 0x0008 TF micro enabled
//         - 0x0010 Magnetometer enabled
//         - 0x0100 PPG BLE transmit enable
//         - 0x0200 IMU BLE transmit enable
//         - 0x0400 orientation BLE transmit enable
//         - 0x0800 TF micro BLE transmit enable
//         - 0x1000 Magnetometer BLE transmit enable
// B_3     - Green intensity
// B_4     - Infra-red intensity
// B_5     - Gyro Sensitivity, Acc sensitivity
//         - 0x01 2g
//         - 0x02 4g
//         - 0x03 8g
//         - 0x04 16g
//         - 0x10 250 dps
//         - 0x20 500 dps
//         - 0x30 1000 dps
//         - 0x40 2000 dps
// B_6     - PPG sampling Rate, Motion Sampling rate
//         - 0x10 - PPG FS=200
//         - 0x20 - PPG FS=100
//         - 0x30 - PPG FS=50
//         - 0x40 - PPG FS=25
//         - 0x01 - Motion FS=200
//         - 0x02 - Motion FS=100
//         - 0x03 - Motion FS=50
//         - 0x04 - Motion FS=25
/* Later, we should just delete these and move the defining bluetooth to the bottom. */
 struct bt_uuid_128 bt_uuid_data = BT_UUID_INIT_128(UPDATE3_SERVICE_UUID);
 struct bt_uuid_128 bt_uuid_config_rx = BT_UUID_INIT_128(RX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_tfmicro_tx = BT_UUID_INIT_128(TF_HR_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_ppg_tx = BT_UUID_INIT_128(PPG_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_acc_gyro_tx = BT_UUID_INIT_128(ACC_GRYO_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_magneto_tx = BT_UUID_INIT_128(MAGNETO_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_orientation_tx = BT_UUID_INIT_128(ORIENTATION_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_ppg_quality = BT_UUID_INIT_128(PPG_QUALITY_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_acc_quality = BT_UUID_INIT_128(ACC_QUALITY_CHARACTERISTIC_UUID);
#define BT_UUID_DATA_SERVICE      (struct bt_uuid_128 *)(&bt_uuid_data)
#define BT_UUID_TFMICRO_CONFIG_RX   (struct bt_uuid_128 *)(&bt_uuid_config_rx)
#define BT_UUID_TFMICRO_TX   (struct bt_uuid_128 *)(&bt_uuid_tfmicro_tx)
#define BT_UUID_PPG_TX   (struct bt_uuid_128 *)(&bt_uuid_ppg_tx)
#define BT_UUID_ACC_GYRO_TX   (struct bt_uuid_128 *)(&bt_uuid_acc_gyro_tx)
#define BT_UUID_MAGNETO_TX   (struct bt_uuid_128 *)(&bt_uuid_magneto_tx)
#define BT_UUID_ORIENTATION_TX   (struct bt_uuid_128 *)(&bt_uuid_orientation_tx)
#define BT_UUID_PPG_QUALITY   (struct bt_uuid_128 *)(&bt_uuid_ppg_quality)
#define BT_UUID_ACC_QUALITY   (struct bt_uuid_128 *)(&bt_uuid_acc_quality)
//#else
struct bt_uuid_128 bt_uuid_control = BT_UUID_INIT_128(CONTROL_SERVICE_UUID);
struct bt_uuid_128 bt_enabledisable = BT_UUID_INIT_128(PPG_TX_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_write_enable = BT_UUID_INIT_128(WRITE_ENABLE_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_datetime = BT_UUID_INIT_128(WRITE_DATE_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_patientnum = BT_UUID_INIT_128(WRITE_PATIENT_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_reset = BT_UUID_INIT_128(WRITE_RESET_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_name = BT_UUID_INIT_128(WRITE_DEVICE_NAME_CHARACTERISTIC_UUID);
//#endif
struct bt_uuid_128 bt_uuid_status_service = BT_UUID_INIT_128(STATUS_SERVICE_UUID);
struct bt_uuid_128 bt_uuid_read_storage = BT_UUID_INIT_128(READ_STORAGE_LEFT_UUID);
struct bt_uuid_128 bt_uuid_read_status = BT_UUID_INIT_128(READ_STATUS_REGISTER_UUID);
struct bt_uuid_128 bt_uuid_read_uptime = BT_UUID_INIT_128(READ_UPTIME_UUID);
struct bt_uuid_128 bt_uuid_update_service = BT_UUID_INIT_128(UPDATE_SERVICE_UUID);
struct bt_uuid_128 bt_uuid_enmo_notify = BT_UUID_INIT_128(NOTIFY_ENMO_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_enmothreshold_notify = BT_UUID_INIT_128(NOTIFY_ENMOTHRESHOLD_CHARACTERISTIC_UUID);

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
/* TF micro Button Service Declaration and Registration */
BT_GATT_SERVICE_DEFINE(data_service,
  BT_GATT_PRIMARY_SERVICE(BT_UUID_DATA_SERVICE), //0
  /*BT_GATT_CHARACTERISTIC(BT_UUID_TFMICRO_CONFIG_RX, //1,2
    BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
    configSet, on_settings_change, configRead),
  BT_GATT_CUD(CONFIG_NAME, BT_GATT_PERM_READ),//3
  BT_GATT_CHARACTERISTIC(BT_UUID_TFMICRO_TX, //4,5
    BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, //6
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
  BT_GATT_CUD(TFMICRO_NAME, BT_GATT_PERM_READ),
  */
  //7
  BT_GATT_CHARACTERISTIC(BT_UUID_PPG_TX,//8,9
    BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, //10
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_DESCRIPTOR(BT_UUID_PPG_QUALITY,//11
    BT_GATT_PERM_READ, read_ppg_quality,
    NULL, ppgQuality),
  BT_GATT_CUD(PPG_NAME, BT_GATT_PERM_READ),//12
  BT_GATT_CHARACTERISTIC(BT_UUID_ACC_GYRO_TX,//13,14
    BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, //15
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_DESCRIPTOR(BT_UUID_ACC_QUALITY,//16
    BT_GATT_PERM_READ, read_acc_quality,
    NULL, accQuality),
  BT_GATT_CUD(ACC_NAME, BT_GATT_PERM_READ),//17
  BT_GATT_CHARACTERISTIC(BT_UUID_MAGNETO_TX,//18,19
    BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, //20
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CUD(MAGNETO_NAME, BT_GATT_PERM_READ),//21
  BT_GATT_CHARACTERISTIC(BT_UUID_ORIENTATION_TX,//22,23
    BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,
    NULL, NULL, NULL),
BT_GATT_CCC(on_cccd_changed, //24
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CUD(ORIENTATION_NAME, BT_GATT_PERM_READ)//25
);
#endif

/* See bas.c for more information, but essentially, the battery service in configured in this exact same way as here,
 with the same macros and everything.
*/

/* Write Service: Enable device, reset device, Write date time, patient num characteristics*/
BT_GATT_SERVICE_DEFINE(tfMicro_service,
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_control),
  BT_GATT_CHARACTERISTIC(&bt_uuid_write_enable,//18,19
    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    read_generic_one, write_enable_value, &host_wants_collection),
  BT_GATT_CHARACTERISTIC(&bt_uuid_datetime, 
    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    read_generic_eight, bt_write_date_time, &set_date_time),
  BT_GATT_CHARACTERISTIC(&bt_uuid_patientnum, 
    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    read_generic_four, bt_write_patient_num, &patient_num),
  BT_GATT_CHARACTERISTIC(&bt_uuid_reset, 
    BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, 
    NULL, bt_reset, NULL),
  BT_GATT_CHARACTERISTIC(&bt_uuid_name, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, bt_change_brightness, NULL),
); 

/* status service: read storage capacity, potentially battery later on*/
BT_GATT_SERVICE_DEFINE(status_service, 
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_status_service),
  BT_GATT_CHARACTERISTIC(&bt_uuid_read_storage,//18,19
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    read_generic_four, NULL, &storage_percent_full),
    BT_GATT_CHARACTERISTIC(&bt_uuid_read_status,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    update_ble_status_register, NULL, &ble_status_register_send),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&bt_uuid_read_uptime,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    update_uptime, NULL, &uptime),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* update service: read ENMO updates */
BT_GATT_SERVICE_DEFINE(update_service,
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_update_service),
  BT_GATT_CHARACTERISTIC(&bt_uuid_enmo_notify, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

  BT_GATT_CHARACTERISTIC(&bt_uuid_enmothreshold_notify, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ , BT_GATT_PERM_READ,
    read_enmo_threshold, NULL, &enmo_threshold_packet),
  BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  );



/* This function sends a notification to a Client with the provided data,
given that the Client Characteristic Control Descripter has been set to Notify (0x1).
It also calls the on_sent() callback if successful*/

  /* 
    The attribute for the TX characteristic is used with bt_gatt_is_subscribed 
    to check whether notification has been enabled by the peer or not.
    Attribute table: 0 = Service, 1 = Primary service, 2 = RX, 3 = TX, 4 = CCC,.
  */

void tfMicro_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  const struct bt_gatt_attr *attr = &tfMicro_service.attrs[BLE_ATTR_TFMICRO_CHARACTERISTIC]; 
  struct bt_gatt_notify_params params = {
    .uuid   = BT_UUID_TFMICRO_TX,
    .attr   = attr,
    .data   = data,
    .len    = len,
    .func   = on_sent
  };
    
  // Check whether notifications are enabled or not
  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    // Send the notification
    if(bt_gatt_notify_cb(conn, &params)){
            printk("Error, unable to send notification\n");
    }
  }
  else{
      //  printk("Warning, notification not enabled on the selected attribute\n");
  }
}


uint8_t configRead[6] = {0,0,0,0,0,0};
uint8_t ppgQuality[4] = {0};
uint8_t accQuality[4] = {0};



uint8_t gyro_first_read = 0;
uint8_t magneto_first_read = 0;  
uint8_t ppgRead = 0;
bool ppgTFPass = false;


uint16_t sampleFreq = MAGNETO_SAMPLING_RATE;


struct ppgInfo my_ppgSensor;
struct ble_battery_info my_battery ;  // work-queue instance for batter level

struct motionInfo my_motionSensor; // work-queue instance for motion sensor
struct magnetoInfo my_magnetoSensor; // work-queue instance for magnetometer
struct orientationInfo my_orientaionSensor; // work-queue instance for orientation
struct bleDataPacket my_ppgDataSensor;


void write_status_register(bool value, int position){
    uint8_t* register_ptr = status_registers[position];
    *register_ptr = value;
}


bool read_status_register(int position){
    return *status_registers[position];
}

void update_uptime(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
    uptime = k_uptime_get() / 1000;
  
    return read_generic_four(conn, attr, buf, len, offset);
}


static ssize_t update_ble_status_register(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  for (int x = 0; x < num_of_status_registers; x++){
    ble_status_register_send[x] = *status_registers[x];
  }
  return read_generic_eight(conn, attr, buf, len, offset);
}

void rtc_handler(nrfx_rtc_int_type_t event_type){
  int work_queue_result;

  // Get the spurious cases out of the way...
  if (!collecting_data) { return; }
  if (event_type != NRFX_RTC_INT_TICK) { return; }

  // submit work to read gyro, acc, magnetometer and orientation
  my_motionSensor.magneto_first_read = magneto_first_read;
  my_motionSensor.pktCounter = global_counter;
  my_motionSensor.gyro_first_read = gyro_first_read;
  work_queue_result = k_work_submit(&my_motionSensor.work);
  if (work_queue_result != 1) { LOG_ERR("accel work queue was not submitted: %i", work_queue_result); }

  if(ppgRead == 0){
    my_ppgSensor.pktCounter = global_counter;
    my_ppgSensor.movingFlag = current_gyro_data.movingFlag;
    my_ppgSensor.ppgTFPass = ppgTFPass;
    work_queue_result = k_work_submit(&my_ppgSensor.work);
    if (work_queue_result != 1) { LOG_ERR("PPG work queue was not submitted: %i", work_queue_result); }
  }

  // ppgConfig.numCounts is derived from the RTC cadence.
  ppgRead = (ppgRead+1) % ppgConfig.numCounts;
  magneto_first_read = (magneto_first_read + 1) % (GYRO_SAMPLING_RATE/MAGNETO_SAMPLING_RATE);

  gyro_first_read = (gyro_first_read + 1) % (gyroConfig.tot_samples);

  global_counter++;
}


static void rtc_deinit(void){
  nrfx_rtc_disable(&rtc);
  nrfx_rtc_uninit(&rtc);
  motion_sleep();
  ppg_sleep();
}



static void rtc_init(void){
  nrfx_err_t          err;

  // Setup RTC0 (RTC1 used by Zephyr)
  // Initialize RTC: 32.768kHz / (IMU_RTC_PRESCALER + 1)
  // We will use the TICK interrupt
  nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
  config.prescaler = IMU_RTC_PRESCALER;
  config.interrupt_priority = IMU_RTC_IRQ_PRIORITY;
  err = nrfx_rtc_init(&rtc, &config, rtc_handler);
  if (err != NRFX_SUCCESS) { printk("nrfx_rtc_init() failed with: %d\n", err); }
  nrfx_rtc_tick_enable(&rtc, true);
  nrfx_rtc_enable(&rtc);
  IRQ_CONNECT(RTC0_IRQn, IMU_RTC_IRQ_PRIORITY, nrfx_rtc_0_irq_handler, NULL, 0);
  irq_enable(RTC0_IRQn);
}

void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param){
    
  LOG_INF("USB Status: %d", status);

}



void connected(struct bt_conn* conn, uint8_t err){
  struct bt_conn_info info; 
  char addr[BT_ADDR_LE_STR_LEN];

  my_connection = conn;
  if (err) {
    printk("Connection failed (err %u)\n", err);
    return;
  }
  else if(bt_conn_get_info(conn, &info))
    printk("Could not parse connection info\n");
  else{  
  // Start the timer and stop advertising and initialize all the modules
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Connection established!		\n\
      Connected to: %s					\n\
      Role: %u							\n\
      Connection interval: %u				\n\
      Slave latency: %u					\n\
      Connection supervisory timeout: %u	\n"
      , addr, info.role, info.le.interval, info.le.latency, info.le.timeout);
		
    
    
    connectedFlag=true;
    #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
    start_stop_device_collection(true);
    #endif
    
    
    
  }
}

void disconnected(struct bt_conn *conn, uint8_t reason){
  // Stop timer and do all the cleanup
  printk("Disconnected (reason %u)\n", reason);
  
  
  connectedFlag=false;

  #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
    start_stop_device_collection(false);
  #endif
}




void reset_device(bool reset_bad_blocks){
  
  if (!IS_ENABLED(CONFIG_USB_ALWAYS_ON)){
    usb_disable();
  } 
    //reset the flash memory first
  LOG_INF("Performing Chip Erase...\n");
  // get our flash device from device tree, which is defined in nrf5340dk_nrf5340_cpuapp.overlay
  struct device* flash_device = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
  if (device_is_ready(flash_device)){
    LOG_INF("flash dev eraseing... \n");
    reset_lock = true;
    #if CONFIG_DISK_DRIVER_RAW_NAND
    if (reset_bad_blocks){
      LOG_WRN("Erasing bad block table...");
      spi_nand_multi_chip_reset_bad_block(flash_device);
    }
    else{
      spi_nand_multi_chip_erase(flash_device);
    }
    #else
    #if !DT_NODE_HAS_PROP(DT_ALIAS(spi_flash0), size)
    #error "flash needs size property in order to be erased"
    #endif 
    LOG_INF("eraseing nor flash");
    int size = DT_PROP(DT_ALIAS(spi_flash0), size) / 8;
    flash_erase(flash_device, 0, size);
    #endif
    
    
    LOG_INF("Chip Erase Complete! Resetting");
    k_sleep(K_SECONDS(2));
  }
  else {
    LOG_ERR("Couldn't erase flash chip, device not ready.");
  }
   
  NVIC_SystemReset();

}

void start_stop_device_collection(uint8_t val){


  if (val != collecting_data){
    if (val){
      ppg_config();
      motion_config();

      #if CONFIG_DISK_DRIVER_RAW_NAND
      set_read_only(false);
      #endif

      #ifndef CONFIG_USB_ALWAYS_ON
        usb_disable();
      #endif
      // we sleep for a tiny bit to let the ppg and accel config power up, 
      //as we get junk values in the initial seconds of turning them on.
      // TODO: Flush the PPG buffer before starting collection, if we sleep and don't do this samples will accumulate!
      //k_sleep(K_MSEC(500));
      
      rtc_init();
      global_counter = 0;
      gyro_first_read = 0;
      magneto_first_read = 0;  
      ppgRead = 0;
      host_wants_collection = true;
      collecting_data = true;
      
    } 
    else{
      rtc_deinit();
      k_sleep(K_MSEC(500));
      close_all_files();
      enmo_sample_counter = 0;
      last_activated_trigger_counter = 0;

      #if CONFIG_DISK_DRIVER_RAW_NAND
      set_read_only(true);
      #endif

      #ifndef CONFIG_USB_ALWAYS_ON
      if (!security_lock){
        usb_enable(NULL);	
      }
      #endif

      collecting_data = false;
    
  }
}

}


bool check_valid_date_and_id(){
    
    return true;
}

static ssize_t write_enable_value(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute enable write, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	

	if (offset != 0) {
		LOG_WRN("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  }
  uint8_t val = *((uint8_t *)buff);
  LOG_INF("write: %i", val);
  host_wants_collection = val;
  if (!(battery_low && val)){

    start_stop_device_collection(val);
    
  }
  return len;
}

static ssize_t bt_write_date_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute time write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

  if (len != 8){
    LOG_WRN("invalid packet length for date: %i", len);
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  }

  uint64_t val = *((uint64_t *)buff);
  LOG_INF("writing: %llu", val);
  set_date_time_bt(val);
  return len;
}


static ssize_t bt_write_patient_num(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
  // date has to be 4 byte int to work.
  if (len != 4){
    LOG_WRN("invalid packet length for date: %i", len);
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  }

  int val = *((int *)buff);
  LOG_INF("new patient id write: %d", val);
  patient_num = val;
  return len;
}

//function from main
void storage_clear_led();



static ssize_t bt_reset(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
	LOG_INF("Write length: %i", len);
  if (len != 1){
    LOG_WRN("invalid packet length for reset: %i", len);
  }
  
  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  // check the bluetooth value entered for the correct code.
  uint8_t val = *((uint8_t *)buff);
  LOG_INF("entered code: %i", val);
  if ((val == 68 || val == 121 || val == 132) && !collecting_data){
    LOG_INF("Correct Code Entered, Resetting Device");
    LOG_INF("disconnecting bluetooth.. \n");
    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_le_adv_stop();
    connectedFlag=false;
    shutdown_filesystem();

    
    storage_clear_led();
    k_sleep(K_SECONDS(1));
    // 68 is for a whole reset, meaning we clear the flash memory of all data too.
    if (val == 68 || val == 132){
      reset_device(val == 132);
    }
    else {
      
      NVIC_SystemReset();
    }
    return NRFX_SUCCESS;  
  }
  
  return -1;
}

// Note: Currently does not work, more work is needed to allow dynamic runtime name changing.
static ssize_t bt_change_name(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  int status;
  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
	LOG_INF("Write length: %i", len);
  
  
  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  const char* val = ((const char*)buff);
  char new_name[30];
  memcpy(new_name, val, len);
  new_name[len] = '\0';
  LOG_INF("entered new name: %s", new_name);
  bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
  bt_le_adv_stop();
  status = bt_set_name(new_name);
  if (status == 0){
    LOG_INF("Sucessfully changed device name!");
  }
  

  const struct bt_le_adv_param v = {
      .id = BT_ID_DEFAULT,
      .sid = 0,
      .secondary_max_skip = 0,
      .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
      .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
      .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
      .peer = NULL};

  /*err = bt_le_adv_start(&v, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err)
    printk("Advertising failed to start (err %d)\n", err);
  else
  {
    printk("Advertising successfully started\n");
  }
  */
  k_sleep(K_SECONDS(1));
  NVIC_SystemReset();
  return 0;
  
  return -1;
}

void create_test_files_through_file_workqueue(struct k_work* work){
  storage_clear_led();
  create_test_files(100);
  blink_led(31);

}

void crash_device(){
    // can use:
    //k_oops();
    // or:
    volatile uint32_t *bad_ptr = NULL;
    *bad_ptr = 0xDEADBEEF; // This will trigger a CPU exception
}


static ssize_t bt_change_brightness(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
  uint16_t offset, uint8_t flags){
    LOG_INF("Attribute other settings write, handle: %u, conn: %p, length %i", attr->handle,
      (void *)conn, len);
  
    
    if (len != 1){
      LOG_WRN("invalid packet length: %i", len);
    }
    
    if (offset != 0) {
      LOG_INF("Write: Incorrect data offset");
      return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
  
    uint8_t val = *((uint8_t *)buff);
    LOG_INF("entered value: %i", val);
    if (!collecting_data){
      if (val == 0){
        LOG_INF("Turning on auto brightness");
        use_fixed_ppg_brightness = false;
      }
      else if (val > 0 && val < 121){
        LOG_INF("Turning on manual brightness");
        use_fixed_ppg_brightness = true;
        ppgConfig.green_intensity = val;
        ppgConfig.infraRed_intensity = val - 10;
      }
      else if (val >= 122){
        // if the value submitted to the brightness characteristic is 150 or 130, create test files, for testing the file system.
        if (val == 130 || val == 150 && !collecting_data){

          bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
          reset_lock = true;
          #ifndef CONFIG_USB_ALWAYS_ON
          usb_disable();
          #endif
          LOG_INF("Manual file creation");
          k_sleep(K_SECONDS(1));
          LOG_INF("begin");
          #if CONFIG_DISK_DRIVER_RAW_NAND
            set_read_only(false);
          #endif

          
          if (val == 150){
            storage_clear_led();
            create_test_files(500);
            blink_led(31);
          }
          else{
            LOG_INF("100 opt");
            //struct k_work work;
            //k_work_init(&work, create_test_files_through_file_workqueue);
            //k_work_submit_to_queue(&my_work_q, &work);
            create_test_files(100);
          }
          
          reset_lock = false;
          #if CONFIG_DISK_DRIVER_RAW_NAND
          set_read_only(true);
          #endif
          //bt_enable(bt_ready);
          #ifndef CONFIG_USB_ALWAYS_ON
          if (!security_lock){
          usb_enable(usb_status_cb);
          }
          #endif
          
          //NVIC_SystemReset();

        }
      }  
      return 0;
      
    }
    
    return -1;
  }


static ssize_t read_generic_one(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  
  const char* value = attr->user_data;
  //uint8_t space_left = storage_percent_full;
  //LOG_INF("space full: %i", space_left);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 1);

}

static ssize_t read_generic_four(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  
  const char* value = attr->user_data;
  //uint8_t space_left = storage_percent_full;
  //LOG_INF("space full: %i", space_left);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(int));

}

static ssize_t read_generic_eight(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  const char* value = attr->user_data;
  //uint8_t space_left = storage_percent_full;
  //LOG_INF("space full: %i", space_left);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(uint64_t));

}

static ssize_t read_enmo_threshold(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  const char* value = attr->user_data;
  //uint8_t space_left = storage_percent_full;
  //LOG_INF("space full: %i", space_left);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 9);

}

/* This function is called whenever the RX Characteristic has been written to by a Client */
ssize_t on_settings_change(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags){
  const uint8_t * buffer =(const uint8_t*) buf;
  
  printk("Received data, handle %d, conn %p, data: 0x", attr->handle, conn);
  for(uint8_t i = 0; i < len; i++){
        printk("%02X,", buffer[i]);
  }
  printk("\n");

  switch(buffer[0]){
    case BLE_CONFIG_SENSOR_ENABLE:
      // Enabling or disabling sensors
      if((buffer[1] & IMU_ENABLE) == IMU_ENABLE){
        gyroConfig.isEnabled = true;
        accelConfig.isEnabled = true;
        configRead[1] = configRead[1] | 0x02;
      }
      else if((buffer[1] & IMU_ENABLE) == 0x00){
        gyroConfig.isEnabled = false;
        accelConfig.isEnabled = false;
        configRead[1] = configRead[1] & 0xFD;     
      }
#if MAGNETOMETER_DATA_PATH_ENABLED
      if((buffer[1] & MAGNETOMETER_ENABLE) == MAGNETOMETER_ENABLE){
        magnetoConfig.isEnabled = true;
        configRead[1] = configRead[1] | MAGNETOMETER_ENABLE;
      }
      else if((buffer[1] & MAGNETOMETER_ENABLE) == 0x00){
        magnetoConfig.isEnabled = false;
        configRead[1] = configRead[1] & ((uint8_t)~MAGNETOMETER_ENABLE);
      }
#else
      magnetoConfig.isEnabled = false;
      configRead[1] = configRead[1] & ((uint8_t)~MAGNETOMETER_ENABLE);
#endif
      if((buffer[1] & PPG_ENABLE) == PPG_ENABLE){
        ppgConfig.isEnabled = true;
        configRead[1] = configRead[1] | 0x01; 
      }
      else if((buffer[1] & PPG_ENABLE) == 0x00){
        ppgConfig.isEnabled = false;
        configRead[1] = configRead[1] & 0xFE; 
      }     
      if((buffer[1] & ORIENTATION_ENABLE) == ORIENTATION_ENABLE){
        orientationConfig.isEnabled = true;
        configRead[1] = configRead[1] | 0x04; 
      }
      else if((buffer[1] & ORIENTATION_ENABLE) == 0x00){
        orientationConfig.isEnabled = false;
        configRead[1] = configRead[1] & 0xFB; 
      }
      if((buffer[2] & MOTION_BLE_ENABLE) == MOTION_BLE_ENABLE){
        accelConfig.txPacketEnable = true;
        gyroConfig.txPacketEnable = true;
        configRead[0] = configRead[0] | 0x02;
      }
      else if((buffer[2] & MOTION_BLE_ENABLE) == MOTION_BLE_ENABLE){
        accelConfig.txPacketEnable = false;
        gyroConfig.txPacketEnable = false;
        configRead[0] = configRead[0] & 0xFD;     
      }
#if MAGNETOMETER_DATA_PATH_ENABLED
      if((buffer[2] & MAGNETOMETER_BLE_ENABLE) == MAGNETOMETER_BLE_ENABLE){
        magnetoConfig.txPacketEnable = true;
        configRead[0] = configRead[0] | MAGNETOMETER_BLE_ENABLE;
      }
      else if((buffer[2] & MAGNETOMETER_BLE_ENABLE) == 0x00){
        magnetoConfig.txPacketEnable = false;
        configRead[0] = configRead[0] & ((uint8_t)~MAGNETOMETER_BLE_ENABLE);
      }
#else
      magnetoConfig.txPacketEnable = false;
      configRead[0] = configRead[0] & ((uint8_t)~MAGNETOMETER_BLE_ENABLE);
#endif
      if((buffer[2] & PPG_BLE_ENABLE) == PPG_BLE_ENABLE){
        ppgConfig.txPacketEnable = true;
        configRead[0] = configRead[0] | 0x01; 
      }
      else if((buffer[2] & PPG_BLE_ENABLE) == 0x00){
        ppgConfig.txPacketEnable = false;
        configRead[0] = configRead[0] & 0xFE; 
      }
      if((buffer[2] & ORIENTATION_BLE_ENABLE) == ORIENTATION_BLE_ENABLE){
        orientationConfig.txPacketEnable = true;
        configRead[0] = configRead[0] | 0x04; 
      }
      else if((buffer[2] & ORIENTATION_BLE_ENABLE) == 0x00){
        orientationConfig.txPacketEnable = false;
        configRead[0] = configRead[0] & 0xFB; 
      }
      if((buffer[2] & TFMICRO_BLE_ENABLE) == TFMICRO_BLE_ENABLE){
        tfMicroCoonfig.txPacketEnable = true;
        configRead[0] = configRead[0] | 0x08; 
      }
      else if((buffer[2] & TFMICRO_BLE_ENABLE) == 0x00){
        tfMicroCoonfig.txPacketEnable = false;
        configRead[0] = configRead[0] & 0xF7; 
      }
      break;
    case BLE_CONFIG_GYRO_SENSITIVITY:
      // configuring Gyroscope Full-scale
      if(buffer[1] == GYRO_250_DPS){
        gyroConfig.sensitivity = GYRO_FS_SEL_250;
        configRead[4] = 0x10 | (configRead[4]&0x0F);
      }
      else if(buffer[1] == GYRO_500_DPS){
        gyroConfig.sensitivity = GYRO_FS_SEL_500;
        configRead[4] = 0x20 | (configRead[4]&0x0F);
      }
      else if(buffer[1] == GYRO_1000_DPS){
        gyroConfig.sensitivity = GYRO_FS_SEL_1000;
        configRead[4] = 0x30 | (configRead[4]&0x0F);
      }
      else if(buffer[1] == GYRO_2000_DPS){
        gyroConfig.sensitivity = GYRO_FS_SEL_2000;
        configRead[4] = 0x40 | (configRead[4]&0x0F);
      }
      else{
        gyroConfig.sensitivity = GYRO_FS_SEL_500;
        configRead[4] = 0x10 | (configRead[4]&0x0F);
      }
      motionSensitivitySampling_config();
      break;
    case BLE_CONFIG_ACC_SENSITIVITY:
      // configuring Accelerometer Full-scale
      if(buffer[1] == ACC_2G){
        accelConfig.sensitivity = ACCEL_FS_SEL_2g;
        configRead[4] = 0x01 | (configRead[4]&0xF0);
      }
      else if(buffer[1] == ACC_4G){
        accelConfig.sensitivity = ACCEL_FS_SEL_4g;
        configRead[4] = 0x02 | (configRead[4]&0xF0);
      }
      else if(buffer[1] == ACC_8G){
        accelConfig.sensitivity = ACCEL_FS_SEL_8g;
        configRead[4] = 0x03 | (configRead[4]&0xF0);
      }
      else if(buffer[1] == ACC_16G){
        accelConfig.sensitivity = ACCEL_FS_SEL_16g;
        configRead[4] = 0x04 | (configRead[4]&0xF0);
      }
      else{
        accelConfig.sensitivity = ACCEL_FS_SEL_4g;
        configRead[4] = 0x01 | (configRead[4]&0xF0);
      }
      motionSensitivitySampling_config();
      break;
    case BLE_CONFIG_LED_INTENSITY_GREEN:
      // configuring PPG Green intensity
      ppgConfig.green_intensity = buffer[1];
      configRead[2] = ppgConfig.green_intensity;
      ppg_changeIntensity();
      break;
    case BLE_CONFIG_LED_INTENSITY_IR:
      // configuring PPG IR intensity
      ppgConfig.infraRed_intensity = buffer[1];
      configRead[3] = ppgConfig.infraRed_intensity;
      ppg_changeIntensity();
      break;
    case BLE_CONFIG_SAMPLING_RATE_ACC:
      // IMU integrates at 512 Hz and emits/stores one record every 16 ticks.
      // Ignore host rate selections so the firmware has one motion cadence.
      accelConfig.sample_bw = IMU_FIXED_ACCEL_DLPFCFG;
      gyroConfig.tot_samples = IMU_FIXED_ACCEL_REPORT_DIVISOR;
      sampleFreq = MAGNETO_SAMPLING_RATE;
      configRead[5] = MOTION_FIXED_32HZ_STATUS | (configRead[5]&0xF0);
      motionSensitivitySampling_config();
      break;
    case BLE_CONFIG_SAMPLING_RATE_PPG:
      // PPG sampling is fixed at 512 sps with 2-sample averaging.
      // Ignore host rate selections so the firmware has a single PPG cadence.
      ppg_changeSamplingRate();
      configRead[5] = PPG_FIXED_256HZ_STATUS | (configRead[5]&0x0F);
      break;
    default: 
      printk("Error, CCCD has been set to an invalid value");        
  }
  return len;
}

/* This function is called whenever a Notification has been sent by the TX Characteristic */
static void on_sent(struct bt_conn* conn, void* user_data){
  ARG_UNUSED(user_data);
  const bt_addr_le_t * addr = bt_conn_get_dst(conn);
    /*    
	//printk("Data sent to Address 0x %02X %02X %02X %02X %02X %02X \n", addr->a.val[0]
                                                                    , addr->a.val[1]
                                                                    , addr->a.val[2]
                                                                    , addr->a.val[3]
                                                                    , addr->a.val[4]
                                                                    , addr->a.val[5]);*/
}

/* This function is called whenever the CCCD register has been changed by the client*/
void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value){
  
  ARG_UNUSED(attr);
  switch(value){
    case BT_GATT_CCC_NOTIFY: 
      // Start sending stuff!
      break;
    case BT_GATT_CCC_INDICATE: 
      // Start sending stuff via indications
      break;

    case 0: 
      // Stop sending stuff
      break;
        
    default: 
      printk("Error, CCCD has been set to an invalid value");     
  }
}
                        


/* This function sends a notification to a Client with the provided data,
given that the Client Characteristic Control Descripter has been set to Notify (0x1).
It also calls the on_sent() callback if successful*/
void enmo_send(struct bt_conn* conn, uint8_t* data, uint8_t len){


  
  // the number 2 acesses the 2rd attribute in the service, enmo characteristic 
  const struct bt_gatt_attr *attr = &update_service.attrs[2];
  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    LOG_INF("sending ennmo...");
    int ret = bt_gatt_notify(conn, attr, data, len);
    if (ret != 0){
      printk("Error, unable to send notification\n");
    }
  } 

}

/* This function sends a notification to a Client with the provided data,
given that the Client Characteristic Control Descripter has been set to Notify (0x1).
It also calls the on_sent() callback if successful*/
void enmo_threshold_send(uint8_t* data, uint8_t len){

  // the number 2 acesses the 2rd attribute in the service, enmo characteristic 
  const struct bt_gatt_attr *attr = &update_service.attrs[4];
  if(bt_gatt_is_subscribed(my_connection, attr, BT_GATT_CCC_NOTIFY)) {
    LOG_INF("sending ennmo...");
    int ret = bt_gatt_notify(my_connection, attr, data, len);
    if (ret != 0){
      printk("Error, unable to send notification\n");
    }
  } 

}


int status_reg_ble_notification(){

  for (int x = 0; x < num_of_status_registers; x++){
    ble_status_register_send[x] = *status_registers[x];
  }
  const struct bt_gatt_attr *attr = &status_service.attrs[4];
  if(bt_gatt_is_subscribed(my_connection, attr, BT_GATT_CCC_NOTIFY)) {
    LOG_INF("sending status reg...");
    int ret = bt_gatt_notify(my_connection, attr, ble_status_register_send, sizeof(ble_status_register_send));
    if (ret != 0){
      printk("Error, unable to send notification\n");
    }
  } 
}

int storage_ble_notification(uint8_t* data, uint8_t len){
  // if there is no notification, then we technically have an error.
  int ret = -1;
  const struct bt_gatt_attr *attr = &status_service.attrs[2];
  if(bt_gatt_is_subscribed(my_connection, attr, BT_GATT_CCC_NOTIFY)) {
    LOG_INF("sending ennmo...");
    int ret = bt_gatt_notify(my_connection, attr, data, len);
    if (ret != 0){
      printk("Error, unable to send notification\n");
    }
  }
  return ret; 
}


int general_ble_notification(uint8_t* data, uint8_t len, int service, int characteristic){

  int ret = 0;
  
  const struct bt_gatt_service_static* selected_service;
  switch (service){
    case 0:
      selected_service = &tfMicro_service;

  }
  const struct bt_gatt_attr *attr; //= selected_service->attrs[4];
  if(bt_gatt_is_subscribed(my_connection, attr, BT_GATT_CCC_NOTIFY)) {
    LOG_INF("sending ennmo...");
    ret = bt_gatt_notify(my_connection, attr, data, len);
    if (ret != 0){
      printk("Error, unable to send notification\n");
    }
  }
  return ret; 
}



void motion_notify(struct k_work *item){
  struct bleDataPacket* the_device = CONTAINER_OF(item, struct bleDataPacket, work);
        
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;
  printk("%i", packetLength);
  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  acc_send(my_connection, the_device->dataPacket, the_device->packetLength);
  #else
  
  memcpy(&dataPacket[4], &global_counter, sizeof(global_counter));
  enmo_send(my_connection, the_device->dataPacket, the_device->packetLength);
  #endif

}

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES


void acc_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  
  const struct bt_gatt_attr *attr = &data_service.attrs[BLE_ATTR_ACC_CHARACTERISTIC]; 
  struct bt_gatt_notify_params params = {
    .uuid   = BT_UUID_ACC_GYRO_TX,
    .attr   = attr,
    .data   = data,
    .len    = len,
    .func   = on_sent
  };

    //printk("attr Counts = %d,selected = %d,handle=%d\n",tfMicro_service.attr_count,BLE_ATTR_ACC_CHARACTERISTIC,attr->handle);
    
  // Check whether notifications are enabled or not

  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    // Send the notification
    
    if(bt_gatt_notify_cb(conn, &params)){
            printk("Error, unable to send notification\n");
    }
    
  }
  else{
      //  printk("Warning, notification not enabled on the selected attribute\n");
  }

}


void ppg_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  const struct bt_gatt_attr *attr = &data_service.attrs[2]; 
  struct bt_gatt_notify_params params = {
    .uuid   = BT_UUID_PPG_TX,
    .attr   = attr,
    .data   = data,
    .len    = len,
    .func   = on_sent
  };
  
  // Check whether notifications are enabled or not
  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    // Send the notification
    if(bt_gatt_notify_cb(conn, &params)){
            LOG_WRN("Error, unable to send notification\n");
    }
  }
  else{
        //printk("Warning, notification not enabled on the selected attribute\n");
  }
}

void magnetometer_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  const struct bt_gatt_attr *attr = &tfMicro_service.attrs[BLE_ATTR_MAGNETO_CHARACTERISTIC]; 
  struct bt_gatt_notify_params params = {
    .uuid   = BT_UUID_MAGNETO_TX,
    .attr   = attr,
    .data   = data,
    .len    = len,
    .func   = on_sent
  };
    
  // Check whether notifications are enabled or not
  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    // Send the notification
    if(bt_gatt_notify_cb(conn, &params)){
            //printk("Error, unable to send notification\n");
    }
  }
  else{
        //printk("Warning, notification not enabled on the selected attribute\n");
  }
}

void orientation_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  const struct bt_gatt_attr *attr = &tfMicro_service.attrs[BLE_ATTR_ORIENTATION_CHARACTERISTIC]; 
  struct bt_gatt_notify_params params = {
    .uuid   = BT_UUID_ORIENTATION_TX,
    .attr   = attr,
    .data   = data,
    .len    = len,
    .func   = on_sent
  };
    
  // Check whether notifications are enabled or not
  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    // Send the notification
    if(bt_gatt_notify_cb(conn, &params)){
            //printk("Error, unable to send notification\n");
    }
  }
  else{
        //printk("Warning, notification not enabled on the selected attribute\n");
  }
}
void tfMicro_notify(struct k_work *item){
  struct bleDataPacket* the_device =  ((struct bleDataPacket *)(((char *)(item)) - offsetof(struct bleDataPacket, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  tfMicro_service_send(my_connection, the_device->dataPacket, TFMICRO_DATA_LEN);
}



void magneto_notify(struct k_work *item){
  struct magnetoInfo* the_device=  ((struct magnetoInfo *)(((char *)(item)) - offsetof(struct magnetoInfo, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  magnetometer_send(my_connection, the_device->dataPacket, MAGNETOMETER_DATA_LEN);
}
void orientation_notify(struct k_work *item){
  struct orientationInfo* the_device=  ((struct orientationInfo *)(((char *)(item)) - offsetof(struct orientationInfo, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  orientation_send(my_connection, the_device->dataPacket, ORIENTATION_DATA_LEN);
}


void ppgData_notify(struct k_work *item){
  struct bleDataPacket* the_device=  ((struct bleDataPacket *)(((char *)(item)) - offsetof(struct bleDataPacket, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  ppg_send(my_connection, the_device->dataPacket, PPG_DATA_UNFILTER_LEN);
}

#endif



static ssize_t configSet(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  uint8_t *value1 = (uint8_t *)attr->user_data;
  uint8_t *rsp;

  rsp = value1;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, rsp,CONFIG_RX_DATA_LEN);
}
static ssize_t read_ppg_quality(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  uint8_t *value1 = (uint8_t *)attr->user_data;
  uint8_t *rsp;

  rsp = value1;

  return bt_gatt_attr_read(conn, attr, buf, len, offset, rsp,PPGQUALITY_DATA_LEN);
}
static ssize_t read_acc_quality(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  uint8_t *value1 = (uint8_t *)attr->user_data;
  uint8_t *rsp;

  rsp = value1;

  return bt_gatt_attr_read(conn, attr, buf, len, offset, rsp,ACCQUALITY_DATA_LEN);
}
