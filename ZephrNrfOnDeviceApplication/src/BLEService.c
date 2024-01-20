
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>
//#include <sys/byteorder.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include "ppgSensor.h"
#include "imuSensor.h"
#include "batteryMonitor.h"
#include "filesystem/zephyrfilesystem.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
#include <nrfx_timer.h>
#include "BLEService.h"
#define CONFIG_NAME "Configure sensor"
#define TFMICRO_NAME "Micromarker Heart-rate"
#define PPG_NAME "PPG Sensor"
#define ACC_NAME "Acc and Gyroscope"
#define MAGNETO_NAME "Magnetometer"
#define ORIENTATION_NAME "Orientation vector"




LOG_MODULE_REGISTER(user_bluetooth);



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
 struct bt_uuid_128 bt_uuid_tfmicro = BT_UUID_INIT_128(TFMICRO_SERVICE_UUID);
 struct bt_uuid_128 bt_uuid_config_rx = BT_UUID_INIT_128(RX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_tfmicro_tx = BT_UUID_INIT_128(TF_HR_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_ppg_tx = BT_UUID_INIT_128(PPG_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_acc_gyro_tx = BT_UUID_INIT_128(ACC_GRYO_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_magneto_tx = BT_UUID_INIT_128(MAGNETO_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_orientation_tx = BT_UUID_INIT_128(ORIENTATION_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_ppg_quality = BT_UUID_INIT_128(PPG_QUALITY_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_acc_quality = BT_UUID_INIT_128(ACC_QUALITY_CHARACTERISTIC_UUID);
#define BT_UUID_TFMICRO_SERVICE      (struct bt_uuid *)(&bt_uuid_tfmicro)
#define BT_UUID_TFMICRO_CONFIG_RX   (struct bt_uuid *)(&bt_uuid_config_rx)
#define BT_UUID_TFMICRO_TX   (struct bt_uuid *)(&bt_uuid_tfmicro_tx)
#define BT_UUID_PPG_TX   (struct bt_uuid *)(&bt_uuid_ppg_tx)
#define BT_UUID_ACC_GYRO_TX   (struct bt_uuid *)(&bt_uuid_acc_gyro_tx)
#define BT_UUID_MAGNETO_TX   (struct bt_uuid *)(&bt_uuid_magneto_tx)
#define BT_UUID_ORIENTATION_TX   (struct bt_uuid *)(&bt_uuid_orientation_tx)
#define BT_UUID_PPG_QUALITY   (struct bt_uuid *)(&bt_uuid_ppg_quality)
#define BT_UUID_ACC_QUALITY   (struct bt_uuid *)(&bt_uuid_acc_quality)
//#else
struct bt_uuid_128 bt_uuid_control = BT_UUID_INIT_128(TFMICRO_SERVICE_UUID);
struct bt_uuid_128 bt_enabledisable = BT_UUID_INIT_128(PPG_TX_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_write_enable = BT_UUID_INIT_128(WRITE_ENABLE_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_datetime = BT_UUID_INIT_128(WRITE_DATE_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_patientnum = BT_UUID_INIT_128(WRITE_PATIENT_CHARACTERISTIC_UUID);
//#endif
struct bt_uuid_128 bt_uuid_status_service = BT_UUID_INIT_128(STATUS_SERVICE_UUID);
struct bt_uuid_128 bt_uuid_read_storage = BT_UUID_INIT_128(READ_STORAGE_LEFT_UUID);
struct bt_uuid_128 bt_uuid_update_service = BT_UUID_INIT_128(UPDATE_SERVICE_UUID);
struct bt_uuid_128 bt_uuid_enmo_notify = BT_UUID_INIT_128(NOTIFY_ENMO_CHARACTERISTIC_UUID);

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
/* TF micro Button Service Declaration and Registration */
BT_GATT_SERVICE_DEFINE(tfMicro_service,
  BT_GATT_PRIMARY_SERVICE(BT_UUID_TFMICRO_SERVICE), //0
  /*BT_GATT_CHARACTERISTIC(BT_UUID_TFMICRO_CONFIG_RX, //1,2
    BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
    configSet, on_receive, configRead),
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
#else
BT_GATT_SERVICE_DEFINE(tfMicro_service,
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_control),
  BT_GATT_CHARACTERISTIC(&bt_uuid_write_enable,//18,19
    BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE,
    NULL, write_enable_value, NULL),
  BT_GATT_CHARACTERISTIC(&bt_uuid_datetime, 
    BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE,
    NULL, bt_write_date_time, NULL),
  BT_GATT_CHARACTERISTIC(&bt_uuid_patientnum, 
    BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE,
    NULL, bt_write_patient_num, NULL),
); 

BT_GATT_SERVICE_DEFINE(status_service, 
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_status_service),
  BT_GATT_CHARACTERISTIC(&bt_uuid_read_storage,//18,19
    BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
    read_storage_left, NULL, &storage_percent_full),
);


BT_GATT_SERVICE_DEFINE(update_service,
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_update_service),
  BT_GATT_CHARACTERISTIC(&bt_uuid_enmo_notify, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
  );
#endif


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

bool connectedFlag = false;
bool collecting_data = false;

uint16_t sampleFreq=25;


struct ppgInfo my_ppgSensor;
struct batteryInfo my_battery ;  // work-queue instance for batter level

struct TfMicroInfo my_HeartRateEncoder;  // work-queue instance for tflite notifications
struct motionInfo my_motionSensor; // work-queue instance for motion sensor
struct magnetoInfo my_magnetoSensor; // work-queue instance for magnetometer
struct orientationInfo my_orientaionSensor; // work-queue instance for orientation
struct ppgDataInfo my_ppgDataSensor;


static const nrfx_timer_t timer_global = NRFX_TIMER_INSTANCE(1); // Using TIMER1 as TIMER 0 is used by RTOS for blestruct device *spi_dev_imu;
#define TIMER_MS 5
#define TIMER_PRIORITY 1

static void timer_deinit(void){
  nrfx_timer_disable(&timer_global);
  nrfx_timer_uninit(&timer_global);	
  motion_sleep();
  ppg_sleep();
}

void timer_handler(nrf_timer_event_t event_type, void* p_context){
  LOG_DBG("Timer Executing");
  if(collecting_data == true){
    switch (event_type){
      case NRF_TIMER_EVENT_COMPARE0:
        
        // submit work to read gyro, acc, magnetometer and orientation
        my_motionSensor.magneto_first_read = magneto_first_read;
        my_motionSensor.pktCounter = global_counter;
        my_motionSensor.gyro_first_read = gyro_first_read;
        k_work_submit(&my_motionSensor.work);

        // Executes every 2 seconds to send battery level
        if(global_counter % 400 == 0) 
          //k_work_submit(&my_battery.work);
    
        if(ppgRead  == 0){
          my_ppgSensor.pktCounter = global_counter;
          my_ppgSensor.movingFlag = gyroData1.movingFlag;
          my_ppgSensor.ppgTFPass = ppgTFPass;
          //k_work_submit(&my_ppgSensor.work);
        }  
        // Executes every 8 seconds to send compressed signal
        
        ppgRead = (ppgRead+1) % ppgConfig.numCounts;
        magneto_first_read = (magneto_first_read +1) % (GYRO_SAMPLING_RATE/MAGNETO_SAMPLING_RATE);
        gyro_first_read = (gyro_first_read + 1) % (gyroConfig.tot_samples);
        
        break;

      default:
              //Do nothing.
        break;
    }
  }
}

static void timer_init(void){
printk("timer init\n");
  uint32_t time_ticks;
  nrfx_err_t          err;
  nrfx_timer_config_t timer_cfg = {
          .frequency = NRF_TIMER_FREQ_1MHz,
          .mode      = NRF_TIMER_MODE_TIMER,
          .bit_width = NRF_TIMER_BIT_WIDTH_24,
          .interrupt_priority = TIMER_PRIORITY,
          .p_context = NULL,
  };  
  uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
  nrfx_timer_config_t config = timer_cfg; //NRFX_TIMER_DEFAULT_CONFIG(1000000);
  err = nrfx_timer_init(&timer_global, &config, timer_handler);
  if (err != NRFX_SUCCESS) {
          printk("nrfx_timer_init failed with: %d\n", err);
  }
  else
          printk("nrfx_timer_init success with: %d\n", err);
  time_ticks = nrfx_timer_ms_to_ticks(&timer_global, TIMER_MS);
  printk("time ticks = %d\n", time_ticks);


  nrfx_timer_extended_compare(&timer_global, NRF_TIMER_CC_CHANNEL0, time_ticks \ 
   , NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrfx_timer_enable(&timer_global);
  printk("timer initialized\n");

}



#define MAX_TRANSMIT_SIZE 250//TODO figure this out


uint8_t data_rx[MAX_TRANSMIT_SIZE];
uint8_t data_tx[MAX_TRANSMIT_SIZE];
struct tfMicro_configData tfMicroCoonfig;
int tfMicro_service_init(void){
  int err=0;

  memset(&data_rx, 0,MAX_TRANSMIT_SIZE);
  memset(&data_tx, 0,MAX_TRANSMIT_SIZE);
  return err;
}

void connected(struct bt_conn *conn, uint8_t err){
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
    
    
    
  }
}

void disconnected(struct bt_conn *conn, uint8_t reason){
  // Stop timer and do all the cleanup
  printk("Disconnected (reason %u)\n", reason);
  
  
  connectedFlag=false;
}

static ssize_t write_enable_value(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute write, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	
	LOG_INF("Write length: %i", len);

	if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  }
  uint8_t val = *((uint8_t *)buff);
  LOG_INF("write: %i", val);

  if (val){
    ppg_config();
    motion_config();

    timer_init();
    global_counter = 0;
    gyro_first_read = 0;
    magneto_first_read = 0;  
    ppgRead = 0;
    collecting_data = true;
  } 
  else{
    timer_deinit();
    close_all_files();
    usb_enable(NULL);	
    collecting_data = false;
  }
}

static ssize_t bt_write_date_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
	LOG_INF("Write length: %i", len);
  if (len != 8){
    LOG_WRN("invalid packet length for date: %i", len);
  }

  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  }

  uint64_t val = *((uint64_t *)buff);
  LOG_INF("write: %llu", val);
  set_date_time_bt(val);
}


static ssize_t bt_write_patient_num(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
	LOG_INF("Write length: %i", len);
  if (len != 4){
    LOG_WRN("invalid packet length for date: %i", len);
  }

  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);

  }

  int val = *((int *)buff);
  LOG_INF("write: %llu", val);
  patient_num = val;
}


static ssize_t bt_turn_off(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags){
  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
	LOG_INF("Write length: %i", len);
  if (len != 4){
    LOG_WRN("invalid packet length for date: %i", len);
  }

  NVIC_SystemReset();
}



static ssize_t read_storage_left(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  
  const char *value = attr->user_data;
  //uint8_t space_left = storage_percent_full;
  //LOG_INF("space full: %i", space_left);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(storage_percent_full));

  }
/* This function is called whenever the RX Characteristic has been written to by a Client */
ssize_t on_receive(struct bt_conn *conn,
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
      if((buffer[1] & MAGNETOMETER_ENABLE) == MAGNETOMETER_ENABLE){
        magnetoConfig.isEnabled = true;
        configRead[1] = configRead[1] | 0x10; 
      }
      else if((buffer[1] & MAGNETOMETER_ENABLE) == 0x00){
        magnetoConfig.isEnabled = false;
        configRead[1] = configRead[1] & 0xEF;        
      }
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
      if((buffer[2] & MAGNETOMETER_BLE_ENABLE) == MAGNETOMETER_BLE_ENABLE){
        magnetoConfig.txPacketEnable = true;
        configRead[0] = configRead[0] | 0x10; 
      }
      else if((buffer[2] & MAGNETOMETER_BLE_ENABLE) == 0x00){
        magnetoConfig.txPacketEnable = false;
        configRead[0] = configRead[0] & 0xEF; 
      }
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
      // configuring Gyroscope sampling-rate
      if(buffer[1] == MOTION_25_FS){
        accelConfig.sample_bw = ACCEL_DLPFCFG_12HZ;
        gyroConfig.tot_samples = 8;
        configRead[5] = 0x04 | (configRead[5]&0xF0);
      }
      else if(buffer[1] == MOTION_50_FS){
        accelConfig.sample_bw = ACCEL_DLPFCFG_24HZ;
        gyroConfig.tot_samples = 4;
        configRead[5] = 0x03 | (configRead[5]&0xF0);
      }
      else if(buffer[1] == MOTION_100_FS){
        accelConfig.sample_bw = ACCEL_DLPFCFG_50HZ;
        gyroConfig.tot_samples = 2;
        configRead[5] = 0x02 | (configRead[5]&0xF0);
      }
      else if(buffer[1] == MOTION_200_FS){
        accelConfig.sample_bw = ACCEL_DLPFCFG_50HZ;
        gyroConfig.tot_samples = 1;
        configRead[5] = 0x01 | (configRead[5]&0xF0);
      }
      else{
        accelConfig.sample_bw = ACCEL_DLPFCFG_12HZ;
        gyroConfig.tot_samples = 8;
        configRead[5] = 0x04 | (configRead[5]&0xF0);
      }
      motionSensitivitySampling_config();
      break;
    case BLE_CONFIG_SAMPLING_RATE_PPG:
      // configuring PPG sampling-rate
      if(buffer[1] == PPG_25_FS){ 
        ppgConfig.sample_avg = PPG_SMP_AVE_16;
        ppgConfig.numCounts = 8;
        configRead[5] = 0x40 | (configRead[5]&0x0F);
        timeWindow = 50;
        high_pass_filter_init_25();
      }
      else if(buffer[1] == PPG_50_FS){
        ppgConfig.sample_avg = PPG_SMP_AVE_8;
        ppgConfig.numCounts = 4;
        configRead[5] = 0x30 | (configRead[5]&0x0F);
        timeWindow = 100;
        high_pass_filter_init_50();
      }
      else if(buffer[1] == PPG_100_FS){
        ppgConfig.sample_avg = PPG_SMP_AVE_4;
        ppgConfig.numCounts = 2;
        configRead[5] = 0x20 | (configRead[5]&0x0F);
        timeWindow = 200;
        high_pass_filter_init_100();
      }
      else if(buffer[1] == PPG_200_FS){
        ppgConfig.sample_avg = PPG_SMP_AVE_2;
        ppgConfig.numCounts = 1;
        configRead[5] = 0x10 | (configRead[5]&0x0F);
        timeWindow = 400;
        high_pass_filter_init_200();
      }
      else{
        ppgConfig.sample_avg = PPG_SMP_AVE_16;
        ppgConfig.numCounts = 8;
        high_pass_filter_init_25();
        timeWindow = 50;
        configRead[5] = 0x40 | (configRead[5]&0x0F);
      }
      ppg_changeSamplingRate();
      break;
    default: 
      printk("Error, CCCD has been set to an invalid value");        
  }
  return len;
}

/* This function is called whenever a Notification has been sent by the TX Characteristic */
static void on_sent(struct bt_conn *conn, void *user_data){
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
void ppg_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  const struct bt_gatt_attr *attr = &tfMicro_service.attrs[BLE_ATTR_PPG_CHARACTERISTIC]; 
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
            //printk("Error, unable to send notification\n");
    }
  }
  else{
        //printk("Warning, notification not enabled on the selected attribute\n");
  }
}

void acc_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  
  const struct bt_gatt_attr *attr = &tfMicro_service.attrs[BLE_ATTR_ACC_CHARACTERISTIC]; 
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

void enmo_send(struct bt_conn* conn, const uint8_t* data, uint16_t len){

  const struct bt_gatt_attr *attr = &update_service.attrs[2];
  if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
    LOG_INF("sending ennmo...");
    int ret = bt_gatt_notify(conn, attr, data, len);
    if (ret != 0){
      printk("Error, unable to send notification\n");
    }
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
  struct TfMicroInfo* the_device =  ((struct TfMicroInfo *)(((char *)(item)) - offsetof(struct TfMicroInfo, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  tfMicro_service_send(my_connection, the_device->dataPacket, TFMICRO_DATA_LEN);
}
void motion_notify(struct k_work *item){
  struct motionSendInfo* the_device=  ((struct motionSendInfo *)(((char *)(item)) - offsetof(struct motionSendInfo, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  acc_send(my_connection, the_device->dataPacket, the_device->packetLength);
  #else
  enmo_send(my_connection, the_device->dataPacket, the_device->packetLength);
  #endif

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
  struct ppgDataInfo* the_device=  ((struct ppgDataInfo *)(((char *)(item)) - offsetof(struct ppgDataInfo, work)));
  
  uint8_t *dataPacket = the_device->dataPacket;
  uint8_t packetLength = the_device->packetLength;

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  ppg_send(my_connection, the_device->dataPacket, PPG_DATA_UNFILTER_LEN);
}





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