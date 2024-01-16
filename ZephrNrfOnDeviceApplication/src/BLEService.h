#ifndef BLESERVICE_H_
#define BLESERVICE_H_
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include "common.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

extern uint8_t gyro_first_read;
extern uint8_t magneto_first_read;  
extern uint8_t ppgRead;
extern bool ppgTFPass;

extern bool connectedFlag;
extern bool collecting_data;


#define TFMICRO_DATA_LEN 66
#define PPG_DATA_LEN 18
#define PPG_DATA_UNFILTER_LEN 12
#define ACC_GYRO_DATA_LEN 20
#define MAGNETOMETER_DATA_LEN 8
#define ORIENTATION_DATA_LEN 18
#define CONFIG_RX_DATA_LEN 6
#define PPGQUALITY_DATA_LEN 4
#define ACCQUALITY_DATA_LEN 4

#define BLE_ATTR_PRIMARY_SERVICE 0
#define BLE_ATTR_CONFIG_CHARACTERISTIC 1
#define BLE_ATTR_TFMICRO_CHARACTERISTIC 4
#define BLE_ATTR_PPG_CHARACTERISTIC 8
#define BLE_ATTR_ACC_CHARACTERISTIC 13
#define BLE_ATTR_MAGNETO_CHARACTERISTIC 18
#define BLE_ATTR_ORIENTATION_CHARACTERISTIC 22


// BLE CONFIG COMMAND
#define BLE_CONFIG_SENSOR_ENABLE 0x00
#define BLE_CONFIG_GYRO_SENSITIVITY 0x01
#define BLE_CONFIG_ACC_SENSITIVITY 0x02
#define BLE_CONFIG_LED_INTENSITY_GREEN 0x03
#define BLE_CONFIG_LED_INTENSITY_IR 0x04
#define BLE_CONFIG_SAMPLING_RATE_ACC 0x05
#define BLE_CONFIG_SAMPLING_RATE_PPG 0x06

// BLE CONFIG DATA
#define IMU_ENABLE 0x02
#define MOTION_BLE_ENABLE 0x02
#define MAGNETOMETER_ENABLE 0x10
#define MAGNETOMETER_BLE_ENABLE 0x10
#define PPG_ENABLE 0x01
#define PPG_BLE_ENABLE 0x01
#define ORIENTATION_ENABLE 0x04
#define ORIENTATION_BLE_ENABLE 0x04
#define TFMICRO_ENABLE 0x08
#define TFMICRO_BLE_ENABLE 0x08
#define GYRO_250_DPS 0x00
#define GYRO_500_DPS 0x01
#define GYRO_1000_DPS 0x02
#define GYRO_2000_DPS 0x03
#define ACC_2G 0x00
#define ACC_4G 0x01
#define ACC_8G 0x02
#define ACC_16G 0x03
#define MOTION_200_FS 0x00
#define MOTION_100_FS 0x01
#define MOTION_50_FS 0x02
#define MOTION_25_FS 0x03
#define PPG_200_FS 0x01
#define PPG_100_FS 0x02
#define PPG_50_FS 0x03
#define PPG_25_FS 0x04

// Main Service UUID 
#define TFMICRO_SERVICE_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C,\
  0xE2, 0x48, 0x81, 0x1D, 0x22, 0x5D, 0x39, 0xDA      

// Configuration characteristic UUID
#define RX_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x21, 0xC9, 0x39, 0xDA       

// TF micro PPG HR characteristic UUID
#define TF_HR_TX_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x22, 0xC9, 0x39, 0xDA       
	
// PPG data characteristic UUID
#define PPG_TX_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x23, 0xC9, 0x39, 0xDA         
                                           
// Acc and Gyro data characteristic UUID
#define ACC_GRYO_TX_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x24, 0xC9, 0x39, 0xDA   

// Magnetometer data characteristic UUID
#define MAGNETO_TX_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x25, 0xC9, 0x39, 0xDA   

// Orientation data characteristic UUID
#define ORIENTATION_TX_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x26, 0xC9, 0x39, 0xDA  
   
// PPG Singal quality descriptor characteristic UUID
#define PPG_QUALITY_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x27, 0xC9, 0x39, 0xDA   

// Acc Singal quality descriptor characteristic UUID
#define ACC_QUALITY_CHARACTERISTIC_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x28, 0xC9, 0x39, 0xDA   

#define WRITE_ENABLE_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x31, 0xC9, 0x39, 0xDA

#define WRITE_DATE_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x32, 0xC9, 0x39, 0xDA

#define WRITE_PATIENT_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x33, 0xC9, 0x39, 0xDA

#define STATUS_SERVICE_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x40, 0xC9, 0x39, 0xDA

extern uint8_t configRead[6];
extern uint8_t ppgQuality[4];
extern uint8_t accQuality[4];

/** @brief Callback type for when new data is received. */
typedef void (*data_rx_cb_t)(uint8_t *data, uint8_t length);

/** @brief Callback struct used by the tfMicro_service Service. */
struct tfMicro_service_cb {
  /** Data received callback. */
  data_rx_cb_t    data_rx_cb;
};
struct TfMicroInfo {
    struct k_work work;
    uint8_t *dataPacket;
    uint8_t packetLength;
}; 

struct motionSendInfo {
    struct k_work work;
    uint8_t *dataPacket;
    uint8_t packetLength;
}; 
extern struct TfMicroInfo my_HeartRateEncoder;  // work-queue instance for tflite notifications
extern struct motionSendInfo my_motionData;  // work-queue instance for tflite notifications

void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

int tfMicro_service_init(void);

static void on_sent(struct bt_conn *conn, void *user_data);
void tfMicro_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void tfMicro_notify(struct k_work *);
void ppg_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void acc_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void magnetometer_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void orientation_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void motion_notify(struct k_work *item);
void magneto_notify(struct k_work *item);
void orientation_notify(struct k_work *item);
void ppgData_notify(struct k_work *item);
static ssize_t configSet(struct bt_conn *,const struct bt_gatt_attr *, void *, uint16_t , uint16_t );
static ssize_t read_ppg_quality(struct bt_conn *,const struct bt_gatt_attr *, void *, uint16_t , uint16_t );
static ssize_t read_acc_quality(struct bt_conn *,const struct bt_gatt_attr *, void *, uint16_t , uint16_t );


static ssize_t read_storage_left(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
  static ssize_t bt_turn_off(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_write_patient_num(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t bt_write_date_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);
static ssize_t write_enable_value(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buff, uint16_t len, 
uint16_t offset, uint8_t flags);


 #endif


