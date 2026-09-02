
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include "custom_qspi.h"

#include "ppgSensor.h"
#include "imuSensor.h"
#include "batterymonitordt.h"
#include "zephyrfilesystem.h"
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "zephyr/bluetooth/services/bas.h"
#include <nrfx_timer.h>
#include "BLEService.h"
#include "msense_msc_media.h"
#include "msense_sensor_stream.h"

#include <nrfx_rtc.h>

#if ((32768U % IMU_RTC_TICK_HZ) != 0)
#error "IMU_RTC_TICK_HZ must divide the 32768 Hz RTC clock exactly"
#endif

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "spi_nand.h"
#include "nand_disk.h"
#endif



static const nrfx_rtc_t rtc = NRFX_RTC_INSTANCE(0);


LOG_MODULE_REGISTER(user_bluetooth, 3);

// define our status registers
bool connectedFlag = false;
bool collecting_data = false;
bool host_wants_collection = false;
bool battery_low = false;
bool file_system_full = false;
bool file_system_malfunction = false;
bool battery_charging = false;

/*
 * PPG collection ownership is serialized here.  BLE callbacks and the main
 * battery-maintenance loop only set the requested state and wake this thread.
 */
K_MUTEX_DEFINE(ppg_collection_transition_lock);
K_SEM_DEFINE(ppg_collection_transition_request_sem, 0, 1);
static atomic_t ppg_collection_transition_requested = ATOMIC_INIT(0);
static atomic_t ppg_collection_runtime_ready = ATOMIC_INIT(0);
static atomic_t ppg_collection_workqueue_ready = ATOMIC_INIT(0);
static atomic_t ppg_collection_msc_enabled = ATOMIC_INIT(0);
static atomic_t ppg_collection_ownership_faulted = ATOMIC_INIT(0);
static atomic_t ppg_collection_producer_gate = ATOMIC_INIT(0);

/* One deferred non-normal storage request may be pending at a time. */
enum ppg_storage_action {
  PPG_STORAGE_ACTION_NONE = 0,
  PPG_STORAGE_ACTION_REBOOT,
  PPG_STORAGE_ACTION_ERASE,
  PPG_STORAGE_ACTION_RESET_BAD_BLOCKS,
  PPG_STORAGE_ACTION_TEST_FILES_100,
  PPG_STORAGE_ACTION_TEST_FILES_500,
  PPG_STORAGE_ACTION_TEST_FILE_LARGE,
};

static atomic_t ppg_storage_action_requested = ATOMIC_INIT(
    PPG_STORAGE_ACTION_NONE);
static bool ppg_rtc_initialized;
static struct k_work_sync ppg_sensor_work_sync;
static struct k_work_sync motion_sensor_work_sync;

bool* status_registers[8] = {&connectedFlag, &collecting_data, &host_wants_collection, &battery_low, &file_system_full,  &file_system_malfunction, &battery_charging };
int num_of_status_registers = 7;
bool ble_status_register_send[8] = { 0 };

uint32_t uptime;

static ssize_t update_ble_status_register(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
static ssize_t update_uptime(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
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
static void ppg_collection_transition_thread(void *arg1, void *arg2,
                                             void *arg3);
static int enter_ppg_collection_mode(void);
static int exit_ppg_collection_mode(bool retain_host_request);
static int finalize_ppg_filesystem_for_host(void);
static void ppg_storage_transition_fault(const char *operation, int error);
static void set_firmware_disk_read_only(void);
static void set_firmware_disk_writable(void);
static int request_ppg_storage_action(enum ppg_storage_action action);
static int execute_ppg_storage_action(enum ppg_storage_action action);
void storage_clear_led(void);

#define PPG_COLLECTION_TRANSITION_STACK_SIZE 8192

K_THREAD_DEFINE(ppg_collection_transition_thread_id, PPG_COLLECTION_TRANSITION_STACK_SIZE,
		ppg_collection_transition_thread, NULL, NULL, NULL, 7, 0, 0);


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


/* Later, we should just delete these and move the defining bluetooth to the bottom. */
 struct bt_uuid_128 bt_uuid_data = BT_UUID_INIT_128(UPDATE3_SERVICE_UUID);
 struct bt_uuid_128 bt_uuid_config_rx = BT_UUID_INIT_128(RX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_ppg_tx = BT_UUID_INIT_128(PPG_TX_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_acc_gyro_tx = BT_UUID_INIT_128(ACC_GRYO_TX_CHARACTERISTIC_UUID);
 
 struct bt_uuid_128 bt_uuid_ppg_quality = BT_UUID_INIT_128(PPG_QUALITY_CHARACTERISTIC_UUID);
 struct bt_uuid_128 bt_uuid_acc_quality = BT_UUID_INIT_128(ACC_QUALITY_CHARACTERISTIC_UUID);
#define BT_UUID_DATA_SERVICE      (struct bt_uuid_128 *)(&bt_uuid_data)

#define BT_UUID_PPG_TX   (struct bt_uuid_128 *)(&bt_uuid_ppg_tx)
#define BT_UUID_ACC_GYRO_TX   (struct bt_uuid_128 *)(&bt_uuid_acc_gyro_tx)
#define BT_UUID_PPG_QUALITY   (struct bt_uuid_128 *)(&bt_uuid_ppg_quality)
#define BT_UUID_ACC_QUALITY   (struct bt_uuid_128 *)(&bt_uuid_acc_quality)
// control service characteristics
struct bt_uuid_128 bt_uuid_control = BT_UUID_INIT_128(CONTROL_SERVICE_UUID);
struct bt_uuid_128 bt_enabledisable = BT_UUID_INIT_128(PPG_TX_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_write_enable = BT_UUID_INIT_128(WRITE_ENABLE_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_datetime = BT_UUID_INIT_128(WRITE_DATE_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_patientnum = BT_UUID_INIT_128(WRITE_PATIENT_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_reset = BT_UUID_INIT_128(WRITE_RESET_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_name = BT_UUID_INIT_128(WRITE_DEVICE_NAME_CHARACTERISTIC_UUID);
// status service characteristics
struct bt_uuid_128 bt_uuid_status_service = BT_UUID_INIT_128(STATUS_SERVICE_UUID);
struct bt_uuid_128 bt_uuid_read_storage = BT_UUID_INIT_128(READ_STORAGE_LEFT_UUID);
struct bt_uuid_128 bt_uuid_read_status = BT_UUID_INIT_128(READ_STATUS_REGISTER_UUID);
struct bt_uuid_128 bt_uuid_read_uptime = BT_UUID_INIT_128(READ_UPTIME_UUID);
struct bt_uuid_128 bt_uuid_update_service = BT_UUID_INIT_128(UPDATE_SERVICE_UUID);
struct bt_uuid_128 bt_uuid_enmo_notify = BT_UUID_INIT_128(NOTIFY_ENMO_CHARACTERISTIC_UUID);
struct bt_uuid_128 bt_uuid_enmothreshold_notify = BT_UUID_INIT_128(NOTIFY_ENMOTHRESHOLD_CHARACTERISTIC_UUID);

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
/* TF micro Button Service Declaration and Registration */
BT_GATT_SERVICE_DEFINE(data_service, // 0
  BT_GATT_PRIMARY_SERVICE(BT_UUID_DATA_SERVICE), // 1 
  BT_GATT_CHARACTERISTIC(BT_UUID_PPG_TX, //2
    BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, //3
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_DESCRIPTOR(BT_UUID_PPG_QUALITY,//4
    BT_GATT_PERM_READ, read_ppg_quality,
    NULL, ppgQuality),
  BT_GATT_CUD(PPG_NAME, BT_GATT_PERM_READ),//5
  BT_GATT_CHARACTERISTIC(BT_UUID_ACC_GYRO_TX,//6,
    BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,
    NULL, NULL, NULL),
  BT_GATT_CCC(on_cccd_changed, 
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), //7
  BT_GATT_DESCRIPTOR(BT_UUID_ACC_QUALITY, //8
    BT_GATT_PERM_READ, read_acc_quality,
    NULL, accQuality),
  BT_GATT_CUD(ACC_NAME, BT_GATT_PERM_READ) //9
);

#define BLE_ATTR_PRIMARY_SERVICE 0
#define BLE_ATTR_CONFIG_CHARACTERISTIC 1
#define BLE_ATTR_PPG_CHARACTERISTIC 2
#define BLE_ATTR_ACC_CHARACTERISTIC 6

#endif

/* See bas.c for more information, but essentially, the battery service in configured in this exact same way as here,
 with the same macros and everything.
*/

/* Write Service: Enable device, reset device, Write date time, patient num characteristics*/
BT_GATT_SERVICE_DEFINE(control_service,
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_control),
  BT_GATT_CHARACTERISTIC(&bt_uuid_write_enable.uuid,//18,19
    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    read_generic_one, write_enable_value, &host_wants_collection),
  BT_GATT_CHARACTERISTIC(&bt_uuid_datetime.uuid, 
    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    read_generic_eight, bt_write_date_time, &set_date_time),
  BT_GATT_CHARACTERISTIC(&bt_uuid_patientnum.uuid, 
    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    read_generic_four, bt_write_patient_num, &patient_num),
  BT_GATT_CHARACTERISTIC(&bt_uuid_reset.uuid, 
    BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, 
    NULL, bt_reset, NULL),
  BT_GATT_CHARACTERISTIC(&bt_uuid_name.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, bt_change_brightness, NULL),
); 

/* status service: read storage capacity, potentially battery later on*/
BT_GATT_SERVICE_DEFINE(status_service,
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_status_service),
  BT_GATT_CHARACTERISTIC(&bt_uuid_read_storage.uuid,//18,19
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    read_generic_four, NULL, &storage_percent_full),
    BT_GATT_CHARACTERISTIC(&bt_uuid_read_status.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    update_ble_status_register, NULL, &ble_status_register_send),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&bt_uuid_read_uptime.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
    update_uptime, NULL, &uptime),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* update service: read ENMO updates */
BT_GATT_SERVICE_DEFINE(update_service, // 0
  BT_GATT_PRIMARY_SERVICE(&bt_uuid_update_service), // 1
  BT_GATT_CHARACTERISTIC(&bt_uuid_enmo_notify.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
    NULL, NULL, NULL), // 2
  BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), //3 

  BT_GATT_CHARACTERISTIC(&bt_uuid_enmothreshold_notify.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ , BT_GATT_PERM_READ,
    read_enmo_threshold, NULL, &enmo_threshold_packet), // 4
  BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  );



struct bt_conn* my_connection;





uint8_t gyro_first_read = 0; 
uint8_t ppg_read = 0;





struct ppgInfo my_ppgSensor;
struct ble_battery_info my_battery ;  // work-queue instance for batter level

struct motionInfo my_motionSensor; // work-queue instance for motion sensor

struct bleDataPacket my_ppgDataSensor;


void write_status_register(bool value, int position){
    uint8_t* register_ptr = status_registers[position];
    *register_ptr = value;
}


bool read_status_register(int position){
    return *status_registers[position];
}

static ssize_t update_uptime(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
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

bool ppg_collection_producers_enabled(void)
{
  return atomic_get(&ppg_collection_producer_gate) != 0;
}

bool ppg_collection_faulted(void)
{
  return atomic_get(&ppg_collection_ownership_faulted) != 0;
}

void ppg_collection_set_usb_msc_enabled(bool enabled)
{
  atomic_set(&ppg_collection_msc_enabled, enabled ? 1 : 0);
}

void ppg_collection_set_filesystem_workqueue_ready(void)
{
  atomic_set(&ppg_collection_workqueue_ready, 1);
}

void ppg_collection_enable_runtime_transitions(void)
{
  atomic_set(&ppg_collection_runtime_ready, 1);
  k_sem_give(&ppg_collection_transition_request_sem);
}

void ppg_collection_latch_storage_fault(void)
{
	msense_sensor_stream_storage_failed(-EIO);
	atomic_set(&ppg_collection_ownership_faulted, 1);
  atomic_clear(&ppg_collection_transition_requested);
  atomic_clear(&ppg_collection_producer_gate);
  k_sem_give(&ppg_collection_transition_request_sem);
}

void request_ppg_collection_mode(bool enable)
{
  if (enable && ppg_collection_faulted()) {
    LOG_ERR("Rejecting PPG collection restart while MSC ownership is faulted");
    return;
  }

  atomic_set(&ppg_collection_transition_requested, enable ? 1 : 0);
  k_sem_give(&ppg_collection_transition_request_sem);
}

void request_ppg_collection_reconcile(void)
{
  k_sem_give(&ppg_collection_transition_request_sem);
}

static int request_ppg_storage_action(enum ppg_storage_action action)
{
  if (ppg_collection_faulted()) {
    return -EIO;
  }
  if (atomic_get(&ppg_collection_runtime_ready) == 0 ||
      atomic_get(&ppg_collection_workqueue_ready) == 0) {
    return -EAGAIN;
  }

  /* Do not queue a storage mutation behind a requested collection start. */
  if (collecting_data || host_wants_collection ||
      atomic_get(&ppg_collection_transition_requested) != 0) {
    return -EBUSY;
  }
  if (!atomic_cas(&ppg_storage_action_requested,
                  PPG_STORAGE_ACTION_NONE, action)) {
    return -EBUSY;
  }

  k_sem_give(&ppg_collection_transition_request_sem);
  return 0;
}

int request_ppg_storage_reboot(void)
{
  return request_ppg_storage_action(PPG_STORAGE_ACTION_REBOOT);
}

int request_ppg_storage_reset(bool reset_bad_blocks)
{
  return request_ppg_storage_action(
      reset_bad_blocks ? PPG_STORAGE_ACTION_RESET_BAD_BLOCKS :
                         PPG_STORAGE_ACTION_ERASE);
}

int request_ppg_manual_test_file(uint8_t command)
{
  enum ppg_storage_action action;

  switch (command) {
  case 130:
    action = PPG_STORAGE_ACTION_TEST_FILES_100;
    break;
  case 150:
    action = PPG_STORAGE_ACTION_TEST_FILES_500;
    break;
  case 151:
    action = PPG_STORAGE_ACTION_TEST_FILE_LARGE;
    break;
  default:
    return -EINVAL;
  }

  return request_ppg_storage_action(action);
}

void rtc_handler(nrfx_rtc_int_type_t event_type){
  int work_queue_result;

  // Get the spurious cases out of the way...
  if (!ppg_collection_producers_enabled()) { return; }
  if (event_type != NRFX_RTC_INT_TICK) { return; }

  // Submit PPG work. Do this first because its more regular. The IMU goes longer when it integrates.
  if(ppg_read == 0){
    my_ppgSensor.pktCounter = global_counter;
    my_ppgSensor.movingFlag = current_gyro_data.movingFlag;
    work_queue_result = k_work_submit(&my_ppgSensor.work);
    if (work_queue_result != 1) { LOG_ERR("PPG work queue was not submitted: %i", work_queue_result); }
  }

  // submit work to read gyro, acc, magnetometer and orientation
  my_motionSensor.pktCounter = global_counter;
  my_motionSensor.gyro_first_read = gyro_first_read;
  work_queue_result = k_work_submit(&my_motionSensor.work);
  if (work_queue_result != 1) { LOG_ERR("accel work queue was not submitted: %i", work_queue_result); }

  // ppgConfig.numCounts is derived from the RTC cadence.
  ppg_read = (ppg_read+1) % ppgConfig.numCounts;

  gyro_first_read = (gyro_first_read + 1) % (gyroConfig.tot_samples);

  global_counter++;
}


static void ppg_rtc_stop_source(void)
{
  if (!ppg_rtc_initialized) {
    return;
  }

  irq_disable(RTC0_IRQn);
  nrfx_rtc_disable(&rtc);
  nrfx_rtc_uninit(&rtc);
  ppg_rtc_initialized = false;
}


static int ppg_rtc_start_source(void)
{
  nrfx_err_t err;
  nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;

  // Setup RTC0 (RTC1 used by Zephyr) at the configured acquisition cadence.
  config.prescaler = IMU_RTC_PRESCALER;
  config.interrupt_priority = IMU_RTC_IRQ_PRIORITY;
  err = nrfx_rtc_init(&rtc, &config, rtc_handler);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("nrfx_rtc_init() failed with: %d", err);
    return -EIO;
  }

  ppg_rtc_initialized = true;
  nrfx_rtc_tick_enable(&rtc, true);
  nrfx_rtc_enable(&rtc);
  IRQ_CONNECT(RTC0_IRQn, IMU_RTC_IRQ_PRIORITY, nrfx_rtc_0_irq_handler, NULL, 0);
  irq_enable(RTC0_IRQn);
  return 0;
}


static void ppg_stop_producers(void)
{
  /* Stop the ISR source before cancelling and joining its system work. */
  atomic_clear(&ppg_collection_producer_gate);
  ppg_rtc_stop_source();
  (void)k_work_cancel_sync(&my_ppgSensor.work, &ppg_sensor_work_sync);
  (void)k_work_cancel_sync(&my_motionSensor.work, &motion_sensor_work_sync);
  motion_sleep();
  ppg_sleep();
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
    request_ppg_collection_mode(true);
    #endif
    // blink proceedure to indicate connected status
    blink_led(30);
    k_sleep(K_MSEC(100));
    blink_led(30);
    k_sleep(K_MSEC(100));
    blink_led(30);

  }
}

void disconnected(struct bt_conn *conn, uint8_t reason){
  // Stop timer and do all the cleanup
  printk("Disconnected (reason %u)\n", reason);
  connectedFlag=false;

  #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
    request_ppg_collection_mode(false);
  #endif
}




void reset_device(bool reset_bad_blocks){
  int ret = request_ppg_storage_reset(reset_bad_blocks);

  if (ret != 0) {
    LOG_ERR("Storage reset request rejected: %d", ret);
  }
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

static int publish_ppg_msc_host_media(void)
{
  int claim_ret;
  int ret;

  ret = msense_msc_media_publish_to_host();
  if (ret == 0) {
    return 0;
  }

  LOG_ERR("MSC host publication failed: %d", ret);
  claim_ret = msense_msc_media_claim_for_firmware();
  if (claim_ret != 0) {
    LOG_ERR("MSC absence recovery failed: %d", claim_ret);
  }
  return ret;
}

static int finalize_ppg_filesystem_for_host(void)
{
  int ret;

  if (atomic_get(&ppg_collection_workqueue_ready) == 0) {
    return -ENODEV;
  }

  ret = filesystem_drain_pending_work();
  if (ret != 0) {
    return ret;
  }

  ret = flush_data_buffer(ppg);
  if (ret != 0) {
    return ret;
  }
  ret = flush_data_buffer(accelorometer);
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

static int shutdown_ppg_filesystem_after_start_failure(bool collection_mount_ready)
{
  int ret;

  if (collection_mount_ready) {
    return finalize_ppg_filesystem_for_host();
  }
  if (atomic_get(&ppg_collection_workqueue_ready) == 0) {
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

static void ppg_storage_transition_fault(const char *operation, int error)
{
  int claim_ret;
  int gate_ret;

  atomic_clear(&ppg_collection_producer_gate);
  if (atomic_get(&ppg_collection_msc_enabled) != 0) {
    claim_ret = msense_msc_media_claim_for_firmware();
    if (claim_ret != 0) {
      LOG_ERR("MSC absence recovery returned %d after %s", claim_ret,
              operation);
    }
  }

  ppg_filesystem_log_disable_and_wait();
  if (atomic_get(&ppg_collection_workqueue_ready) != 0) {
    gate_ret = filesystem_gate_and_drain();
    if (gate_ret != 0) {
      LOG_ERR("Filesystem fault gate returned %d after %s", gate_ret,
              operation);
    }
  }
  set_firmware_disk_read_only();
  atomic_set(&ppg_collection_ownership_faulted, 1);
  atomic_clear(&ppg_collection_transition_requested);
  LOG_ERR("%s failed: %d; keeping MSC medium absent", operation, error);
}

/* Called only by ppg_collection_transition_thread while it holds the mutex. */
static int prepare_ppg_storage_action(void)
{
  int ret;

  if (collecting_data || host_wants_collection ||
      atomic_get(&ppg_collection_transition_requested) != 0) {
    return -EBUSY;
  }

  reset_lock = true;
  ppg_filesystem_log_disable_and_wait();
  ppg_stop_producers();

  if (atomic_get(&ppg_collection_msc_enabled) != 0) {
    ret = msense_msc_media_claim_for_firmware();
    if (ret != 0) {
      ppg_storage_transition_fault("PPG non-normal MSC firmware claim", ret);
      return ret;
    }
  }

  if (atomic_get(&ppg_collection_workqueue_ready) == 0) {
    ret = -ENODEV;
    goto teardown_failed;
  }

  ret = filesystem_gate_and_drain();
  if (ret != 0) {
    goto teardown_failed;
  }
  if (filesystem_is_mounted()) {
    ret = shutdown_filesystem();
    if (ret != 0) {
      goto teardown_failed;
    }
  }
  if (filesystem_is_mounted()) {
    ret = -EBUSY;
    goto teardown_failed;
  }

  return 0;

teardown_failed:
  set_firmware_disk_read_only();
  ppg_storage_transition_fault("PPG non-normal storage teardown", ret);
  return ret;
}

static int reset_ppg_storage_and_reboot(bool reset_bad_blocks)
{
  const struct device *flash_device = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
  int ret;

  if (!device_is_ready(flash_device)) {
    ret = -ENODEV;
    goto erase_failed;
  }

  set_firmware_disk_writable();
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
  LOG_INF("Storage erase complete; resetting while MSC remains absent");
  NVIC_SystemReset();
  ret = -EIO;

erase_failed:
  set_firmware_disk_read_only();
  ppg_storage_transition_fault("PPG storage erase/reset", ret);
  return ret;
}

static int reboot_ppg_after_storage_teardown(void)
{
  set_firmware_disk_read_only();
  LOG_INF("Resetting while MSC remains absent");
  NVIC_SystemReset();
  ppg_storage_transition_fault("PPG reboot", -EIO);
  return -EIO;
}

static int run_ppg_manual_test_file_action(enum ppg_storage_action action)
{
  int ret;

  set_firmware_disk_writable();
  ret = setup_disk();
  if (ret != 0) {
    goto test_failed;
  }

  storage_clear_led();
  switch (action) {
  case PPG_STORAGE_ACTION_TEST_FILES_100:
    ret = create_test_files(100);
    break;
  case PPG_STORAGE_ACTION_TEST_FILES_500:
    ret = create_test_files(500);
    break;
  case PPG_STORAGE_ACTION_TEST_FILE_LARGE:
    ret = create_test_file(512 * 450);
    break;
  default:
    ret = -EINVAL;
    break;
  }
  if (ret != 0) {
    goto test_failed;
  }

  ret = filesystem_gate_and_drain();
  if (ret != 0) {
    goto test_failed;
  }
  ret = shutdown_filesystem();
  if (ret != 0) {
    goto test_failed;
  }
  if (filesystem_is_mounted()) {
    ret = -EBUSY;
    goto test_failed;
  }

  set_firmware_disk_read_only();
  if (atomic_get(&ppg_collection_msc_enabled) != 0) {
    ret = publish_ppg_msc_host_media();
    if (ret != 0) {
      goto test_failed;
    }
  }

  reset_lock = false;
  collecting_data = false;
  host_wants_collection = false;
  return 0;

test_failed:
  set_firmware_disk_read_only();
  ppg_storage_transition_fault("PPG manual test-file operation", ret);
  return ret;
}

static int execute_ppg_storage_action(enum ppg_storage_action action)
{
  int ret;

  k_mutex_lock(&ppg_collection_transition_lock, K_FOREVER);
  if (ppg_collection_faulted()) {
    ret = -EIO;
    goto out;
  }

  ret = prepare_ppg_storage_action();
  if (ret != 0) {
    goto out;
  }

  switch (action) {
  case PPG_STORAGE_ACTION_REBOOT:
    ret = reboot_ppg_after_storage_teardown();
    break;
  case PPG_STORAGE_ACTION_ERASE:
    ret = reset_ppg_storage_and_reboot(false);
    break;
  case PPG_STORAGE_ACTION_RESET_BAD_BLOCKS:
    ret = reset_ppg_storage_and_reboot(true);
    break;
  case PPG_STORAGE_ACTION_TEST_FILES_100:
  case PPG_STORAGE_ACTION_TEST_FILES_500:
  case PPG_STORAGE_ACTION_TEST_FILE_LARGE:
    ret = run_ppg_manual_test_file_action(action);
    break;
  default:
    ret = -EINVAL;
    break;
  }

out:
  k_mutex_unlock(&ppg_collection_transition_lock);
  return ret;
}

static int enter_ppg_collection_mode(void)
{
  int cleanup_ret;
  int ret;
  int start_ret;
  bool collection_mount_ready = false;
  bool sensors_configured = false;
  bool rtc_started = false;

  k_mutex_lock(&ppg_collection_transition_lock, K_FOREVER);
  if (ppg_collection_faulted()) {
    k_mutex_unlock(&ppg_collection_transition_lock);
    return -EIO;
  }
  if (atomic_get(&ppg_collection_runtime_ready) == 0) {
    k_mutex_unlock(&ppg_collection_transition_lock);
    return -EAGAIN;
  }
  if (collecting_data) {
    k_mutex_unlock(&ppg_collection_transition_lock);
    return 0;
  }

  LOG_INF("Entering PPG collection mode");
  if (atomic_get(&ppg_collection_msc_enabled) != 0) {
    ret = msense_msc_media_claim_for_firmware();
    if (ret != 0) {
      ppg_storage_transition_fault("MSC firmware claim", ret);
      k_mutex_unlock(&ppg_collection_transition_lock);
      return ret;
    }
  }

  set_firmware_disk_writable();
  ret = setup_disk();
  if (ret != 0) {
    LOG_ERR("Failed to mount PPG filesystem: %d", ret);
    goto start_failed;
  }
  collection_mount_ready = true;

  if (atomic_get(&ppg_collection_workqueue_ready) == 0) {
    ret = -ENODEV;
    goto start_failed;
  }
  ret = k_work_queue_unplug(&my_work_q);
  if (ret != 0) {
    LOG_ERR("Unable to enable PPG filesystem workqueue: %d", ret);
    goto start_failed;
  }

  ppg_config();
  motion_config();
  sensors_configured = true;
  global_counter = 0;
  gyro_first_read = 0;
  ppg_read = 0;

  ret = ppg_rtc_start_source();
  if (ret != 0) {
    goto start_failed;
  }
  rtc_started = true;

	msense_sensor_stream_recording_started();
	atomic_set(&ppg_collection_producer_gate, 1);
  collecting_data = true;
  host_wants_collection = true;
  ppg_filesystem_log_enable();
  k_mutex_unlock(&ppg_collection_transition_lock);
  return 0;

start_failed:
  start_ret = ret;
  ppg_filesystem_log_disable_and_wait();
  atomic_clear(&ppg_collection_producer_gate);
  if (rtc_started || sensors_configured) {
    ppg_stop_producers();
  }

  cleanup_ret = shutdown_ppg_filesystem_after_start_failure(
      collection_mount_ready);
  set_firmware_disk_read_only();
  collecting_data = false;
  host_wants_collection = false;
  atomic_clear(&ppg_collection_transition_requested);
  if (cleanup_ret == 0 && !filesystem_is_mounted() &&
      atomic_get(&ppg_collection_msc_enabled) != 0) {
    cleanup_ret = publish_ppg_msc_host_media();
  }
  if (cleanup_ret != 0) {
    ppg_storage_transition_fault("PPG collection start unwind", cleanup_ret);
    ret = cleanup_ret;
  } else {
    ret = start_ret;
  }

  k_mutex_unlock(&ppg_collection_transition_lock);
  return ret;
}

static int exit_ppg_collection_mode(bool retain_host_request)
{
  int ret;

  k_mutex_lock(&ppg_collection_transition_lock, K_FOREVER);
  if (!collecting_data) {
    if (ppg_collection_faulted()) {
      ppg_storage_transition_fault("PPG ownership fault", -EIO);
      k_mutex_unlock(&ppg_collection_transition_lock);
      return -EIO;
    }

    host_wants_collection = retain_host_request;
    k_mutex_unlock(&ppg_collection_transition_lock);
    return 0;
  }

	LOG_INF("Leaving PPG collection mode");
	ppg_filesystem_log_disable_and_wait();
	msense_sensor_stream_recording_stopped();
	ppg_stop_producers();
  if (ppg_collection_faulted()) {
    ppg_storage_transition_fault("PPG collection ownership fault", -EIO);
    k_mutex_unlock(&ppg_collection_transition_lock);
    return -EIO;
  }

  ret = finalize_ppg_filesystem_for_host();
  if (ret != 0) {
    ppg_storage_transition_fault("PPG filesystem teardown", ret);
    k_mutex_unlock(&ppg_collection_transition_lock);
    return ret;
  }

  set_firmware_disk_read_only();
  if (atomic_get(&ppg_collection_msc_enabled) != 0) {
    ret = publish_ppg_msc_host_media();
    if (ret != 0) {
      ppg_storage_transition_fault("PPG MSC publication", ret);
      k_mutex_unlock(&ppg_collection_transition_lock);
      return ret;
    }
  }

  enmo_sample_counter = 0;
  last_activated_trigger_counter = 0;
  collecting_data = false;
  host_wants_collection = retain_host_request;
  k_mutex_unlock(&ppg_collection_transition_lock);
  return 0;
}

static void ppg_collection_transition_thread(void *arg1, void *arg2,
                                             void *arg3)
{
  enum ppg_storage_action action;
  bool retain_host_request;
  int ret;

  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  for (;;) {
    (void)k_sem_take(&ppg_collection_transition_request_sem, K_FOREVER);
    action = (enum ppg_storage_action)atomic_get(
        &ppg_storage_action_requested);
    if (action != PPG_STORAGE_ACTION_NONE) {
      ret = execute_ppg_storage_action(action);
      atomic_set(&ppg_storage_action_requested, PPG_STORAGE_ACTION_NONE);
      if (ret != 0) {
        LOG_ERR("PPG non-normal storage action failed: %d", ret);
      }
    }
    if (ppg_collection_faulted()) {
      (void)exit_ppg_collection_mode(false);
      continue;
    }
    if (atomic_get(&ppg_collection_runtime_ready) == 0) {
      continue;
    }

    retain_host_request =
        atomic_get(&ppg_collection_transition_requested) != 0;
    if (retain_host_request && !battery_low) {
      (void)enter_ppg_collection_mode();
    } else {
      (void)exit_ppg_collection_mode(retain_host_request);
    }
  }
}

void start_stop_device_collection(uint8_t val)
{
  request_ppg_collection_mode(val != 0);
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
  if (val != 0 && ppg_collection_faulted()) {
    LOG_ERR("Rejecting enable write while MSC ownership is faulted");
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }

  request_ppg_collection_mode(val != 0);
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
  int ret;

  LOG_INF("Attribute write, handle: %u, conn: %p, length %i", attr->handle,
		(void *)conn, len);

	
	LOG_INF("Write length: %i", len);
  if (len != 1){
    LOG_WRN("invalid packet length for reset: %i", len);
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }
  
  if (offset != 0) {
		LOG_INF("Write: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  // check the bluetooth value entered for the correct code.
  uint8_t val = *((uint8_t *)buff);
  LOG_INF("entered code: %i", val);
  if (val == 68) {
    ret = request_ppg_storage_reset(false);
  } else if (val == 132) {
    ret = request_ppg_storage_reset(true);
  } else if (val == 121) {
    NVIC_SystemReset();
    return len;
  } else {
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }

  if (ret != 0) {
    LOG_ERR("Reset request rejected: %d", ret);
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }

  LOG_INF("Queued reset through PPG storage transition owner");
  (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
  (void)bt_le_adv_stop();
  connectedFlag = false;
  return len;
}

// Note: Currently does not work, more work is needed to allow dynamic runtime name changing.
/*
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

  err = bt_le_adv_start(&v, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err)
    printk("Advertising failed to start (err %d)\n", err);
  else
  {
    printk("Advertising successfully started\n");
  }
  k_sleep(K_SECONDS(1));
  NVIC_SystemReset();
  return 0;
  
}
*/
void create_test_files_through_file_workqueue(struct k_work* work){
  int ret;

  ARG_UNUSED(work);
  ret = request_ppg_manual_test_file(130);
  if (ret != 0) {
    LOG_ERR("Queued test-file request rejected: %d", ret);
  }

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
    int ret;

    LOG_INF("Attribute other settings write, handle: %u, conn: %p, length %i", attr->handle,
      (void *)conn, len);
  
    
    
    if (offset != 0) {
      LOG_INF("Write: Incorrect data offset");
      return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
  
    int val = 0;
    memcpy(&val, buff, len);
    LOG_INF("entered value: %i", val);
    if (!collecting_data){
      if (val == 0){
        LOG_INF("Turning off auto brightness");
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
        if ((val == 130 || val == 150 || val == 151) && !collecting_data){
          ret = request_ppg_manual_test_file((uint8_t)val);
          if (ret != 0) {
            LOG_ERR("Manual file creation request rejected: %d", ret);
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
          }
          LOG_INF("Queued manual file creation through PPG storage owner");
        }
        if (val >= 1000){
          print_out_page(val - 1000);
        }

      }  
      return len;
      
    }
    
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }


static ssize_t read_generic_one(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset){
  
  const char* value = attr->user_data;
  //uint8_t space_left = storage_percent_full;
  //LOG_INF("space full: %i", space_left);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 1);

}

static ssize_t read_generic_four(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
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


void status_reg_ble_notification(){

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
      selected_service = &control_service;

  }
  const struct bt_gatt_attr *attr = &selected_service->attrs[characteristic];
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
  
  uint8_t packetLength = the_device->packetLength;
  //printk("%i", packetLength);
  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  #ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
  acc_send(my_connection, the_device->dataPacket, the_device->packetLength);
  #else
  uint8_t *dataPacket = the_device->dataPacket;
  memcpy(&dataPacket[4], &global_counter, sizeof(global_counter));
  enmo_send(my_connection, the_device->dataPacket, the_device->packetLength);
  #endif

}

#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES

uint8_t configRead[6] = {0,0,0,0,0,0};
uint8_t ppgQuality[4] = {0};
uint8_t accQuality[4] = {0};


void acc_send(struct bt_conn *conn, const uint8_t *data, uint16_t len){
  
  const struct bt_gatt_attr *attr = &data_service.attrs[BLE_ATTR_ACC_CHARACTERISTIC]; 
  struct bt_gatt_notify_params params = {
    .uuid   = BT_UUID_ACC_GYRO_TX,
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



void ppgData_notify(struct k_work *item){
  struct bleDataPacket* the_device=  ((struct bleDataPacket *)(((char *)(item)) - offsetof(struct bleDataPacket, work)));

  ////printk("data LED =%u, Data counter1=%u, Data counter2=%u,pk=%u\n", dataPacket[0],dataPacket[1],dataPacket[2],packetLength);
  ppg_send(my_connection, the_device->dataPacket, PPG_DATA_UNFILTER_LEN);
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




//#if CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES
// Config Data Tx
// B_1 B_2 - 0x0001 PPG enabled
//         - 0x0002 IMU enabled
//         - 0x0004 orientation enabled
//         - 0x0008 TF micro enabled (deprecated)
//         - 0x0010 Magnetometer enabled (deprecated)
//         - 0x0100 PPG BLE transmit enable
//         - 0x0200 IMU BLE transmit enable
//         - 0x0400 orientation BLE transmit enable
//         - 0x0800 TF micro BLE transmit enable
//         - 0x1000 Magnetometer BLE transmit enable (deprecated)
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
/* This function is called whenever the RX Characteristic has been written to by a Client */
ssize_t legacy_on_settings_change(struct bt_conn *conn,
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
      if((buffer[1] & PPG_ENABLE) == PPG_ENABLE){
        ppgConfig.isEnabled = true;
        configRead[1] = configRead[1] | 0x01; 
      }
      else if((buffer[1] & PPG_ENABLE) == 0x00){
        ppgConfig.isEnabled = false;
        configRead[1] = configRead[1] & 0xFE; 
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
      if((buffer[2] & PPG_BLE_ENABLE) == PPG_BLE_ENABLE){
        ppgConfig.txPacketEnable = true;
        configRead[0] = configRead[0] | 0x01; 
      }
      else if((buffer[2] & PPG_BLE_ENABLE) == 0x00){
        ppgConfig.txPacketEnable = false;
        configRead[0] = configRead[0] & 0xFE; 
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

void legacy_initialize_settings(){
  configRead[0] = MOTION_BLE_ENABLE | PPG_BLE_ENABLE;
  configRead[1] = IMU_ENABLE | PPG_ENABLE;
  configRead[2] = ppgConfig.green_intensity;
  configRead[3] = ppgConfig.infraRed_intensity;
  configRead[4] = 0x12;
  configRead[5] = PPG_FIXED_256HZ_STATUS | MOTION_FIXED_32HZ_STATUS;
}

/* This function is called whenever a Notification has been sent by the TX Characteristic */
static void on_sent(struct bt_conn* conn, void* user_data){
  ARG_UNUSED(user_data);
  //const bt_addr_le_t * addr = bt_conn_get_dst(conn);
    /*    
	//printk("Data sent to Address 0x %02X %02X %02X %02X %02X %02X \n", addr->a.val[0]
                                                                    , addr->a.val[1]
                                                                    , addr->a.val[2]
                                                                    , addr->a.val[3]
                                                                    , addr->a.val[4]
                                                                    , addr->a.val[5]);*/
}





#endif
