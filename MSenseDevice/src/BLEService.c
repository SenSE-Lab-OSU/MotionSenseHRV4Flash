
#include <zephyr/types.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>
#include <zephyr/usb/usb_device.h>
#include "drivers/jdec_nor/custom_qspi.h"

#include "ppgSensor.h"
#include "batterymonitordt.h"
#include "zephyrfilesystem.h"
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "zephyr/bluetooth/services/bas.h"
#include <hal/nrf_rtc.h>
#include <nrfx_rtc.h>
#include "BLEService.h"

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "drivers/nand/spi_nand.h"
#include "drivers/nand/nand_disk.h"
#endif




LOG_MODULE_REGISTER(user_bluetooth);

#define RTC0_COLLECTION_COUNTER_HZ 512U
#define RTC0_COLLECTION_COUNTER_PRESCALER \
	((32768U / RTC0_COLLECTION_COUNTER_HZ) - 1U)
#define RTC0_COLLECTION_COUNTER_MASK 0x00FFFFFFU
#define RTC0_COLLECTION_COUNTER_WRAP (RTC0_COLLECTION_COUNTER_MASK + 1U)
#define RTC0_COLLECTION_NOTIFY_INTERVAL_TICKS (3U * RTC0_COLLECTION_COUNTER_HZ)
#define RTC0_COLLECTION_NOTIFY_COMPARE_CHANNEL 0U

BUILD_ASSERT((32768U % RTC0_COLLECTION_COUNTER_HZ) == 0U,
	     "RTC0 collection counter frequency must divide LFCLK");

static const nrfx_rtc_t rtc0_collection_counter = NRFX_RTC_INSTANCE(0);
static atomic_t rtc0_collection_counter_started;
static atomic_t rtc0_collection_notify_active;
static atomic_t rtc0_collection_notify_ticks;
static uint32_t rtc0_collection_counter_epoch;
static uint32_t rtc0_collection_next_compare;
static uint32_t rtc0_collection_next_notify_ticks;
static struct k_work_sync rtc0_collection_notify_work_sync;

static void rtc0_collection_notify_work_handler(struct k_work *work);
K_WORK_DEFINE(rtc0_collection_notify_work, rtc0_collection_notify_work_handler);

static uint32_t rtc0_collection_counter_get_locked(uint32_t *raw_ticks)
{
	bool overflow_pending;
	uint32_t raw;
	uint32_t epoch = rtc0_collection_counter_epoch;

	/*
	 * Account for a hardware wrap whose interrupt is still pending. Re-read the
	 * low counter if the overflow arrives between the first read and check.
	 */
	overflow_pending = nrf_rtc_event_check(rtc0_collection_counter.p_reg,
						 NRF_RTC_EVENT_OVERFLOW);
	raw = nrfx_rtc_counter_get(&rtc0_collection_counter) &
	      RTC0_COLLECTION_COUNTER_MASK;
	if (!overflow_pending && nrf_rtc_event_check(rtc0_collection_counter.p_reg,
						      NRF_RTC_EVENT_OVERFLOW)) {
		overflow_pending = true;
		raw = nrfx_rtc_counter_get(&rtc0_collection_counter) &
		      RTC0_COLLECTION_COUNTER_MASK;
	}
	if (overflow_pending) {
		epoch += RTC0_COLLECTION_COUNTER_WRAP;
	}

	if (raw_ticks != NULL) {
		*raw_ticks = raw;
	}

	return epoch | raw;
}

static void rtc0_collection_notify_stop(void)
{
	unsigned int key;

	if (atomic_cas(&rtc0_collection_notify_active, 1, 0)) {
		key = irq_lock();
		(void)nrfx_rtc_cc_disable(&rtc0_collection_counter,
					  RTC0_COLLECTION_NOTIFY_COMPARE_CHANNEL);
		irq_unlock(key);
	}

	(void)k_work_cancel_sync(&rtc0_collection_notify_work,
				 &rtc0_collection_notify_work_sync);
}

static void rtc0_collection_counter_handler(nrfx_rtc_int_type_t event_type)
{
	nrfx_err_t err;
	uint32_t notify_ticks;

	if (event_type == NRFX_RTC_INT_OVERFLOW) {
		rtc0_collection_counter_epoch += RTC0_COLLECTION_COUNTER_WRAP;
		return;
	}

	if (event_type != (nrfx_rtc_int_type_t)RTC0_COLLECTION_NOTIFY_COMPARE_CHANNEL ||
	    atomic_get(&rtc0_collection_notify_active) == 0) {
		return;
	}

	/* Advance from the prior target to keep an exact 1536-tick cadence. */
	notify_ticks = rtc0_collection_next_notify_ticks;
	rtc0_collection_next_notify_ticks += RTC0_COLLECTION_NOTIFY_INTERVAL_TICKS;
	rtc0_collection_next_compare =
		(rtc0_collection_next_compare + RTC0_COLLECTION_NOTIFY_INTERVAL_TICKS) &
		RTC0_COLLECTION_COUNTER_MASK;

	err = nrfx_rtc_cc_set(&rtc0_collection_counter,
			      RTC0_COLLECTION_NOTIFY_COMPARE_CHANNEL,
			      rtc0_collection_next_compare, true);
	if (err != NRFX_SUCCESS) {
		atomic_set(&rtc0_collection_notify_active, 0);
		return;
	}

	atomic_set(&rtc0_collection_notify_ticks, (atomic_val_t)notify_ticks);
	(void)k_work_submit(&rtc0_collection_notify_work);
}

int rtc0_collection_counter_start(void)
{
	nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
	nrfx_err_t err;

	if (atomic_get(&rtc0_collection_counter_started) != 0 ||
	    nrfx_rtc_init_check(&rtc0_collection_counter)) {
		return -EALREADY;
	}

	config.prescaler = RTC0_COLLECTION_COUNTER_PRESCALER;
	/* Keep the nrfx peripheral priority and Zephyr vector entry in sync. */
	config.interrupt_priority = IRQ_PRIO_LOWEST;
	err = nrfx_rtc_init(&rtc0_collection_counter, &config,
			    rtc0_collection_counter_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("RTC0 collection counter initialization failed: %d", err);
		return -EIO;
	}

	nrfx_rtc_counter_clear(&rtc0_collection_counter);
	rtc0_collection_counter_epoch = 0U;
	rtc0_collection_next_compare = 0U;
	rtc0_collection_next_notify_ticks = 0U;
	atomic_set(&rtc0_collection_notify_active, 0);
	atomic_set(&rtc0_collection_notify_ticks, 0);
	nrfx_rtc_overflow_enable(&rtc0_collection_counter, true);
	nrfx_rtc_enable(&rtc0_collection_counter);
	/*
	 * nrfx_rtc_init() enables the NVIC line but does not populate Zephyr's
	 * vector table. Without this connection, the first compare interrupt is
	 * handled as a spurious IRQ and resets the application.
	 */
	IRQ_CONNECT(RTC0_IRQn, IRQ_PRIO_LOWEST, nrfx_rtc_0_irq_handler, NULL, 0);
	irq_enable(RTC0_IRQn);
	atomic_set(&rtc0_collection_counter_started, 1);
	LOG_INF("RTC0 collection counter started at %u Hz",
		RTC0_COLLECTION_COUNTER_HZ);

	return 0;
}

void rtc0_collection_counter_stop(void)
{
	if (!atomic_cas(&rtc0_collection_counter_started, 1, 0)) {
		return;
	}

	rtc0_collection_notify_stop();
	nrfx_rtc_overflow_disable(&rtc0_collection_counter);
	nrfx_rtc_disable(&rtc0_collection_counter);
	nrfx_rtc_uninit(&rtc0_collection_counter);
	LOG_INF("RTC0 collection counter stopped");
}

int rtc0_collection_counter_get(uint32_t *ticks)
{
	if (ticks == NULL) {
		return -EINVAL;
	}
	if (atomic_get(&rtc0_collection_counter_started) == 0) {
		return -EACCES;
	}

	{
		unsigned int key = irq_lock();

		*ticks = rtc0_collection_counter_get_locked(NULL);
		irq_unlock(key);
	}

	return 0;
}

int rtc0_collection_notification_start(void)
{
	nrfx_err_t err;
	unsigned int key;
	uint32_t raw_ticks;
	uint32_t extended_ticks;

	if (atomic_get(&rtc0_collection_counter_started) == 0) {
		return -EACCES;
	}
	if (!atomic_cas(&rtc0_collection_notify_active, 0, 1)) {
		return -EALREADY;
	}

	key = irq_lock();
	extended_ticks = rtc0_collection_counter_get_locked(&raw_ticks);
	rtc0_collection_next_compare =
		(raw_ticks + RTC0_COLLECTION_NOTIFY_INTERVAL_TICKS) &
		RTC0_COLLECTION_COUNTER_MASK;
	rtc0_collection_next_notify_ticks =
		extended_ticks + RTC0_COLLECTION_NOTIFY_INTERVAL_TICKS;
	err = nrfx_rtc_cc_set(&rtc0_collection_counter,
			      RTC0_COLLECTION_NOTIFY_COMPARE_CHANNEL,
			      rtc0_collection_next_compare, true);
	irq_unlock(key);
	if (err != NRFX_SUCCESS) {
		atomic_set(&rtc0_collection_notify_active, 0);
		return -EIO;
	}

	LOG_INF("RTC0 BLE timing notification started at %u-tick cadence",
		RTC0_COLLECTION_NOTIFY_INTERVAL_TICKS);
	return 0;
}

// define our status registers
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
static ssize_t update_uptime(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
static ssize_t read_generic_one(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
static ssize_t read_generic_four(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset);
static ssize_t read_generic_eight(struct bt_conn *conn,const struct bt_gatt_attr *attr, void *buf,
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
 
 struct bt_uuid_128 bt_uuid_ppg_quality = BT_UUID_INIT_128(PPG_QUALITY_CHARACTERISTIC_UUID);
#define BT_UUID_DATA_SERVICE      (struct bt_uuid_128 *)(&bt_uuid_data)

#define BT_UUID_PPG_TX   (struct bt_uuid_128 *)(&bt_uuid_ppg_tx)
#define BT_UUID_PPG_QUALITY   (struct bt_uuid_128 *)(&bt_uuid_ppg_quality)
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
static struct bt_uuid_128 bt_uuid_timing_update_service =
	BT_UUID_INIT_128(TIMING_UPDATE_SERVICE_UUID);
static struct bt_uuid_128 bt_uuid_timing_update =
	BT_UUID_INIT_128(TIMING_UPDATE_CHARACTERISTIC_UUID);

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
  BT_GATT_CUD(PPG_NAME, BT_GATT_PERM_READ)
);

#define BLE_ATTR_PRIMARY_SERVICE 0
#define BLE_ATTR_CONFIG_CHARACTERISTIC 1
#define BLE_ATTR_PPG_CHARACTERISTIC 2

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

/*
 * This preserves the main-branch update-service UUID and its 8-byte packet
 * compatibility. It deliberately exposes no ENMO, threshold, or sensor data.
 */
BT_GATT_SERVICE_DEFINE(timing_update_service,
	BT_GATT_PRIMARY_SERVICE(&bt_uuid_timing_update_service),
	BT_GATT_CHARACTERISTIC(&bt_uuid_timing_update.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static struct k_spinlock my_connection_lock;
static struct bt_conn *my_connection;

static struct bt_conn *collection_notification_connection_get(void)
{
	struct bt_conn *conn = NULL;
	k_spinlock_key_t key;

	key = k_spin_lock(&my_connection_lock);
	if (my_connection != NULL) {
		conn = bt_conn_ref(my_connection);
	}
	k_spin_unlock(&my_connection_lock, key);

	return conn;
}

static void rtc0_collection_notify_work_handler(struct k_work *work)
{
	const struct bt_gatt_attr *attr = &timing_update_service.attrs[2];
	struct bt_conn *conn;
	uint8_t packet[8] = {0};
	uint32_t ticks;
	int ret;

	ARG_UNUSED(work);

	if (atomic_get(&rtc0_collection_notify_active) == 0 || !collecting_data) {
		return;
	}

	ticks = (uint32_t)atomic_get(&rtc0_collection_notify_ticks);
	sys_put_le32(ticks, &packet[4]);

	conn = collection_notification_connection_get();
	if (conn == NULL) {
		return;
	}

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		ret = bt_gatt_notify(conn, attr, packet, sizeof(packet));
		if (ret != 0) {
			LOG_WRN("RTC0 timing notification failed: %d", ret);
		}
	}

	bt_conn_unref(conn);
}





struct ppgInfo my_ppgSensor;
struct ble_battery_info my_battery ;  // work-queue instance for batter level

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


void connected(struct bt_conn* conn, uint8_t err){
  struct bt_conn_info info; 
  struct bt_conn *previous;
  char addr[BT_ADDR_LE_STR_LEN];

  if (err) {
    printk("Connection failed (err %u)\n", err);
    return;
  }

  {
    k_spinlock_key_t key = k_spin_lock(&my_connection_lock);

    previous = my_connection;
    my_connection = bt_conn_ref(conn);
    k_spin_unlock(&my_connection_lock, key);
  }
  if (previous != NULL) {
    bt_conn_unref(previous);
  }

  if(bt_conn_get_info(conn, &info))
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
  struct bt_conn *previous = NULL;
  // Stop timer and do all the cleanup
  printk("Disconnected (reason %u)\n", reason);

  {
    k_spinlock_key_t key = k_spin_lock(&my_connection_lock);

    if (my_connection == conn) {
      previous = my_connection;
      my_connection = NULL;
    }
    k_spin_unlock(&my_connection_lock, key);
  }
  if (previous != NULL) {
    bt_conn_unref(previous);
  }
  
  connectedFlag=false;

}




void reset_device(bool reset_bad_blocks){
  
  if (!IS_ENABLED(CONFIG_USB_ALWAYS_ON)){
    usb_disable();
  } 
    //reset the flash memory first
  LOG_INF("Performing Chip Erase...\n");
  // get our flash device from device tree, which is defined in nrf5340dk_nrf5340_cpuapp.overlay
  const struct device* flash_device = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
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

void exit_ecg_collection_mode(void);
void enter_ecg_collection_mode(void);

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
  
  if (collecting_data && !host_wants_collection) {
    exit_ecg_collection_mode();
  } 
  else if(!collecting_data && host_wants_collection) {
    enter_ecg_collection_mode();
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
void blink_led(gpio_pin_t pin);



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
        if ((val == 130 || val == 150) && !collecting_data){

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
          usb_enable(NULL);
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
#ifdef CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES

uint8_t configRead[6] = {0,0,0,0,0,0};
uint8_t ppgQuality[4] = {0};

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
      if((buffer[1] & PPG_ENABLE) == PPG_ENABLE){
        ppgConfig.isEnabled = true;
        configRead[1] = configRead[1] | 0x01; 
      }
      else if((buffer[1] & PPG_ENABLE) == 0x00){
        ppgConfig.isEnabled = false;
        configRead[1] = configRead[1] & 0xFE; 
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
  configRead[0] = PPG_BLE_ENABLE;
  configRead[1] = PPG_ENABLE;
  configRead[2] = ppgConfig.green_intensity;
  configRead[3] = ppgConfig.infraRed_intensity;
  configRead[4] = 0U;
  configRead[5] = PPG_FIXED_256HZ_STATUS;
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
