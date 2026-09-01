#ifndef BLESERVICE_H_
#define BLESERVICE_H_
#include <zephyr/types.h>
#include <sys/types.h>
#include <zephyr/bluetooth/bluetooth.h>


extern bool connectedFlag;
extern bool collecting_data;
extern bool host_wants_collection;
extern bool battery_low;
extern bool file_system_full;
extern bool file_system_malfunction;
extern bool battery_charging;



// Main Service UUID 
#define CONTROL_SERVICE_UUID  0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x30, 0xC9, 0x39, 0xDA    

#define WRITE_ENABLE_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x31, 0xC9, 0x39, 0xDA

#define WRITE_DATE_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x32, 0xC9, 0x39, 0xDA

#define WRITE_PATIENT_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x33, 0xC9, 0x39, 0xDA

#define WRITE_RESET_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x34, 0xC9, 0x39, 0xDA

/* Legacy UUID retained for active ECG manual test-file commands 130 and 150. */
#define WRITE_DEVICE_NAME_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x35, 0xC9, 0x39, 0xDA

#define STATUS_SERVICE_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x40, 0xC9, 0x39, 0xDA

#define READ_STORAGE_LEFT_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x41, 0xC9, 0x39, 0xDA

#define READ_STATUS_REGISTER_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x42, 0xC9, 0x39, 0xDA

#define READ_UPTIME_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x43, 0xC9, 0x39, 0xDA

/* Legacy motion-update UUIDs retained for client compatibility. */
#define TIMING_UPDATE_SERVICE_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x50, 0xC9, 0x39, 0xDA

#define TIMING_UPDATE_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x51, 0xC9, 0x39, 0xDA

void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);





int general_ble_notification(uint8_t* data, uint8_t len, int service, int characteristic);
void status_reg_ble_notification();

int storage_ble_notification(uint8_t* data, uint8_t len);

/**
 * @brief Start the collection-local RTC0 counter at 512 Hz.
 *
 * The counter is cleared before it starts. This API is called only by the
 * collection-mode lifecycle while its mutex is held.
 */
int rtc0_collection_counter_start(void);

/**
 * @brief Stop and uninitialize the collection-local RTC0 counter.
 */
void rtc0_collection_counter_stop(void);

/**
 * @brief Read the active 32-bit wrap-extended RTC0 counter in 512 Hz ticks.
 *
 * The 24-bit hardware counter is extended in software and wraps naturally
 * after 2^32 ticks. Callers must provide a non-NULL output pointer and must
 * tolerate -EACCES outside collection mode.
 */
int rtc0_collection_counter_get(uint32_t *ticks);

/**
 * @brief Start 3-second BLE timing notifications from the RTC0 compare event.
 *
 * This must be called after the 512 Hz collection counter is running.
 */
int rtc0_collection_notification_start(void);


/* These APIs only signal the authoritative ECG transition owner in main.c. */
void request_ecg_collection_mode(bool enable);
int request_ecg_storage_reboot(void);
int request_ecg_storage_reset(bool reset_bad_blocks);
int request_ecg_manual_test_file(uint8_t command);
void request_ecg_storage_fault(void);
void ecg_filesystem_log_enable(void);
void ecg_filesystem_log_disable(void);
void ecg_filesystem_log_disable_and_wait(void);
void reset_device(bool reset_bad_blocks);

#endif
