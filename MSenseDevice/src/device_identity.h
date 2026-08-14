#ifndef DEVICE_IDENTITY_H_
#define DEVICE_IDENTITY_H_

#include <stdint.h>

#define MSENSE_DEVICE_ID_LEN 8U
#define MSENSE_DEVICE_ID_HEX_LEN 16U
#define MSENSE_BLE_NAME_LEN 16U

int msense_device_identity_init(void);

const uint8_t *msense_device_identity_bytes(void);

const char *msense_device_identity_hex(void);

const char *msense_device_identity_name(void);

#endif /* DEVICE_IDENTITY_H_ */
