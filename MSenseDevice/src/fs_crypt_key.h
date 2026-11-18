#ifndef FS_CRYPT_KEY_H_
#define FS_CRYPT_KEY_H_

/* Development key for file system encryption (src/fs_crypt.c).
 *
 * Every device built from this source shares this key, and anyone with this file
 * or the firmware .hex can read it. Use it for development only; real devices
 * should get their own key when they are programmed.
 *
 * The same 32 bytes as one hex string (data key first, then tweak key), which is
 * the form PC tools such as Python's cryptography AES-XTS expect:
 *   25f6a2a155cb7d7855e2d22d6377260c5d4d1efea0aeb33122d4babc175fdd68
 *
 * Changing this key makes anything already written with the old one unreadable.
 *
 * Include this from one .c file only.
 */

#include <stdint.h>

#include "fs_crypt.h"

static const uint8_t fs_crypt_dev_data_key[FS_CRYPT_KEY_LEN] = {
	0x25, 0xf6, 0xa2, 0xa1, 0x55, 0xcb, 0x7d, 0x78,
	0x55, 0xe2, 0xd2, 0x2d, 0x63, 0x77, 0x26, 0x0c,
};

static const uint8_t fs_crypt_dev_tweak_key[FS_CRYPT_KEY_LEN] = {
	0x5d, 0x4d, 0x1e, 0xfe, 0xa0, 0xae, 0xb3, 0x31,
	0x22, 0xd4, 0xba, 0xbc, 0x17, 0x5f, 0xdd, 0x68,
};

#endif /* FS_CRYPT_KEY_H_ */
