#ifndef FS_CRYPT_H_
#define FS_CRYPT_H_

/* In-place AES-128 encryption for file system sectors.
 *
 * Uses XTS-AES-128 (IEEE 1619), the standard mode for disk encryption:
 *  - the output overwrites the input in the same buffer, same length
 *  - the sector number is mixed in, so the same data written to two different
 *    sectors gives different ciphertext
 *
 * The AES runs on the nRF5340's CryptoCell-312 hardware through PSA Crypto. That
 * needs the nrf_security options in prj.conf (see the "File system encryption"
 * section there).
 *
 * XTS takes two 128-bit AES keys: one encrypts the data, the other encrypts the
 * sector number. They must be different and kept secret.
 *
 * Buffers must be in RAM and a multiple of 16 bytes (one AES block). A 4096-byte
 * dhara sector is 256 blocks. If a call fails, the buffer contents are undefined.
 *
 * Callers should serialize calls; the disk driver already does this with
 * disk_access_mutex.
 */

#include <stddef.h>
#include <stdint.h>

#define FS_CRYPT_KEY_LEN	16U
#define FS_CRYPT_BLOCK_LEN	16U

/* Starts PSA Crypto, runs a known-answer self-test on the hardware, and loads both
 * keys. The caller can wipe its copies of the keys afterwards. Calling it again
 * replaces the keys.
 *
 * Returns 0, -EINVAL if a key is NULL or the two keys match, or -EIO if the crypto
 * hardware fails or the self-test gives a wrong answer.
 */
int fs_crypt_init(const uint8_t data_key[FS_CRYPT_KEY_LEN],
		  const uint8_t tweak_key[FS_CRYPT_KEY_LEN]);

/* Destroys the keys. Encrypt and decrypt fail until init is called again. */
void fs_crypt_deinit(void);

/* Encrypts len bytes of buf in place. sector is the disk sector the data belongs
 * to, and the same number has to be passed to fs_crypt_decrypt() to get it back.
 *
 * Returns 0, -EINVAL if buf is NULL or len is not a nonzero multiple of 16, -EPERM
 * if fs_crypt_init() has not succeeded, or -EIO if the crypto hardware fails.
 */
int fs_crypt_encrypt(uint8_t *buf, size_t len, uint64_t sector);

/* Decrypts len bytes of buf in place. Same arguments and returns as encrypt. */
int fs_crypt_decrypt(uint8_t *buf, size_t len, uint64_t sector);

#endif /* FS_CRYPT_H_ */
