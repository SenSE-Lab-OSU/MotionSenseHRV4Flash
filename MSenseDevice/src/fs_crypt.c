/* In-place XTS-AES-128 for file system sectors. See fs_crypt.h.
 *
 * The AES itself runs on the nRF5340's CryptoCell-312 through the PSA Crypto API
 * (nrf_security). This file only adds the XTS layer on top, which NCS does not
 * offer for this chip: XOR each 16-byte block with a per-block value derived from
 * the sector number, run plain AES (ECB) over the whole buffer in one hardware
 * call, then XOR again.
 *
 * The PC test (tests/fs_crypt/test_fs_crypt.c) builds this same file against a
 * software stand-in for PSA.
 */

// Only built when file system encryption is on, since it needs the PSA crypto options.
#ifdef CONFIG_MSENSE_FS_ENCRYPTION

#include "fs_crypt.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <psa/crypto.h>

static psa_key_id_t data_key_id;
static psa_key_id_t tweak_key_id;
static bool keys_loaded;

static void secure_zero(void *p, size_t n)
{
	volatile uint8_t *v = (volatile uint8_t *)p;

	while (n--) {
		*v++ = 0;
	}
}

/* The key is copied into the PSA key store and lives only in RAM until destroyed. */
static int import_aes_key(const uint8_t key[FS_CRYPT_KEY_LEN], psa_key_id_t *id)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128);

	status = psa_import_key(&attr, key, FS_CRYPT_KEY_LEN, id);
	psa_reset_key_attributes(&attr);

	return (status == PSA_SUCCESS) ? 0 : -EIO;
}

/* Plain AES over every 16-byte block of buf, in place. */
static int aes_ecb(psa_key_id_t key, bool encrypt, uint8_t *buf, size_t len)
{
	psa_status_t status;
	size_t out_len = 0;

	// PSA allows the input and output buffers to be the same
	if (encrypt) {
		status = psa_cipher_encrypt(key, PSA_ALG_ECB_NO_PADDING, buf, len, buf, len,
					    &out_len);
	} else {
		status = psa_cipher_decrypt(key, PSA_ALG_ECB_NO_PADDING, buf, len, buf, len,
					    &out_len);
	}

	return ((status == PSA_SUCCESS) && (out_len == len)) ? 0 : -EIO;
}

/* Multiply the tweak by x in GF(2^128). The tweak is little endian. */
static void xts_next_tweak(uint8_t t[16])
{
	uint8_t carry = 0;

	for (int i = 0; i < 16; i++) {
		uint8_t next_carry = (uint8_t)(t[i] >> 7);

		t[i] = (uint8_t)((t[i] << 1) | carry);
		carry = next_carry;
	}
	t[0] ^= (uint8_t)(carry * 0x87U);
}

/* XOR every block of buf with its tweak, starting from first_tweak. */
static void xts_xor_tweaks(uint8_t *buf, size_t len, const uint8_t first_tweak[16])
{
	uint8_t t[16];

	memcpy(t, first_tweak, sizeof(t));
	for (size_t off = 0; off < len; off += FS_CRYPT_BLOCK_LEN) {
		for (int i = 0; i < 16; i++) {
			buf[off + i] ^= t[i];
		}
		xts_next_tweak(t);
	}
	secure_zero(t, sizeof(t));
}

static int xts_crypt(psa_key_id_t data_key, psa_key_id_t tweak_key, uint8_t *buf,
		     size_t len, uint64_t sector, bool encrypt)
{
	uint8_t tweak[16];
	int ret;

	if ((buf == NULL) || (len == 0) || ((len % FS_CRYPT_BLOCK_LEN) != 0)) {
		return -EINVAL;
	}

	// first tweak is the sector number, little endian, encrypted with the tweak key
	memset(tweak, 0, sizeof(tweak));
	for (int i = 0; i < 8; i++) {
		tweak[i] = (uint8_t)(sector >> (8 * i));
	}
	ret = aes_ecb(tweak_key, true, tweak, sizeof(tweak));
	if (ret == 0) {
		xts_xor_tweaks(buf, len, tweak);
		ret = aes_ecb(data_key, encrypt, buf, len);
		xts_xor_tweaks(buf, len, tweak);
	}

	secure_zero(tweak, sizeof(tweak));
	return ret;
}

/* Runs IEEE 1619 XTS-AES-128 test vector 2 through the hardware, in place, both
 * ways. Catches a crypto setup that builds but gives wrong answers on the board.
 */
static int self_test(void)
{
	static const uint8_t expected[32] = {
		0xc4, 0x54, 0x18, 0x5e, 0x6a, 0x16, 0x93, 0x6e, 0x39, 0x33, 0x40, 0x38,
		0xac, 0xef, 0x83, 0x8b, 0xfb, 0x18, 0x6f, 0xff, 0x74, 0x80, 0xad, 0xc4,
		0x28, 0x93, 0x82, 0xec, 0xd6, 0xd3, 0x94, 0xf0,
	};
	uint8_t key1[16], key2[16], buf[32], plain[32];
	psa_key_id_t k1 = 0, k2 = 0;
	int ret;

	memset(key1, 0x11, sizeof(key1));
	memset(key2, 0x22, sizeof(key2));
	memset(plain, 0x44, sizeof(plain));
	memcpy(buf, plain, sizeof(buf));

	ret = import_aes_key(key1, &k1);
	if (ret == 0) {
		ret = import_aes_key(key2, &k2);
	}
	if (ret == 0) {
		ret = xts_crypt(k1, k2, buf, sizeof(buf), 0x3333333333ULL, true);
	}
	if ((ret == 0) && (memcmp(buf, expected, sizeof(buf)) != 0)) {
		ret = -EIO;
	}
	if (ret == 0) {
		ret = xts_crypt(k1, k2, buf, sizeof(buf), 0x3333333333ULL, false);
	}
	if ((ret == 0) && (memcmp(buf, plain, sizeof(buf)) != 0)) {
		ret = -EIO;
	}

	psa_destroy_key(k1);
	psa_destroy_key(k2);
	return ret;
}

int fs_crypt_init(const uint8_t data_key[FS_CRYPT_KEY_LEN],
		  const uint8_t tweak_key[FS_CRYPT_KEY_LEN])
{
	int ret;

	if ((data_key == NULL) || (tweak_key == NULL)) {
		return -EINVAL;
	}
	// XTS loses its security guarantees if both halves of the key are the same
	if (memcmp(data_key, tweak_key, FS_CRYPT_KEY_LEN) == 0) {
		return -EINVAL;
	}

	fs_crypt_deinit();

	if (psa_crypto_init() != PSA_SUCCESS) {
		return -EIO;
	}

	ret = self_test();
	if (ret == 0) {
		ret = import_aes_key(data_key, &data_key_id);
	}
	if (ret == 0) {
		ret = import_aes_key(tweak_key, &tweak_key_id);
	}
	if (ret != 0) {
		fs_crypt_deinit();
		return ret;
	}

	keys_loaded = true;
	return 0;
}

void fs_crypt_deinit(void)
{
	keys_loaded = false;
	// destroying a null key id is a harmless no-op
	psa_destroy_key(data_key_id);
	psa_destroy_key(tweak_key_id);
	data_key_id = 0;
	tweak_key_id = 0;
}

int fs_crypt_encrypt(uint8_t *buf, size_t len, uint64_t sector)
{
	if (!keys_loaded) {
		return -EPERM;
	}
	return xts_crypt(data_key_id, tweak_key_id, buf, len, sector, true);
}

int fs_crypt_decrypt(uint8_t *buf, size_t len, uint64_t sector)
{
	if (!keys_loaded) {
		return -EPERM;
	}
	return xts_crypt(data_key_id, tweak_key_id, buf, len, sector, false);
}

#endif /* CONFIG_MSENSE_FS_ENCRYPTION */
