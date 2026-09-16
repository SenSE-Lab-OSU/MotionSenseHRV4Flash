#include <string.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "nand.h"
#include "msense_dhara.h"
#include "../nand/spi_nand.h"
#include "../nand/bad_page.h"

LOG_MODULE_REGISTER(dhara_nand, CONFIG_FLASH_LOG_LEVEL);

/* 4096 byte pages, 64 pages per erase block. Both have to be powers of two for
 * dhara, which treats a page number as (block << log2_ppb) | page_in_block.
 */
#define DHARA_LOG2_PAGE_SIZE	12
#define DHARA_LOG2_PPB		6
#define DHARA_PAGE_SIZE		(1U << DHARA_LOG2_PAGE_SIZE)

BUILD_ASSERT(DHARA_PAGE_SIZE == 4096, "dhara page size must match the nand page size");
BUILD_ASSERT((1U << DHARA_LOG2_PPB) == NAND_PAGES_PER_ERASE_BLOCK,
	     "dhara pages per block must match NAND_PAGES_PER_ERASE_BLOCK");
BUILD_ASSERT(offsetof(struct msense_dhara_nand, base) == 0,
	     "dhara_nand must be the first member so nand_dev() can cast through it");

static struct msense_dhara_nand msense_nand = {
	.base = {
		.log2_page_size = DHARA_LOG2_PAGE_SIZE,
		.log2_ppb = DHARA_LOG2_PPB,
		.num_blocks = 0,
	},
	.dev = NULL,
};

/* Bounce buffer for the partial reads dhara asks for and for page copies. The
 * chip only ever reads or writes whole pages, so everything has to land here
 * first. Single buffer is safe because all disk access is serialized by the
 * caller's mutex and dhara never reenters itself.
 */
static uint8_t dhara_page_buffer[DHARA_PAGE_SIZE];

/* base is the first member, so the dhara_nand dhara hands back is the context. */
static inline const struct device *nand_dev(const dhara_nand *n)
{
	return ((const struct msense_dhara_nand *)n)->dev;
}

/* dhara indexes blocks; the rest of this driver indexes pages. */
static inline uint32_t block_first_page(dhara_block_t b)
{
	return convert_block_to_page(0, b);
}

dhara_nand *msense_dhara_nand_init(const struct device *dev)
{
	uint32_t total_pages = dev_flash_size(dev) / dev_page_size(dev);

	msense_nand.dev = dev;
	msense_nand.base.num_blocks = total_pages / NAND_PAGES_PER_ERASE_BLOCK;

	LOG_INF("dhara nand: %u blocks of %u pages", msense_nand.base.num_blocks,
		NAND_PAGES_PER_ERASE_BLOCK);

	return &msense_nand.base;
}

int dhara_nand_is_bad(const dhara_nand* n, dhara_block_t b)
{
	return is_sector_bad(block_first_page(b)) ? 1 : 0;
}

void dhara_nand_mark_bad(const dhara_nand* n, dhara_block_t b)
{
	// unconditional: dhara has decided this block is bad, so the first boot scan
	// gate that register_bad_sector() applies must not suppress it
	mark_bad_sector(block_first_page(b));
}

int dhara_nand_erase(const dhara_nand* n, dhara_block_t b, dhara_error_t* err)
{
	if (multi_nand_block_erase(nand_dev(n), block_first_page(b)) != 0){
		dhara_set_error(err, DHARA_E_BAD_BLOCK);
		return -1;
	}
	return 0;
}

int dhara_nand_prog(const dhara_nand* n, dhara_page_t p, const uint8_t* data, dhara_error_t* err)
{
	if (multi_nand_page_write(nand_dev(n), p, data, DHARA_PAGE_SIZE) != 0){
		dhara_set_error(err, DHARA_E_BAD_BLOCK);
		return -1;
	}
	return 0;
}

int dhara_nand_is_free(const dhara_nand* n, dhara_page_t p)
{
	return multi_nand_page_is_erased(nand_dev(n), p) ? 1 : 0;
}

int dhara_nand_read(const dhara_nand* n, dhara_page_t p, size_t offset, size_t length, uint8_t* data, dhara_error_t* err)
{
	if (offset > DHARA_PAGE_SIZE || length > (DHARA_PAGE_SIZE - offset)){
		LOG_ERR("read past end of page: off %u len %u", (unsigned int)offset,
			(unsigned int)length);
		dhara_set_error(err, DHARA_E_ECC);
		return -1;
	}

	// the chip reads whole pages, so take the slice out of the bounce buffer
	if (multi_nand_page_read(nand_dev(n), p, dhara_page_buffer) != 0){
		dhara_set_error(err, DHARA_E_ECC);
		return -1;
	}

	memcpy(data, &dhara_page_buffer[offset], length);
	return 0;
}

int dhara_nand_copy(const dhara_nand* n, dhara_page_t src, dhara_page_t dst, dhara_error_t* err)
{
	const struct device *dev = nand_dev(n);

	// no on-chip copy across dies or chips, so it round trips through RAM. Both
	// halves go through ECC because they are ordinary page reads and writes.
	if (multi_nand_page_read(dev, src, dhara_page_buffer) != 0){
		dhara_set_error(err, DHARA_E_ECC);
		return -1;
	}

	if (multi_nand_page_write(dev, dst, dhara_page_buffer, DHARA_PAGE_SIZE) != 0){
		dhara_set_error(err, DHARA_E_BAD_BLOCK);
		return -1;
	}

	return 0;
}
