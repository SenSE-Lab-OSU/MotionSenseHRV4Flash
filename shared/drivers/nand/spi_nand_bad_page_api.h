/* SPDX-License-Identifier: Apache-2.0 */

#ifndef MSENSE_SPI_NAND_BAD_PAGE_API_H_
#define MSENSE_SPI_NAND_BAD_PAGE_API_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <zephyr/device.h>

/*
 * The NAND transport remains product-local until Phase 3. Bad-page support
 * needs only this narrow transport contract, not either product's private
 * spi_nand.h implementation header.
 */
extern int file_table_sector_num;

uint32_t convert_block_to_page(uint32_t page, uint32_t block);
uint32_t convert_page_to_block(uint32_t page_number);
int spi_nand_page_read(const struct device *dev, off_t page_addr, void *dest);
int spi_nand_page_write(const struct device *dev, off_t page_address,
			const void *src, size_t size);

#endif /* MSENSE_SPI_NAND_BAD_PAGE_API_H_ */
