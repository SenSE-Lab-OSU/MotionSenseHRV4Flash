#ifndef BAD_PAGE_H
#define BAD_PAGE_H


#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>


/* Persisted through the settings subsystem so the manufacturer bad-block scan
 * only ever runs on the very first boot of a device.
 */
extern bool bad_block_scan_done;

/* Number of entries currently live in the bad sector table. */
extern int total_bad_sectors;

/* Brings up the settings subsystem, restores the bad sector table, and runs the
 * manufacturer bad-block scan if it has never been run on this device.
 * Call once from spi_init().
 */
int bad_sector_storage_init(const struct device *dev);

int save_bad_sectors_arr();
int load_bad_sectors_arr();
int erase_bad_sectors_arr();
int register_bad_sector(uint32_t sector_num);
int get_sector_offset(int sector_num);
void print_bad_sect_info();

int spi_nand_bad_page_init(const struct device* dev);
int spi_nand_page_read_badsim_wrapper(const struct device* dev, off_t page_addr, void* dest);
int spi_nand_page_write_badsim_wrapper(const struct device* dev, off_t page_address, const void* src, size_t size);
int add_bad_page(off_t page);
int remove_bad_page(off_t page);

#endif /* BAD_PAGE_H */
