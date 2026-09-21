#ifndef BAD_BLOCK_H
#define BAD_BLOCK_H


#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <zephyr/device.h>


/* Persisted through the settings subsystem so the manufacturer bad-block scan
 * only ever runs on the very first boot of a device.
 */
extern bool bad_block_scan_done;

/* Number of entries currently live in the bad block table. */
extern int total_bad_blocks;

/* Brings up the settings subsystem, restores the bad block table, and runs the
 * manufacturer bad-block scan if it has never been run on this device.
 * Call once from spi_init().
 */
int bad_block_storage_init(const struct device *dev);

int save_bad_blocks_arr();
int load_bad_blocks_arr();
int erase_bad_blocks_arr();
int register_bad_block(uint32_t sector_num);

/* Unconditional form of register_bad_block(), for callers that already know the
 * block is bad and must not be gated on the first boot scan having run.
 */
int mark_bad_block(uint32_t sector_num);

/* True if the block holding this sector is recorded bad. */
bool is_block_bad(uint32_t sector_num);
int get_sector_offset(int sector_num);
void print_bad_block_info();

int bad_page_sim_init(const struct device* dev);
int multi_nand_page_read_badsim_wrapper(const struct device* dev, uint32_t page_number, void* buffer);
int multi_nand_page_write_badsim_wrapper(const struct device* dev, uint32_t page_number, const void* buffer, size_t size);
int add_bad_page(uint32_t page);
int remove_bad_page(uint32_t page);

#endif /* BAD_BLOCK_H */
