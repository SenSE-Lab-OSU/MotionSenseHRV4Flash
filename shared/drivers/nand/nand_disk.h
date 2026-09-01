#include <zephyr/drivers/disk.h>

extern bool VerifyWrites;
extern bool CheckDuplicateAccess;

extern struct disk_info sdmmc_disk;

void set_read_only(bool enable);

bool get_read_only();

/* When read-only is enabled, every caller, including USB mass storage,
 * receives -EROFS for writes. This is the storage ownership contract. */

void print_flash_status_info();

int disk_nand_access_read(struct disk_info* disk, uint8_t *buf,
				 uint32_t sector, uint32_t count);

extern const int file_table_sector_num;
