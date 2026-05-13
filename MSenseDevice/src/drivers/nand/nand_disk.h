#include <zephyr/drivers/disk.h>

extern bool VerifyWrites;
extern bool CheckDuplicateAccess;

extern struct disk_info sdmmc_disk;

void set_read_only(bool enable);


extern const int file_table_sector_num;