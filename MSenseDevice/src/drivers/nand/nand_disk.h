#include <zephyr/drivers/disk.h>

extern struct disk_info sdmmc_disk;

void set_read_only(bool enable);