/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SDMMC disk driver using zephyr SD subsystem
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/storage/disk_access.h>
#include "spi_nand.h"
#include "nand_disk.h"

#define DT_DRV_COMPAT senselab_nanddisk

LOG_MODULE_REGISTER(nand_disk, 4);

enum sd_status {
	SD_UNINIT,
	SD_ERROR,
	SD_OK,
};

struct sdmmc_config {
	//const struct device *host_controller;
};

struct sdmmc_data {
	//struct sd_card card;
	enum sd_status status;
	char *name;
	bool read_for_filename;
};


// File System Controls
bool CheckDuplicateAccess = false;
bool VerifyWrites = true;

// The current sector offset, caused by the file system having to move data in a different sector due to the prescense of a bad block.
int total_bad_sectors = 0;

/*perhaps we could make this bad blocks.
TODO: Make this system calculate this in offline mode.
Essentially the idea for this is that when we are acessing sectors, we check to see how many bad sectors are below, and that determines the offset to use,
since bad sectors aren't used and the next sector over is used.
*/
uint32_t bad_sectors[5000];

// Config sector monitoring
int sector_write_list[5000] = { 0 };
int unique_sectors_written = 0;


int file_table_sector_num = 60;

bool read_only = false;


#define FILE_TABLE_NAND_PARTITION	slot0_partition


#ifdef CONFIG_PARTITION_MANAGER_ENABLED
#define FILETABLE_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(PM_FATFILETABLE_PARTITION_NAME)
#define FILETABLE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(PM_FATFILETABLE_PARTITION_NAME)
#else

#define FILETABLE_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(FILE_TABLE_NAND_PARTITION)
#define FILETABLE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(FILE_TABLE_NAND_PARTITION)
#endif 

 
// eventually we should just change this to blocks.
static int register_bad_sector(uint32_t sector_num){
	
	bad_sectors[total_bad_sectors] = sector_num;
	total_bad_sectors++;
	return total_bad_sectors;
}

static int get_sector_offset(int sector_num){
	
	for (int x = 0; x < total_bad_sectors; x++){
		if (bad_sectors[x] <= sector_num){
			sector_num++;
		}
	}
	return sector_num;
}


/* We will need to make this all be enabled by a KConfig. */

static int duplicate_sector_access(struct disk_info* disk, int sector_num){
	if (unique_sectors_written >= 80000){
		return 0;
	}
	for (int i = 0; i < unique_sectors_written; i++){
		if (sector_write_list[i] == sector_num){
			LOG_WRN("error: attempted duplicate write for sector %i", sector_num);
			// TODO: Determine this offline first.
			
			return -1;
		}
	}
	sector_write_list[unique_sectors_written] = sector_num;
	unique_sectors_written++;
	return 0;
}


uint8_t check_buffer[4096];
static int duplicate_sector_access2(const struct disk_info* disk, int sector_num){

	disk_access_read(disk->name, check_buffer, sector_num, 1);
	for (int x = 0; x < 4096; x++){
		if (check_buffer[x] != 0xff){
			LOG_WRN("error: attempted duplicate write for sector %i, total bad %d", sector_num, total_bad_sectors);
			register_bad_sector(sector_num);
			return -1;
			
		}
	}
	return 0;
}

/* handle a duplicate write by rewritting the entire block. Configurable because it will use up a lot of erase write cycles. */
#ifdef CONFIG_RAW_NAND_ALLOW_PAGE_REWRITE
int duplicate_writes = 0;
int duplicate_write_max = 50;
char sector_buffer[64][4096];
int rewrite_page(struct disk_info* disk, void* buffer, int sector_num){
	if (duplicate_writes < duplicate_write_max){
	// Get the addresses for the starting page of the block and the page relative to the block number
	int current_page_in_block = sector_num % 64;
	int block_num = sector_num / 64;
	int starting_page_number = sector_num - current_page_in_block;
	// read in the block the page is located in to the buffer
	disk_access_read(disk, sector_buffer, starting_page_number, 64);
	spi_nand_block_erase(disk->dev, sector_num);

	// modify the desired buffer with the updated page contents
	memcpy(sector_buffer[current_page_in_block], buffer, 4096);

	// fill the block back up with the buffer
	for (int x = starting_page_number; x < starting_page_number + 64; x++){
		spi_nand_page_write(disk->dev, x, sector_buffer[x], 4096);
	}
	duplicate_writes++;
	}
	return 0;
}


#endif

int erase_file_table() {
	const struct device* soc_flash = FILETABLE_PARTITION_DEVICE;
	return flash_erase(soc_flash, FILETABLE_PARTITION_OFFSET, 4096*file_table_sector_num);
}




char nor_buffer[256];
static int flash_nor_adjustment_write(const struct device* dev, off_t address, char* buf, size_t size){
	int ret;
	
	int memory_corrections = 0;
	int cmp;
	off_t computed_address;
	for (int nor_page = 0; nor_page <= 4096; nor_page += 256){
		computed_address = address + nor_page;
		ret = flash_read(dev, computed_address, nor_buffer, 256);
		void* adjusted_buf = &buf[nor_page];
		cmp = memcmp(adjusted_buf, nor_buffer, 256);

		if (cmp != 0) {
			ret = flash_erase(dev, computed_address, 256);
			ret = flash_write(dev, computed_address, adjusted_buf, 256);
		}
	}
}


static int file_table_access(void* buf, int sector_num, bool write){
	
	int ret;
	const struct device* soc_flash = FILETABLE_PARTITION_DEVICE;
	struct flash_pages_info* page_info_ptr;
	off_t address = FILETABLE_PARTITION_OFFSET + (4096*sector_num);
	//flash_get_page_info_by_offs(soc_flash, address, page_info_ptr);

	//sector cannot be greater than the allocated file table segment size
	if (sector_num > file_table_sector_num){
		LOG_ERR("sector num %d too big for file allocation table", sector_num);
		return -1;
	}

	if (write){

		ret = flash_erase(soc_flash, address, 4096);
		ret = flash_write(soc_flash, address, buf, 4096);	
	}
	else {
		ret = flash_read(soc_flash, address, buf, 4096);
	}
	return ret;
}

void set_read_only(bool enable){
	read_only = enable;
}

static int disk_nand_access_init(struct disk_info *disk)
{
	const struct device* dev = disk->dev;
	
	int sucess = spi_init(dev);
	return 0;
}


static int disk_acess_init2(struct disk_info *disk){
	return 0;
} 

static int disk_nand_access_status(struct disk_info *disk)
{
	//LOG_DBG("Accessing Status");
	const struct device* dev = disk->dev;
	
	/*const struct sdmmc_config* cfg = dev->config;
	struct sdmmc_data *data = dev->data;

	if (!sd_is_card_present(cfg->host_controller)) {
		return DISK_STATUS_NOMEDIA;
	}
	if (data->status == SD_OK) {
		return DISK_STATUS_OK;
	} else {
		return DISK_STATUS_UNINIT;
	}
	*/
	//uint8_t status = spi_rdsr(dev);
	if (read_only){
		return DISK_STATUS_WR_PROTECT;
	}
	return DISK_STATUS_OK;

}

static int disk_nand_access_read(struct disk_info* disk, uint8_t *buf,
				 uint32_t sector, uint32_t count)
{

	
	LOG_DBG("performing disk read at sector %i", sector);
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;
	
	

	off_t addr;
	int ret = 0;

	if (count > 1){
	LOG_WRN("count: %i", count);
	}

	for (int x = 0; x < count; x++) {

		if (sector+x < file_table_sector_num){
		//memcpy(buf, sector_buffer[sector], 4096);
		ret = file_table_access(buf, sector+x, false);
		continue;
		}
		int non_corrupt_sector = get_sector_offset(sector+x);
		addr = convert_page_to_address(dev, non_corrupt_sector);
		ret = spi_nand_page_read(dev, addr, &buf[x*4096]);
	}
	
	//lol
	return ret; //sdmmc_read_blocks(&data->card, buf, sector, count);
}

static int disk_nand_access_write(struct disk_info *disk, const uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	LOG_DBG("performing disk write at sector %i", sector);
	if (count > 1){
	LOG_INF("count: %i", count);
	}
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;
	int ret = 0;
	off_t addr;
	


	
	for (int x = 0; x < count; x++){
		// Do we know what count means?
		if (sector+x < file_table_sector_num){
			//memcpy(sector_buffer[sector], buf, 4096);
			file_table_access(buf, sector+x, true);
			continue;
		}
		else {
		int error;
		int sector_num = x+sector;
		if (CheckDuplicateAccess){
			for (int y = 0; y < 1000; y++){
				sector_num = get_sector_offset(x+sector);
				
				int error = duplicate_sector_access2(disk, sector_num);
				if (error == -1){
					continue;
				}
				break;	
			}
		}

		addr = convert_page_to_address(dev, sector_num);
		ret = spi_nand_page_write(dev, addr, &buf[x*4096], 4096);
		// perhaps a read back here, but we need to do something about a bad sector that is fully erased fine, or a sector that returns a bad ret value.
		if (ret > 3){
			//TODO: Implement read back logic here.
		}
		}

		
		
	}
	
	
	return ret; //sdmmc_write_blocks(&data->card, buf, sector, count);
}

static int disk_nand_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	LOG_INF("Acessing ioctl with cmd %d", cmd);
	const struct device *dev = disk->dev;
	struct sdmmc_data *data = dev->data;

    switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		uint32_t sectors = dev_flash_size(dev) / dev_page_size(dev);
		
		LOG_INF("sectors avalible: %i", sectors);
		(*(uint32_t *)buf) = sectors;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		(*(uint32_t *)buf) = dev_page_size(dev); 
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		(*(uint32_t *)buf) = dev_page_size(dev)*64;
		break;
	case DISK_IOCTL_CTRL_SYNC:
		/* Ensure card is not busy with data write.
		 * Note that SD stack does not support enabling caching, so
		 * cache flush is not required here
		 */
		return 0; //spi_flash_wait_until_ready(dev);
	default:
		return -ENOTSUP;
	}
	return 0;


	return 0; //sdmmc_ioctl(&data->card, cmd, buf);
}

static const struct disk_operations sdmmc_disk_ops = {
	.init = disk_acess_init2,
	.status = disk_nand_access_status,
	.read = disk_nand_access_read,
	.write = disk_nand_access_write,
	.ioctl = disk_nand_access_ioctl,
};

struct disk_info sdmmc_disk = {
	.ops = &sdmmc_disk_ops,
};

#define CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE 4096


BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size),
	     "jedec,spi-nor size required for non-runtime SFDP page layout");

#if defined(CONFIG_FLASH_PAGE_LAYOUT)


BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size),
	     "jedec,spi-nor size required for non-runtime SFDP page layout");


#define INST_0_BYTES (DT_INST_PROP(0, size))

BUILD_ASSERT(SPI_NOR_IS_SECTOR_ALIGNED(CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE),
	     "SPI_NOR_FLASH_LAYOUT_PAGE_SIZE must be multiple of 4096");


#define LAYOUT_PAGES_COUNT (INST_0_BYTES / CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE)

BUILD_ASSERT((CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE * LAYOUT_PAGES_COUNT)
	     == INST_0_BYTES,
	     "SPI_NOR_FLASH_LAYOUT_PAGE_SIZE incompatible with flash size");

#endif 



static const struct spi_flash_config spi_flash_config_0 = 
{
	
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8),
				    0),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif

#if !defined(CONFIG_SPI_NOR_SFDP_RUNTIME)

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.layout = {
		.pages_count = LAYOUT_PAGES_COUNT,
		.pages_size = CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE,
	},
#undef LAYOUT_PAGES_COUNT
#endif 

	.flash_size = DT_INST_PROP(0, size),
	.jedec_id = DT_INST_PROP(0, jedec_id),

#if DT_INST_NODE_HAS_PROP(0, has_lock)
	.has_lock = DT_INST_PROP(0, has_lock),
#endif
#if defined(CONFIG_SPI_NOR_SFDP_MINIMAL)		\
	&& DT_INST_NODE_HAS_PROP(0, enter_4byte_addr)
	.enter_4byte_addr = DT_INST_PROP(0, enter_4byte_addr),
#endif
#ifdef CONFIG_SPI_NOR_SFDP_DEVICETREE
	.bfp_len = sizeof(bfp_data_0) / 4,
	.bfp = (const struct jesd216_bfp *)bfp_data_0,
	
#endif 

#endif 

};

static struct spi_nor_data spi_nor_data_0;




static int disk_sdmmc_init(const struct device *dev)
{
	//struct sdmmc_data* data = dev->data;
	//data->status = SD_UNINIT;

	//spi_nor_data* nand_data = dev->data;
	LOG_INF("Initializing disk Registration"); 

	sdmmc_disk.dev = dev;
	sdmmc_disk.name = "SD";//dev->name;
	disk_nand_access_init(&sdmmc_disk);
	return disk_access_register(&sdmmc_disk);
}

/*
#define DISK_ACCESS_SDMMC_INIT(n)						\
	static const struct sdmmc_config sdmmc_config_##n = {			\
		.host_controller = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};									\
										\
	static struct sdmmc_data sdmmc_data_##n = {				\
		.name = CONFIG_SDMMC_VOLUME_NAME,				\
	};									\
										\

*/

	DEVICE_DT_INST_DEFINE(0,						
			&disk_sdmmc_init,					
			NULL,							
			&spi_nor_data_0,					
			&spi_flash_config_0,					
			POST_KERNEL,						
			80,				
			NULL);

//DT_INST_FOREACH_STATUS_OKAY(DISK_ACCESS_SDMMC_INIT)
