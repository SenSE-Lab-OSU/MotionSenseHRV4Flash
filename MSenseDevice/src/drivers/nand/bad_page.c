#include <zephyr/logging/log.h>
#include "spi_nand.h"
#include "nand_disk.h"
#include "bad_page.h"


LOG_MODULE_REGISTER(spi_nand_bad_page, CONFIG_FLASH_LOG_LEVEL);


#define FILE_TABLE_NAND_PARTITION	slot0_partition


#ifdef CONFIG_PARTITION_MANAGER_ENABLED
#define FILETABLE_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(PM_FATFILETABLE_PARTITION_NAME)
#define FILETABLE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(PM_FATFILETABLE_PARTITION_NAME)
#else

#define FILETABLE_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(FILE_TABLE_NAND_PARTITION)
#define FILETABLE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(FILE_TABLE_NAND_PARTITION)
#endif 

#define FILETABLE_PARTITION_DEVICE DEVICE_DT_GET(DT_NODELABEL(mx25u80))
#define FILETABLE_PARTITION_OFFSET 0

#define bad_sector_detect_limit 2000

#ifdef CONFIG_RAW_NAND_BAD_SECTOR_SAVING



// The current sector offset, caused by the file system having to move data in a different sector due to the prescense of a bad block.
int total_bad_sectors = 0;

/* This should work for bad blocks.
TODO: Test this system to save and load in offline mode.
Essentially the idea for this is that when we are acessing sectors, we check to see how many bad sectors are below, and that determines the offset to use,
since bad sectors aren't used and the next sector over is used.
*/


bool use_blocks = false;


uint32_t bad_sectors[bad_sector_detect_limit] = {0};



int save_bad_sectors_arr(){
	
	const struct device* soc_flash = FILETABLE_PARTITION_DEVICE;
	size_t total_bad_sect_arr_size = sizeof(bad_sectors);
	// start address for bad sectors
	off_t address = FILETABLE_PARTITION_OFFSET + (4096*(file_table_sector_num+1));
	int ret = 0;
	ret = flash_erase(soc_flash, address, total_bad_sect_arr_size);
	if (ret == 0){
		ret = flash_write(soc_flash, address, bad_sectors, total_bad_sect_arr_size);
	}
	return ret;
};

int load_bad_sectors_arr()
{
	const struct device* soc_flash = FILETABLE_PARTITION_DEVICE;
	size_t total_bad_sect_arr_size = sizeof(bad_sectors);
	size_t total_bad_sect_count = total_bad_sect_arr_size / sizeof(bad_sectors[0]);
	off_t address = FILETABLE_PARTITION_OFFSET + (4096*(file_table_sector_num+1));
	int ret = 0;
	
	ret = flash_read(soc_flash, address, bad_sectors, total_bad_sect_arr_size);
	// if the memory is all 1s, that means we haven't written to the array yet
	if (bad_sectors[0] == 0xFFFFFFFF){
		LOG_INF("first load bad sect arr, saving");
		for (int x = 0; x < total_bad_sect_count; x++){
			bad_sectors[x] = 0;
		}
		save_bad_sectors_arr();
	}
	LOG_INF("found and loaded bad sector arr");
	for (int x = 0; x < total_bad_sect_count; x++){
		if (bad_sectors[x] != 0){
			total_bad_sectors++;
		}
	}
	LOG_WRN("Load Bad Sect count: %d", total_bad_sectors);
	return ret;
};




void print_bad_sect_info()
{
	LOG_INF("Load Bad Sect count: %d", total_bad_sectors);
	int bad_sects_tot_length = sizeof(bad_sectors) / sizeof(bad_sectors[0]);
	for (int x = 0; x < bad_sects_tot_length; x++)
	{
		if (bad_sectors[x] != 0){
			LOG_WRN("sect %lu", bad_sectors[x]);
		}
	}
}

// eventually we should just change this to blocks.
int register_bad_sector(uint32_t sector_num){
    if (use_blocks){
        sector_num = convert_page_to_block(sector_num);
        sector_num = convert_block_to_page(0, sector_num);
    }
	if (total_bad_sectors < bad_sector_detect_limit)
	{
		bad_sectors[total_bad_sectors] = sector_num;
		total_bad_sectors++;
		LOG_WRN("New bad sector hit! total bad sectors: %d", sector_num);
		save_bad_sectors_arr();
	}
	else{
		LOG_ERR("Bad sectors hit max allowable bad limit");
	}
	return total_bad_sectors;
}
#else
int register_bad_sector(uint32_t sector_num){return 0;}
int save_bad_sectors_arr(){return 0;}
int load_bad_sectors_arr(){return 0;}
void print_bad_sect_info(){}

#endif

int erase_bad_sectors_arr()
{
	const struct device* soc_flash = FILETABLE_PARTITION_DEVICE;
	off_t address = FILETABLE_PARTITION_OFFSET + (4096*(file_table_sector_num+1));
	int ret = flash_erase(soc_flash, address, sizeof(uint32_t)*bad_sector_detect_limit);
	return ret;

}

int get_sector_offset(int sector_num){
	#ifdef CONFIG_RAW_NAND_BAD_SECTOR_SAVING
	for (int x = 0; x < total_bad_sectors; x++){
		if (bad_sectors[x] <= sector_num){
            if (use_blocks){
                sector_num += 64;
            }
            else{
			sector_num++;
            }
		}
	}
	#endif
	return sector_num;	
}


#define MAX_BAD_PAGES 100
#define PAGE_SIZE 4096

static off_t bad_pages[MAX_BAD_PAGES] = {}; // Global array of bad pages, initialize to 0 (invalid)







/**
 * Wrapper for spi_nand_page_read that simulates bad pages.
 * For pages in the bad_pages array, fills dest with 0xFF (bad output).
 * Otherwise, calls the original spi_nand_page_read.
 */
int spi_nand_page_read_badsim_wrapper(const struct device* dev, off_t page_addr, void* dest) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page_addr && bad_pages[i] != 0) {
            // Bad page: fill with 0xFF
            memset(dest, 0xFF, PAGE_SIZE);
            return 0; // Success, but with bad data
        }
    }
    // Not bad: call original
    return spi_nand_page_read(dev, page_addr, dest);
}

/**
 * Wrapper for spi_nand_page_write that simulates bad pages.
 * For pages in the bad_pages array, returns an error without writing.
 * Otherwise, calls the original spi_nand_page_write.
 */
int spi_nand_page_write_badsim_wrapper(const struct device* dev, off_t page_address, const void* src, size_t size) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page_address && bad_pages[i] != 0) {
            return 0; //  Fail Silently
        }
    }
    // Not bad: call original
    return spi_nand_page_write(dev, page_address, src, size);
}

/**
 * Function to add a bad page to the global array.
 * Returns 0 on success, -1 if array is full.
 */
int add_bad_page(off_t page) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == 0) {
            bad_pages[i] = page;
            return 0;
        }
    }
    return -1; // Array full
}

/**
 * Function to remove a bad page from the global array.
 * Returns 0 on success, -1 if not found.
 */
int remove_bad_page(off_t page) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page) {
            bad_pages[i] = 0;
            return 0;
        }
    }
    return -1; // Not found
}


int spi_nand_bad_page_init(const struct device* dev) {
    
    add_bad_page(126);
    add_bad_page(10895);
    return 0;
}