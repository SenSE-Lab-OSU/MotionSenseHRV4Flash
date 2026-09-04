#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include "spi_nand.h"
#include "nand_disk.h"
#include "bad_block.h"


LOG_MODULE_REGISTER(spi_nand_bad_block, CONFIG_FLASH_LOG_LEVEL);


/* Both keys live under the "main" settings subtree, which is what
 * bad_block_storage_init() loads at boot.
 */
#define BAD_BLOCK_SUBTREE	"main"
#define BAD_BLOCK_SCAN_DONE_KEY	BAD_BLOCK_SUBTREE "/bbscan_done"
#define BAD_BLOCK_TABLE_KEY	BAD_BLOCK_SUBTREE "/bbtable"

#ifdef CONFIG_RAW_NAND_BAD_BLOCK_SAVING



// The current sector offset, caused by the file system having to move data in a different sector due to the prescense of a bad block.
int total_bad_blocks = 0;

// persisted through the settings subsystem so the manufacturer bad-block scan
// only ever runs on the very first boot of a device
bool bad_block_scan_done;

/* This should work for bad blocks.
TODO: Test this system to save and load in offline mode.
Essentially the idea for this is that when we are acessing sectors, we check to see how many bad blocks are below, and that determines the offset to use,
since bad blocks aren't used and the next sector over is used.
*/


bool use_blocks = true;


uint32_t bad_blocks[bad_block_detect_limit] = {0};

/* The first-boot scan registers every bad block it finds one at a time. Writing
 * the whole table back to NVS on each hit would rewrite the same value hundreds
 * of times, so the scan defers the write and saves once when it finishes.
 */
static bool static_scan_in_progress = false;


/* Only the live entries are persisted, so a device with a handful of bad blocks
 * writes a handful of bytes rather than the whole table.
 */
int save_bad_blocks_arr(){

	if (static_scan_in_progress){
		return 0;
	}

	int ret = settings_save_one(BAD_BLOCK_TABLE_KEY, bad_blocks,
				    total_bad_blocks * sizeof(bad_blocks[0]));
	if (ret != 0){
		LOG_ERR("failed to persist bad block table: %d", ret);
	}
	return ret;
};

/* The table itself is restored by bad_block_settings_set(), which the settings
 * subsystem drives from inside this call. It initializes both the table and the bool.
 */
int load_bad_blocks_arr()
{
	int ret = settings_load_subtree(BAD_BLOCK_SUBTREE);
	if (ret != 0){
		LOG_ERR("failed to load bad block table: %d", ret);
		return ret;
	}
	LOG_WRN("Load Bad Block count: %d", total_bad_blocks);
	return ret;
};
/* This function loads both the bad_block_scan and bad block table variables.
 it's an implicit function that's done by the settings subsystem */
static int bad_block_settings_set(const char *name, size_t len,
				   settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	int rc;

	if (settings_name_steq(name, "bbscan_done", &next) && !next) {
		if (len != sizeof(bad_block_scan_done)) {
			return -EINVAL;
		}
		rc = read_cb(cb_arg, &bad_block_scan_done, sizeof(bad_block_scan_done));
		return (rc >= 0) ? 0 : rc;
	}

	if (settings_name_steq(name, "bbtable", &next) && !next) {
		if ((len % sizeof(bad_blocks[0])) != 0 || len > sizeof(bad_blocks)) {
			LOG_ERR("stored bad block table has bad length %u", (unsigned int)len);
			return -EINVAL;
		}
		rc = read_cb(cb_arg, bad_blocks, len);
		if (rc < 0) {
			return rc;
		}
		total_bad_blocks = rc / sizeof(bad_blocks[0]);
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(nand_bad_block, BAD_BLOCK_SUBTREE, NULL,
			       bad_block_settings_set, NULL, NULL);

/* Brings the persisted state back, then runs the manufacturer bad-block scan if
 * it has never run on this device, remembering completion in the settings
 * subsystem (NVS on internal flash).
 */
int bad_block_storage_init(const struct device *dev)
{
	int rc = settings_subsys_init();
	if (rc == 0) {
		rc = load_bad_blocks_arr();
	}
	if (rc != 0) {
		LOG_WRN("settings unavailable (%d), skipping bad block scan", rc);
		return rc;
	}

	if (bad_block_scan_done) {
		LOG_INF("manufacturer bad block scan already done, skipping");
		print_bad_block_info();
		return 0;
	}

	LOG_INF("first boot: scanning for manufacturer bad blocks");
	static_scan_in_progress = true;
	detect_manufacturer_bad_blocks(dev);
	static_scan_in_progress = false;

	rc = save_bad_blocks_arr();
	if (rc != 0) {
		return rc;
	}

	bad_block_scan_done = true;
	rc = settings_save_one(BAD_BLOCK_SCAN_DONE_KEY, &bad_block_scan_done,
			       sizeof(bad_block_scan_done));
	if (rc != 0) {
		LOG_WRN("failed to persist bad block scan flag: %d", rc);
	}
	return rc;
}

int erase_bad_blocks_arr()
{
	memset(bad_blocks, 0, sizeof(bad_blocks));
	total_bad_blocks = 0;
	bad_block_scan_done = false;

	int ret = settings_delete(BAD_BLOCK_TABLE_KEY);
	if (ret != 0){
		LOG_ERR("fail to delete bad sect table: %d", ret);
		return ret;
	}
	// dropping the flag too means the next boot rescans the freshly erased chips
	ret = settings_delete(BAD_BLOCK_SCAN_DONE_KEY);
	if (ret != 0){
		LOG_ERR("fail to delete bad sect scan flag: %d", ret);
	}
	return ret;
}

/* Drops the scan done flag and keeps the table, so a later boot can see something
 * went wrong. The flag has its own key here, separate from the table.
 */
int clear_bad_block_scan_done()
{
	if (!bad_block_scan_done){
		return 0;
	}
	bad_block_scan_done = false;
	LOG_WRN("clearing bad block scan done flag");
	int ret = settings_save_one(BAD_BLOCK_SCAN_DONE_KEY, &bad_block_scan_done,
				    sizeof(bad_block_scan_done));
	if (ret != 0){
		LOG_ERR("fail to clear bad block scan flag: %d", ret);
	}
	return ret;
}

void print_bad_block_info()
{
	LOG_INF("Load Bad Block count: %d", total_bad_blocks);
	LOG_INF("Current bad blocks:");
	for (int x = 0; x < total_bad_blocks; x++)
	{
		LOG_INF("sect %lu", bad_blocks[x]);
	}
}

/* Records a bad block unconditionally. register_bad_block() gates this behind the
 * first boot scan, but a caller with authoritative knowledge (dhara_nand_mark_bad)
 * has to be able to record a block whether or not that scan has already run.
 */
// eventually we should just change this to blocks.
int mark_bad_block(uint32_t sector_num){
    if (use_blocks){
        sector_num = convert_page_to_block(sector_num);
        sector_num = convert_block_to_page(0, sector_num);
    }
	if (total_bad_blocks >= bad_block_detect_limit)
	{
		LOG_ERR("Bad sectors hit max allowable bad limit");
		return total_bad_blocks;
	}

	/* get_sector_offset() walks the table in order and skips a whole block per
	 * entry, so it has to stay sorted ascending and free of duplicates. With
	 * use_blocks any two failing pages in the same block floor to the same value,
	 * so the duplicate check is what stops one bad block from costing two.
	 */
	int pos = 0;
	while (pos < total_bad_blocks && bad_blocks[pos] < sector_num)
	{
		pos++;
	}
	if (pos < total_bad_blocks && bad_blocks[pos] == sector_num)
	{
		LOG_DBG("sect %u already registered, ignoring", sector_num);
		return total_bad_blocks;
	}

	memmove(&bad_blocks[pos + 1], &bad_blocks[pos],
		(total_bad_blocks - pos) * sizeof(bad_blocks[0]));
	bad_blocks[pos] = sector_num;
	total_bad_blocks++;
	LOG_WRN("New bad block hit! sect %u, total bad blocks: %d", sector_num,
		total_bad_blocks);
	save_bad_blocks_arr();
	return total_bad_blocks;
}

int register_bad_block(uint32_t sector_num){
	// after the first boot scan the table is treated as the fixed factory bad
	// block list, unless runtime registration is explicitly enabled
	if (static_scan_in_progress || IS_ENABLED(CONFIG_BAD_BLOCK_SAVING_RUNTIME)){
		LOG_INF("registering bad block sect %u", sector_num);
		return mark_bad_block(sector_num);
	}
	return total_bad_blocks;
}

/* True if the block holding this sector is in the bad table. register_bad_block()
 * keeps the table sorted ascending and duplicate free, so this is a binary search.
 */
bool is_block_bad(uint32_t sector_num){
	if (use_blocks){
		sector_num = convert_block_to_page(0, convert_page_to_block(sector_num));
	}
	int lo = 0;
	int hi = total_bad_blocks - 1;
	while (lo <= hi){
		int mid = lo + ((hi - lo) / 2);
		if (bad_blocks[mid] == sector_num){
			return true;
		}
		if (bad_blocks[mid] < sector_num){
			lo = mid + 1;
		}
		else{
			hi = mid - 1;
		}
	}
	return false;
}

/* Maps a logical sector onto the physical one by stepping over every bad entry at
 * or below it. Relies on register_bad_block() keeping the table sorted ascending:
 * sector_num only ever grows, so the first entry above it means no later entry can
 * match either and the walk can stop there.
 */
int get_sector_offset(int sector_num){
	// the file table lives on the NOR, so a bad NAND block under it must not move it
	if (sector_num < file_table_sector_num){
		return sector_num;
	}
	for (int x = 0; x < total_bad_blocks; x++){
		if (bad_blocks[x] > (uint32_t)sector_num){
			break;
		}
		if (use_blocks){
			sector_num += NAND_PAGES_PER_ERASE_BLOCK;
		}
		else{
			sector_num++;
		}
	}
	return sector_num;
}

#else
int total_bad_blocks = 0;
bool bad_block_scan_done;
int bad_block_storage_init(const struct device *dev){return 0;}
int register_bad_block(uint32_t sector_num){return 0;}
int mark_bad_block(uint32_t sector_num){return 0;}
bool is_block_bad(uint32_t sector_num){return false;}
int save_bad_blocks_arr(){return 0;}
int load_bad_blocks_arr(){return 0;}
int erase_bad_blocks_arr(){return 0;}
int clear_bad_block_scan_done(){return 0;}
void print_bad_block_info(){}
int get_sector_offset(int sector_num){return sector_num;}

#endif


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