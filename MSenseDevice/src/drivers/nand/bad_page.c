#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>
#ifdef CONFIG_RAW_NAND_BAD_SECTOR_STORAGE_SETTINGS
#include <zephyr/settings/settings.h>
#endif
#include "spi_nand.h"
#include "nand_disk.h"
#include "bad_page.h"


LOG_MODULE_REGISTER(spi_nand_bad_page, CONFIG_FLASH_LOG_LEVEL);


/* The external NOR the table is kept on when the NOR backend is selected. It is the
 * same device and base address the file table in nand_disk.c uses, so the layout
 * below is expressed relative to where that table ends.
 */
#define BAD_SECT_NOR_DEVICE		DEVICE_DT_GET(DT_NODELABEL(mx25u80))
#define BAD_SECT_NOR_BASE		0	/* matches FILETABLE_PARTITION_OFFSET */
#define BAD_SECT_NOR_SECTOR_SIZE	4096	/* one NOR erase sector */

/* Both keys live under the "main" settings subtree, which is what
 * bad_sector_storage_init() loads at boot.
 */
#define BAD_SECT_SUBTREE	"main"
#define BAD_SECT_SCAN_DONE_KEY	BAD_SECT_SUBTREE "/bbscan_done"
#define BAD_SECT_TABLE_KEY	BAD_SECT_SUBTREE "/bbtable"

/* Table size in entries of 4 bytes. Both backends cap this: the settings backend by
 * the NVS per-entry limit of (sector_size - 4 * ate_size), 4064 bytes or 1016
 * entries on a 4096 byte page, and the NOR backend by one erase sector minus the
 * header, which the BUILD_ASSERT below checks.
 */
#define bad_sector_detect_limit 200

#ifdef CONFIG_RAW_NAND_BAD_SECTOR_SAVING



// The current sector offset, caused by the file system having to move data in a different sector due to the prescense of a bad block.
int total_bad_sectors = 0;

// persisted through the settings subsystem so the manufacturer bad-block scan
// only ever runs on the very first boot of a device
bool bad_block_scan_done;

/* This should work for bad blocks.
TODO: Test this system to save and load in offline mode.
Essentially the idea for this is that when we are acessing sectors, we check to see how many bad sectors are below, and that determines the offset to use,
since bad sectors aren't used and the next sector over is used.
*/


bool use_blocks = true;


uint32_t bad_sectors[bad_sector_detect_limit] = {0};

/* The first-boot scan registers every bad block it finds one at a time. Writing
 * the whole table back to NVS on each hit would rewrite the same value hundreds
 * of times, so the scan defers the write and saves once when it finishes.
 */
static bool static_scan_in_progress;


#ifdef CONFIG_RAW_NAND_BAD_SECTOR_STORAGE_NOR

/* NOR backend. Keeps the table in one erase sector of the external NOR, just past
 * the file table, reached through the flash API. Nothing is stored on the internal
 * flash, so Partition Manager lays out exactly the same map as a build without any
 * of this, which is what lets devices already in the field take a DFU update.
 *
 * Layout of the sector: a header carrying the header_name, the one time scan flag and the
 * entry count, followed by that many table entries. An erased sector reads back as
 * all ones, so a missing header_name means nothing has ever been written here.
 */
#define BAD_SECT_NOR_HEADER_NAME		0x42414421u /* spells "BAD!", arbitrary label */

/* The whole table lives in a single erase sector. */
#define BAD_SECT_NOR_REGION_SIZE	BAD_SECT_NOR_SECTOR_SIZE

/* Sectors left unused between the end of the file table and the bad sector table.
 * file_table_access() bounds checks with sector_num > file_table_sector_num, which
 * lets the sector at index file_table_sector_num through, so that sector is treated
 * as part of the file table and skipped here.
 */
#define BAD_SECT_NOR_GUARD_SECTORS	1

struct bad_sect_nor_header {
	uint32_t header_name;
	uint32_t scan_done;
	uint32_t count;
};

BUILD_ASSERT(sizeof(struct bad_sect_nor_header) + sizeof(bad_sectors)
	     <= BAD_SECT_NOR_REGION_SIZE,
	     "bad sector table does not fit in one NOR erase sector");

static inline const struct device *bad_sect_nor_dev(void)
{
	return BAD_SECT_NOR_DEVICE;
}

/* Byte offset just past the last sector of the file table. The file table starts at
 * the base of the NOR and is file_table_sector_num sectors long.
 */
static inline off_t file_table_end_offset(void)
{
	return BAD_SECT_NOR_BASE + (file_table_sector_num * BAD_SECT_NOR_SECTOR_SIZE);
}

/* The bad sector table sits after the file table and its guard sector, matching
 * where this lived originally.
 */
static inline off_t bad_sect_nor_offset(void)
{
	return file_table_end_offset() + (BAD_SECT_NOR_GUARD_SECTORS * BAD_SECT_NOR_SECTOR_SIZE);
}

/* Only the live entries are written, so a device with a handful of bad blocks
 * writes a handful of bytes rather than the whole table. The header goes out in the
 * same pass, so the flag and the table can never disagree.
 */
int save_bad_sectors_arr(){

	if (static_scan_in_progress){
		return 0;
	}

	const struct device *nor = bad_sect_nor_dev();
	off_t addr = bad_sect_nor_offset();
	struct bad_sect_nor_header hdr = {
		.header_name = BAD_SECT_NOR_HEADER_NAME,
		.scan_done = bad_block_scan_done ? 1u : 0u,
		.count = (uint32_t)total_bad_sectors,
	};

	// NOR can only clear bits, so the whole sector has to be erased to rewrite it
	int ret = flash_erase(nor, addr, BAD_SECT_NOR_REGION_SIZE);
	if (ret != 0){
		LOG_ERR("failed to erase bad sector region: %d", ret);
		return ret;
	}

	ret = flash_write(nor, addr, &hdr, sizeof(hdr));
	if (ret != 0){
		LOG_ERR("failed to persist bad sector header: %d", ret);
		return ret;
	}

	if (total_bad_sectors > 0){
		ret = flash_write(nor, addr + sizeof(hdr), bad_sectors,
				  total_bad_sectors * sizeof(bad_sectors[0]));
		if (ret != 0){
			LOG_ERR("failed to persist bad sector table: %d", ret);
		}
	}
	return ret;
};

/* Restores both the table and the scan flag from the header. */
int load_bad_sectors_arr()
{
	const struct device *nor = bad_sect_nor_dev();
	off_t addr = bad_sect_nor_offset();
	struct bad_sect_nor_header hdr;

	int ret = flash_read(nor, addr, &hdr, sizeof(hdr));
	if (ret != 0){
		LOG_ERR("failed to read bad sector header: %d", ret);
		return ret;
	}

	if (hdr.header_name != BAD_SECT_NOR_HEADER_NAME){
		// erased region, so this device has never stored a table
		LOG_INF("no bad sector table on nor yet");
		total_bad_sectors = 0;
		bad_block_scan_done = false;
		return 0;
	}

	if (hdr.count > bad_sector_detect_limit){
		LOG_ERR("stored bad sector count %u out of range", hdr.count);
		total_bad_sectors = 0;
		bad_block_scan_done = false;
		return -EINVAL;
	}

	if (hdr.count > 0){
		ret = flash_read(nor, addr + sizeof(hdr), bad_sectors,
				 hdr.count * sizeof(bad_sectors[0]));
		if (ret != 0){
			LOG_ERR("failed to read bad sector table: %d", ret);
			return ret;
		}
	}

	total_bad_sectors = (int)hdr.count;
	bad_block_scan_done = (hdr.scan_done != 0);
	LOG_WRN("Load Bad Sect count: %d", total_bad_sectors);
	return 0;
};

/* Brings the persisted state back, then runs the manufacturer bad-block scan if it
 * has never run on this device, recording completion in the NOR header.
 */
int bad_sector_storage_init(const struct device *dev)
{
	int rc = load_bad_sectors_arr();
	if (rc != 0) {
		LOG_WRN("bad sector storage unavailable (%d), skipping bad block scan", rc);
		return rc;
	}

	if (bad_block_scan_done) {
		LOG_INF("manufacturer bad block scan already done, skipping");
		print_bad_sect_info();
		return 0;
	}

	LOG_INF("first boot: scanning for manufacturer bad blocks");
	static_scan_in_progress = true;
	detect_manufacturer_bad_blocks(dev);
	static_scan_in_progress = false;

	// the flag rides along in the same header as the table, so one write does both
	bad_block_scan_done = true;
	rc = save_bad_sectors_arr();
	if (rc != 0) {
		LOG_WRN("failed to persist bad sector table: %d", rc);
	}
	return rc;
}

int erase_bad_sectors_arr()
{
	memset(bad_sectors, 0, sizeof(bad_sectors));
	total_bad_sectors = 0;
	// dropping the flag too means the next boot rescans the freshly erased chips
	bad_block_scan_done = false;

	int ret = flash_erase(bad_sect_nor_dev(), bad_sect_nor_offset(),
			      BAD_SECT_NOR_REGION_SIZE);
	if (ret != 0){
		LOG_ERR("fail to erase bad sect region: %d", ret);
	}
	return ret;
}

#else /* CONFIG_RAW_NAND_BAD_SECTOR_STORAGE_SETTINGS */

/* Only the live entries are persisted, so a device with a handful of bad blocks
 * writes a handful of bytes rather than the whole table.
 */
int save_bad_sectors_arr(){

	if (static_scan_in_progress){
		return 0;
	}

	int ret = settings_save_one(BAD_SECT_TABLE_KEY, bad_sectors,
				    total_bad_sectors * sizeof(bad_sectors[0]));
	if (ret != 0){
		LOG_ERR("failed to persist bad sector table: %d", ret);
	}
	return ret;
};

/* The table itself is restored by bad_sector_settings_set(), which the settings
 * subsystem drives from inside this call. It initializes both the table and the bool.
 */
int load_bad_sectors_arr()
{
	int ret = settings_load_subtree(BAD_SECT_SUBTREE);
	if (ret != 0){
		LOG_ERR("failed to load bad sector table: %d", ret);
		return ret;
	}
	LOG_WRN("Load Bad Sect count: %d", total_bad_sectors);
	return ret;
};
/* This function loads both the bad_block_scan and bad block table variables.
 it's an implicit function that's done by the settings subsystem */
static int bad_sector_settings_set(const char *name, size_t len,
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
		if ((len % sizeof(bad_sectors[0])) != 0 || len > sizeof(bad_sectors)) {
			LOG_ERR("stored bad sector table has bad length %u", (unsigned int)len);
			return -EINVAL;
		}
		rc = read_cb(cb_arg, bad_sectors, len);
		if (rc < 0) {
			return rc;
		}
		total_bad_sectors = rc / sizeof(bad_sectors[0]);
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(nand_bad_page, BAD_SECT_SUBTREE, NULL,
			       bad_sector_settings_set, NULL, NULL);

/* Brings the persisted state back, then runs the manufacturer bad-block scan if
 * it has never run on this device, remembering completion in the settings
 * subsystem (NVS on internal flash).
 */
int bad_sector_storage_init(const struct device *dev)
{
	int rc = settings_subsys_init();
	if (rc == 0) {
		rc = load_bad_sectors_arr();
	}
	if (rc != 0) {
		LOG_WRN("settings unavailable (%d), skipping bad block scan", rc);
		return rc;
	}

	if (bad_block_scan_done) {
		LOG_INF("manufacturer bad block scan already done, skipping");
		print_bad_sect_info();
		return 0;
	}

	LOG_INF("first boot: scanning for manufacturer bad blocks");
	static_scan_in_progress = true;
	detect_manufacturer_bad_blocks(dev);
	static_scan_in_progress = false;

	rc = save_bad_sectors_arr();
	if (rc != 0) {
		return rc;
	}

	bad_block_scan_done = true;
	rc = settings_save_one(BAD_SECT_SCAN_DONE_KEY, &bad_block_scan_done,
			       sizeof(bad_block_scan_done));
	if (rc != 0) {
		LOG_WRN("failed to persist bad block scan flag: %d", rc);
	}
	return rc;
}

int erase_bad_sectors_arr()
{
	memset(bad_sectors, 0, sizeof(bad_sectors));
	total_bad_sectors = 0;
	bad_block_scan_done = false;

	int ret = settings_delete(BAD_SECT_TABLE_KEY);
	if (ret != 0){
		LOG_ERR("fail to delete bad sect table: %d", ret);
		return ret;
	}
	// dropping the flag too means the next boot rescans the freshly erased chips
	ret = settings_delete(BAD_SECT_SCAN_DONE_KEY);
	if (ret != 0){
		LOG_ERR("fail to delete bad sect scan flag: %d", ret);
	}
	return ret;
}

#endif /* CONFIG_RAW_NAND_BAD_SECTOR_STORAGE_NOR */

void print_bad_sect_info()
{
	LOG_INF("Load Bad Sect count: %d", total_bad_sectors);
	for (int x = 0; x < total_bad_sectors; x++)
	{
		LOG_WRN("sect %lu", bad_sectors[x]);
	}
}

/* Records a bad sector unconditionally. register_bad_sector() gates this behind the
 * first boot scan, but a caller with authoritative knowledge (dhara_nand_mark_bad)
 * has to be able to record a block whether or not that scan has already run.
 */
// eventually we should just change this to blocks.
int mark_bad_sector(uint32_t sector_num){
    if (use_blocks){
        sector_num = convert_page_to_block(sector_num);
        sector_num = convert_block_to_page(0, sector_num);
    }
	if (total_bad_sectors >= bad_sector_detect_limit)
	{
		LOG_ERR("Bad sectors hit max allowable bad limit");
		return total_bad_sectors;
	}

	/* get_sector_offset() walks the table in order and skips a whole block per
	 * entry, so it has to stay sorted ascending and free of duplicates. With
	 * use_blocks any two failing pages in the same block floor to the same value,
	 * so the duplicate check is what stops one bad block from costing two.
	 */
	int pos = 0;
	while (pos < total_bad_sectors && bad_sectors[pos] < sector_num)
	{
		pos++;
	}
	if (pos < total_bad_sectors && bad_sectors[pos] == sector_num)
	{
		LOG_DBG("sect %u already registered, ignoring", sector_num);
		return total_bad_sectors;
	}

	memmove(&bad_sectors[pos + 1], &bad_sectors[pos],
		(total_bad_sectors - pos) * sizeof(bad_sectors[0]));
	bad_sectors[pos] = sector_num;
	total_bad_sectors++;
	LOG_WRN("New bad sector hit! sect %u, total bad sectors: %d", sector_num,
		total_bad_sectors);
	save_bad_sectors_arr();
	return total_bad_sectors;
}

int register_bad_sector(uint32_t sector_num){
	// after the first boot scan the table is treated as the fixed factory bad
	// block list, unless runtime registration is explicitly enabled
	if (!bad_block_scan_done || IS_ENABLED(CONFIG_BAD_SECTOR_SAVING_RUNTIME)){
		return mark_bad_sector(sector_num);
	}
	return total_bad_sectors;
}

/* True if the block holding this sector is in the bad table. register_bad_sector()
 * keeps the table sorted ascending and duplicate free, so this is a binary search.
 */
bool is_sector_bad(uint32_t sector_num){
	if (use_blocks){
		sector_num = convert_block_to_page(0, convert_page_to_block(sector_num));
	}
	int lo = 0;
	int hi = total_bad_sectors - 1;
	while (lo <= hi){
		int mid = lo + ((hi - lo) / 2);
		if (bad_sectors[mid] == sector_num){
			return true;
		}
		if (bad_sectors[mid] < sector_num){
			lo = mid + 1;
		}
		else{
			hi = mid - 1;
		}
	}
	return false;
}

/* Maps a logical sector onto the physical one by stepping over every bad entry at
 * or below it. Relies on register_bad_sector() keeping the table sorted ascending:
 * sector_num only ever grows, so the first entry above it means no later entry can
 * match either and the walk can stop there.
 */
int get_sector_offset(int sector_num){
	for (int x = 0; x < total_bad_sectors; x++){
		if (bad_sectors[x] > (uint32_t)sector_num){
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
int total_bad_sectors = 0;
bool bad_block_scan_done;
int bad_sector_storage_init(const struct device *dev){return 0;}
int register_bad_sector(uint32_t sector_num){return 0;}
int mark_bad_sector(uint32_t sector_num){return 0;}
bool is_sector_bad(uint32_t sector_num){return false;}
int save_bad_sectors_arr(){return 0;}
int load_bad_sectors_arr(){return 0;}
int erase_bad_sectors_arr(){return 0;}
void print_bad_sect_info(){}
int get_sector_offset(int sector_num){return sector_num;}

#endif


#define MAX_BAD_PAGES 100
#define PAGE_SIZE 4096

// Global array of simulated bad pages, as physical page numbers across all 4 flashes.
// 0 marks an empty slot, so page 0 itself cannot be simulated bad.
static uint32_t bad_pages[MAX_BAD_PAGES] = {};

static bool is_simulated_bad_page(uint32_t page_number) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page_number && bad_pages[i] != 0) {
            return true;
        }
    }
    return false;
}

/**
 * Wrapper for multi_nand_page_read that simulates bad pages.
 * For pages in the bad_pages array, behaves like a real uncorrectable read: the
 * buffer is filled with 0xFF, the block is registered bad the same way
 * multi_nand_page_read does, and FLASH_TOO_MANY_ECC_ERROR is returned.
 * Otherwise, calls multi_nand_page_read.
 */
int multi_nand_page_read_badsim_wrapper(const struct device* dev, uint32_t page_number, void* buffer) {
    if (is_simulated_bad_page(page_number)) {
        memset(buffer, 0xFF, PAGE_SIZE);
        LOG_WRN("simulated uncorrectable ECC error at sect %u", page_number);
        register_bad_sector(page_number);
        return FLASH_TOO_MANY_ECC_ERROR;
    }
    return multi_nand_page_read(dev, page_number, buffer);
}

/**
 * Wrapper for multi_nand_page_write that simulates bad pages.
 * For pages in the bad_pages array, nothing is written and the failure is
 * reported the way multi_nand_page_write reports a real one: the block is
 * registered bad and a non-zero error is returned.
 * Otherwise, calls multi_nand_page_write.
 */
int multi_nand_page_write_badsim_wrapper(const struct device* dev, uint32_t page_number, const void* buffer, size_t size) {
    if (is_simulated_bad_page(page_number)) {
        LOG_WRN("simulated program fail at sect %u", page_number);
        register_bad_sector(page_number);
        return FLASH_PROGRAM_FAILURE;
    }
    return multi_nand_page_write(dev, page_number, buffer, size);
}

/**
 * Function to add a bad page to the global array.
 * Returns 0 on success, -1 if array is full.
 */
int add_bad_page(uint32_t page) {
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
int remove_bad_page(uint32_t page) {
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