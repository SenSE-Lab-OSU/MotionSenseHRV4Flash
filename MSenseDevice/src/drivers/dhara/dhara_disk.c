/*
 * Copyright 2026 Devan Mallory
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NAND disk driver backed by the dhara flash translation layer, available through
 * the zephyr disk subsystem. Drop in replacement for nand_disk.c: it exports the
 * same public surface, so the rest of the application links against either one
 * unchanged.
 *
 * The difference is where the translation lives. nand_disk.c keeps a file table on
 * the external NOR and steps over bad blocks with a sector offset table. Dhara keeps
 * a log structured journal on the NAND itself and owns the sector to page mapping,
 * bad block retirement and wear levelling, so none of that is needed here.
 */

#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/storage/disk_access.h>

#include "map.h"
#include "error.h"
#include "msense_dhara.h"
#include "../nand/spi_nand.h"
#include "../nand/bad_page.h"
#include "../nand/nand_disk.h"

#define DT_DRV_COMPAT senselab_nanddisk

LOG_MODULE_REGISTER(dhara_disk, 3);

/* Both disk drivers define the same device instance and the same disk_info, so only
 * one of them can be in a build.
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_DISK_DRIVER_RAW_NAND),
	     "enable only one of DISK_DRIVER_RAW_NAND and DISK_DRIVER_DHARA");

/* A dhara sector is one NAND page. */
#define DHARA_DISK_SECTOR_SIZE	4096

/* Garbage collection operations per write. Lower is faster and more predictable at
 * the cost of usable capacity. It has to stay the same for the life of the chip,
 * because the stored journal was built with it.
 */
#define DHARA_DISK_GC_RATIO	4

static struct dhara_map map;

/* Scratch page dhara uses internally for metadata. */
static uint8_t dhara_map_page_buf[DHARA_DISK_SECTOR_SIZE];

/* File system controls, kept for parity with nand_disk.c. Dhara checks its own
 * writes through the journal, so these are inert here.
 */
bool CheckDuplicateAccess = false;
bool VerifyWrites = false;

/* Dhara stores its mapping on the NAND, so there is no NOR file table to reserve
 * sectors for.
 */
const int file_table_sector_num = 0;

bool read_only = false;

/* Serializes disk access so only one thread drives the shared SPI bus at a time.
 * Dhara is not reentrant either, so this doubles as the lock protecting the map.
 */
K_MUTEX_DEFINE(disk_access_mutex);

/* No NOR file table under dhara. Declared by spi_nand.c for the full erase path. */
int erase_file_table(void)
{
	return 0;
}

void print_flash_status_info(void)
{
	LOG_INF("dhara: %u of %u sectors used, tot ECC corrections %d, tot ECC errors %d",
		dhara_map_size(&map), dhara_map_capacity(&map), ECC_corrections, ECC_err);
}

// TODO: Potentially refactor because we can just mount the filesystem as readonly, instead of doing things this way.
void set_read_only(bool enable)
{
	if (IS_ENABLED(CONFIG_RAW_NAND_ALLOW_RUNTIME_READONLY_FS)){
		LOG_INF("toggling readonly runtime status");
		read_only = enable;
	}
}

bool get_read_only(void)
{
	return read_only;
}

int disk_nand_access_read(struct disk_info *disk, uint8_t *buf, uint32_t sector,
			  uint32_t count)
{
	int ret = 0;

	k_mutex_lock(&disk_access_mutex, K_FOREVER);
	LOG_DBG("performing disk read at sector %u for %u counts", sector, count);

	for (uint32_t x = 0; x < count; x++){
		dhara_error_t err = DHARA_E_NONE;

		// an unmapped sector reads back as a blank page rather than failing
		if (dhara_map_read(&map, sector + x, &buf[x * DHARA_DISK_SECTOR_SIZE],
				   &err) < 0){
			LOG_ERR("dhara read sect %u failed: %s", sector + x,
				dhara_strerror(err));
			ret = -EIO;
			break;
		}
	}

	k_mutex_unlock(&disk_access_mutex);
	return ret;
}

static int disk_dhara_access_write(struct disk_info *disk, const uint8_t *buf,
				   uint32_t sector, uint32_t count)
{
	int ret = 0;

	k_mutex_lock(&disk_access_mutex, K_FOREVER);

	const char *name = k_thread_name_get(k_current_get());
	bool disabled_usb_write = (name != NULL) && (strcmp(name, "usb_mass") == 0) &&
				  !IS_ENABLED(CONFIG_USB_WRITABLE);

	if (read_only || disabled_usb_write){
		LOG_DBG("fs wr req sect %u num %u, but dev read only", sector, count);
		// we fake that we wrote so the USB mass system does not complain
		k_mutex_unlock(&disk_access_mutex);
		return disabled_usb_write ? -2 : -1;
	}

	for (uint32_t x = 0; x < count; x++){
		dhara_error_t err = DHARA_E_NONE;

		if (dhara_map_write(&map, sector + x, &buf[x * DHARA_DISK_SECTOR_SIZE],
				    &err) < 0){
			LOG_ERR("dhara write sect %u failed: %s", sector + x,
				dhara_strerror(err));
			ret = -EIO;
			break;
		}
	}

	k_mutex_unlock(&disk_access_mutex);
	return ret;
}

static int disk_dhara_access_init(struct disk_info *disk)
{
	// the map is brought up in disk_sdmmc_init, before the disk is registered
	return 0;
}

static int disk_dhara_access_status(struct disk_info *disk)
{
	return DISK_STATUS_OK;
}

static int disk_dhara_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	LOG_DBG("Ac ioctl with cmd %d", cmd);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		// what dhara can actually hand out, which is less than the raw page
		// count: the journal holds metadata pages and spare blocks back
		uint32_t sectors = dhara_map_capacity(&map);

		LOG_INF("sect aval: %u", sectors);
		(*(uint32_t *)buf) = sectors;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		(*(uint32_t *)buf) = DHARA_DISK_SECTOR_SIZE;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		// dhara presents a flat remapped sector space and does its own erase
		// block accounting, so there is nothing for the file system to align to
		(*(uint32_t *)buf) = 1;
		break;
	case DISK_IOCTL_CTRL_SYNC:
		{
			dhara_error_t err = DHARA_E_NONE;
			int ret = 0;

			k_mutex_lock(&disk_access_mutex, K_FOREVER);
			if (dhara_map_sync(&map, &err) < 0){
				LOG_ERR("dhara sync failed: %s", dhara_strerror(err));
				ret = -EIO;
			}
			k_mutex_unlock(&disk_access_mutex);
			return ret;
		}
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct disk_operations sdmmc_disk_ops = {
	.init = disk_dhara_access_init,
	.status = disk_dhara_access_status,
	.read = disk_nand_access_read,
	.write = disk_dhara_access_write,
	.ioctl = disk_dhara_access_ioctl,
};

struct disk_info sdmmc_disk = {
	.ops = &sdmmc_disk_ops,
};

#define CONFIG_SPI_FLASH_LAYOUT_PAGE_SIZE 4096

BUILD_ASSERT(DT_INST_NODE_HAS_PROP(0, size),
	     "jedec,spi-nor size required for non-runtime SFDP page layout");

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

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
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8), 0),
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
	/* Note: even though variables use dashes (-) in .yaml and devicetree, DT_INST_PROP requires them in underscores! (_)
	 So, num-flashchips is num_flashchips.
	*/
	.flash_size = DT_INST_PROP(0, individual_size)*DT_INST_PROP(0, num_flashchips),
	.num_flashes = DT_INST_PROP(0, num_flashchips),
	.jedec_id = DT_INST_PROP(0, jedec_id),

#if DT_INST_NODE_HAS_PROP(0, has_lock)
	.has_lock = DT_INST_PROP(0, has_lock),
#endif

#endif

};

static struct spi_nor_data spi_nor_data_0;

static int disk_sdmmc_init(const struct device *dev)
{
	LOG_INF("Init dhara disk regist");

	int status = spi_init(dev);
	if (status != 0){
		LOG_WRN("disk_dhara_init failed %d", status);
	}

	/* Binds the device to the dhara_nand context and fills in the geometry, so it
	 * has to run before the map touches the chip. spi_init() has already restored
	 * the bad block table by this point, which is what dhara_nand_is_bad() reads.
	 */
	dhara_nand *nand = msense_dhara_nand_init(dev);

	dhara_map_init(&map, nand, dhara_map_page_buf, DHARA_DISK_GC_RATIO);

	dhara_error_t err = DHARA_E_NONE;
	if (dhara_map_resume(&map, &err) < 0){
		// nothing valid stored yet, which is the normal first boot case. resume
		// leaves an empty map behind, so the disk is still usable.
		LOG_WRN("no dhara map to resume (%s), starting empty", dhara_strerror(err));
	}

	LOG_INF("dhara map: %u sectors usable, %u in use", dhara_map_capacity(&map),
		dhara_map_size(&map));

	sdmmc_disk.dev = dev;
	sdmmc_disk.name = "SD";
	return disk_access_register(&sdmmc_disk);
}

DEVICE_DT_INST_DEFINE(0,
		&disk_sdmmc_init,
		NULL,
		&spi_nor_data_0,
		&spi_flash_config_0,
		POST_KERNEL,
		80,
		NULL);
