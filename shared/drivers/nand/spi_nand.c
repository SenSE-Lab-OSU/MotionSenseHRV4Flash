/*
* Copyright (c) 2018 Savoir-Faire Linux.
* Copyright (c) 2020 Peter Bigot Consulting, LLC
* Copyright (c) 2025 SENSE Lab Ohio State
* 
* This driver is heavily inspired from the spi_flash_w25qxxdv.c SPI NOR driver.
*
* SPDX-License-Identifier: Apache-2.0
*/


#define CONFIG_NORDIC_QSPI_NOR_STACK_WRITE_BUFFER_SIZE 4
#define DT_DRV_COMPAT senselab_nanddisk


#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/pm/device.h>

#include "spi_nand.h"
#include "flash_priv.h"
#include "bad_page.h"

int erase_file_table();


LOG_MODULE_REGISTER(spi_nand, CONFIG_FLASH_LOG_LEVEL);

#define NAND_STATUS_POLL_INTERVAL_US 100U
#define NAND_PAGE_READ_TIMEOUT_US 250U
#define NAND_PAGE_PROGRAM_TIMEOUT_US 750U
#define NAND_BLOCK_ERASE_TIMEOUT_US 12000U
#define NAND_RESET_NO_COMMAND_US 1500U
#define NAND_RESET_POLL_TIMEOUT_US 1000U
#define NAND_DIE_UNKNOWN (-1)

#define NAND_STATUS_ERASE_FAIL BIT(2)
#define NAND_STATUS_PROGRAM_FAIL BIT(3)
/* Micron parameter page: at most 40 bad blocks per logical unit (die). */
#define NAND_FORMAT_MAX_SKIPPED_BLOCKS_PER_DIE 40U

K_MUTEX_DEFINE(storage_spi_bus_mutex);

void storage_spi_bus_lock(void)
{
	k_mutex_lock(&storage_spi_bus_mutex, K_FOREVER);
}

void storage_spi_bus_unlock(void)
{
	k_mutex_unlock(&storage_spi_bus_mutex);
}

/* Device Power Management Notes
*
* These flash devices have several modes during operation:
* * When CSn is asserted (during a SPI operation) the device is
*   active.
* * When CSn is deasserted the device enters a standby mode.
* * Some devices support a Deep Power-Down mode which reduces current
*   to as little as 0.1% of standby.
*
* The power reduction from DPD is sufficient to warrant allowing its
* use even in cases where Zephyr's device power management is not
* available.  This is selected through the SPI_NOR_IDLE_IN_DPD
* Kconfig option.
*
* When mapped to the Zephyr Device Power Management states:
* * PM_DEVICE_STATE_ACTIVE covers both active and standby modes;
* * PM_DEVICE_STATE_SUSPENDED, and PM_DEVICE_STATE_OFF all correspond to
*   deep-power-down mode.
*/

#define SPI_NOR_MAX_ADDR_WIDTH 4

#if DT_INST_NODE_HAS_PROP(0, t_enter_dpd)
#define T_DP_MS DIV_ROUND_UP(DT_INST_PROP(0, t_enter_dpd), NSEC_PER_MSEC)
#else /* T_ENTER_DPD */
#define T_DP_MS 0
#endif /* T_ENTER_DPD */
#if DT_INST_NODE_HAS_PROP(0, t_exit_dpd)
#define T_RES1_MS DIV_ROUND_UP(DT_INST_PROP(0, t_exit_dpd), NSEC_PER_MSEC)
#endif /* T_EXIT_DPD */
#if DT_INST_NODE_HAS_PROP(0, dpd_wakeup_sequence)
#define T_DPDD_MS DIV_ROUND_UP(DT_INST_PROP_BY_IDX(0, dpd_wakeup_sequence, 0), NSEC_PER_MSEC)
#define T_CRDP_MS DIV_ROUND_UP(DT_INST_PROP_BY_IDX(0, dpd_wakeup_sequence, 1), NSEC_PER_MSEC)
#define T_RDP_MS DIV_ROUND_UP(DT_INST_PROP_BY_IDX(0, dpd_wakeup_sequence, 2), NSEC_PER_MSEC)
#else /* DPD_WAKEUP_SEQUENCE */
#define T_DPDD_MS 0
#endif /* DPD_WAKEUP_SEQUENCE */

#define SPI_BUS_NODE 
//#define SPI_BUS_NODE DT_NODELABEL(spi4)


int current_writes = 0;
int current_reads = 0;
int current_erases = 0;

int ECC_corrections = 0;
int ECC_err = 0;


/* The board DTS is the single source for the NAND population. */
#define NAND_FLASH_COUNT DT_INST_PROP(0, num_flashchips)
BUILD_ASSERT(NAND_FLASH_COUNT > 0, "NAND needs at least one package");

// die select for each flash; initialized before the packages are configured
int current_die[NAND_FLASH_COUNT];


// parameter for multiple flashes.
int current_flash = 0;

static int get_features(const struct device *dev, uint8_t register_select,
			uint8_t *data);
static int set_features(const struct device *dev, uint8_t register_select,
			uint8_t data);
static int set_die(const struct device *dev, int die_select);
static int set_flash(const struct device *dev, int flash_id);

/* Get the size of the flash device.  Data for runtime, constant for
* minimal and devicetree.
*/
uint32_t dev_flash_size(const struct device *dev)
{

	const struct spi_flash_config* cfg = dev->config;

	return cfg->flash_size;

}

static inline int dev_die_size(const struct device* dev){
	const struct spi_flash_config* cfg = dev->config;
	return dev_flash_size(dev) / cfg->dies_per_flash;
}

/* Get the flash device page size.  Constant for minimal, data for
* runtime and devicetree.
*/
uint16_t dev_page_size(const struct device *dev)
{
	const struct spi_flash_config* cfg = dev->config;
	return cfg->page_size;
}

uint32_t dev_pages_per_erase_block(const struct device *dev)
{
	const struct spi_flash_config* cfg = dev->config;
	return cfg->pages_per_erase_block;
}

uint32_t dev_total_sector_count(const struct device *dev)
{
	const struct spi_flash_config* cfg = dev->config;
	uint64_t total_bytes = (uint64_t)cfg->flash_size * cfg->num_flashes;

	return (uint32_t)(total_bytes / cfg->page_size);
}

static const struct flash_parameters flash_nor_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

/* Capture the time at which the device entered deep power-down. */
static inline void record_entered_dpd(const struct device *const dev)
{
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	struct spi_nor_data *const driver_data = dev->data;

	driver_data->ts_enter_dpd = k_uptime_get_32();
#endif
}

/* Check the current time against the time DPD was entered and delay
* until it's ok to initiate the DPD exit process.
*/
static inline void delay_until_exit_dpd_ok(const struct device *const dev)
{
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	struct spi_nor_data *const driver_data = dev->data;
	int32_t since = (int32_t)(k_uptime_get_32() - driver_data->ts_enter_dpd);

	/* If the time is negative the 32-bit counter has wrapped,
	* which is certainly long enough no further delay is
	* required.  Otherwise we have to check whether it's been
	* long enough taking into account necessary delays for
	* entering and exiting DPD.
	*/
	if (since >= 0) {
		/* Subtract time required for DPD to be reached */
		since -= T_DP_MS;

		/* Subtract time required in DPD before exit */
		since -= T_DPDD_MS;

		/* If the adjusted time is negative we have to wait
		* until it reaches zero before we can proceed.
		*/
		if (since < 0) {
			k_sleep(K_MSEC((uint32_t)-since));
		}
	}
#endif /* DT_INST_NODE_HAS_PROP(0, has_dpd) */
}



uint32_t convert_block_to_page(uint32_t page, uint32_t block){
	return page + (block * NAND_PAGES_PER_ERASE_BLOCK);
}

// The pages representing a block are from block - 65.
// 4 gigabit is 536870912 bytes / 4096 = 131072 pages (131071 is last address)
off_t convert_page_to_address(const struct device* dev, uint32_t page) {
	const struct spi_flash_config *cfg = dev->config;
	uint32_t die_size = dev_die_size(dev) / dev_page_size(dev);
	uint32_t selected_die_num = page / die_size;
	uint32_t flash = selected_die_num / cfg->dies_per_flash;
	uint32_t die = selected_die_num % cfg->dies_per_flash;

	if ((page >= dev_total_sector_count(dev)) || (flash >= cfg->num_flashes)) {
		return -EINVAL;
	}
	LOG_DBG("die/flash  number: %d", selected_die_num);

	if ((set_flash(dev, flash) != 0) || (set_die(dev, die) != 0)) {
		return -EIO;
	}

	return page - (die_size * selected_die_num);
}

// this is not the actual address for 4 flash, 
off_t convert_block_to_singledie_address(uint32_t block){
	//werid fix because of noticed offsets, perhaps there is another issue we are unaware of.
	return (block * NAND_PAGES_PER_ERASE_BLOCK);
}

uint32_t convert_page_to_block(uint32_t page_number){
	return (page_number / NAND_PAGES_PER_ERASE_BLOCK);
}

bool is_page_in_block(uint32_t page_number, uint32_t block_number){
	uint32_t first_page = convert_block_to_page(0, block_number);
	uint32_t difference = (page_number - first_page);
	return difference >= 0 && difference < NAND_PAGES_PER_ERASE_BLOCK;
}


static void acquire_device_inner(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nor_data *const driver_data = dev->data;

		k_sem_take(&driver_data->sem_inner, K_FOREVER);
	}
}

static void release_device_inner(const struct device *dev)
{

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nor_data* const driver_data = dev->data;

		k_sem_give(&driver_data->sem_inner);
	}
}


/*
* @brief Send an SPI command
*
* @param dev Device struct
* @param opcode The command to send
* @param access flags that determine how the command is constructed.
*        See NOR_ACCESS_*.
* @param addr The address to send
* @param data The buffer to store or read the value
* @param length The size of the buffer
* @return 0 on success, negative errno code otherwise
*/
static int spi_nand_access(const struct device *const dev, spi_send_request* request)
{
	acquire_device_inner(dev);
	const struct spi_flash_config* const driver_cfg = dev->config;
	int ret;

	if ((current_flash < 0) || (current_flash >= driver_cfg->num_flashes)) {
		release_device_inner(dev);
		return -EINVAL;
	}

	// get parameters needed for spi_transceive and spi_write
	struct spi_dt_spec spi_spec = driver_cfg->spi;
	struct spi_config spi_flash_cfg = spi_spec.config;
	spi_flash_cfg.cs.gpio = driver_cfg->chip_selects[current_flash];
	
	uint8_t buf[5] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = 1,
		},
		{
			.buf = request->data,
			.len = request->data_length
		}
	};

	buf[0] = request->opcode;
	if (request->addr_length > 0) {
		memcpy(&buf[1], request->addr, request->addr_length);
		spi_buf[0].len += request->addr_length;
	};

	const struct spi_buf_set tx_set = {
		.buffers = spi_buf,
		.count = (request->data_length > 0) ? 2 : 1,
	};

	const struct spi_buf_set rx_set = {
		.buffers = spi_buf,
		.count = 2,
	};

	// Perform Desired Spi Operation
	if (request->is_write) {
		ret = spi_write(driver_cfg->spi.bus, &spi_flash_cfg, &tx_set);
		
	}
	else {
		ret = spi_transceive(driver_cfg->spi.bus, &spi_flash_cfg, &tx_set, &rx_set);
	}

	release_device_inner(dev);
	return ret;
}

static int spi_cmd(const struct device* dev, uint8_t opcode, void* dest, size_t length){
	spi_send_request request = {
		.opcode = opcode,
		.data = dest,
		.data_length = length
	};
	return spi_nand_access(dev, &request); 
}

static int get_status(const struct device *dev, uint8_t *status)
{
	return get_features(dev, REGISTER_STATUS, status);
}

static int write_enable(const struct device* dev){
	// First, enable write acess if needed
	
		int ret = spi_cmd(dev, SPI_NOR_CMD_WREN, NULL, 0);
		if (ret != 0){
			LOG_WRN("write enable failed");
		}
		return ret;
}
	
		

static int write_disable(const struct device* dev){
	
	int ret = spi_cmd(dev, SPI_NOR_CMD_WRDI, NULL, 0);
	if (ret != 0){
			LOG_WRN("write disable fail");
	}
	return ret;
}


static int spi_nand_reset(const struct device *dev)
{
	return spi_cmd(dev, SPI_NAND_RESET, NULL, 0);
}

static int set_die(const struct device* dev, int die_select){
	const struct spi_flash_config *cfg = dev->config;
	if ((current_flash < 0) || (current_flash >= cfg->num_flashes) ||
	    (die_select < 0) || (die_select >= cfg->dies_per_flash)) {
		return -EINVAL;
	}
	if (current_die[current_flash] == die_select) {
		return 0;
	}
	current_die[current_flash] = NAND_DIE_UNKNOWN;

	uint8_t feature = 0x0;
	if (die_select == 1) {
		feature = 0x40;
	}
	int ret = set_features(dev, REGISTER_DIESELECT, feature);
	if (ret == 0){
		current_die[current_flash] = die_select;
		LOG_DBG("selected NAND package %d die %d", current_flash,
			current_die[current_flash]);
	}
	else{
		LOG_WRN("error die setting %d", ret);
	}
	return ret;

}




static int set_flash(const struct device* dev, int flash_id){
	const struct spi_flash_config *cfg = dev->config;
	if ((flash_id < 0) || (flash_id >= cfg->num_flashes)) {
		return -EINVAL;
	}
	current_flash = flash_id;
	return 0;
}


static int get_features(const struct device *dev, uint8_t register_select,
			uint8_t *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	spi_send_request request = {
		.opcode = SPI_NAND_GF,
		.addr = &register_select,
		.addr_length = 1,
		.data = data,
		.data_length = 1,
	};

	int ret = spi_nand_access(dev, &request);
	if (ret != 0) {
		LOG_WRN("get features failed: %d", ret);
	}
	return ret;
}

static int set_features(const struct device* dev, uint8_t register_select,
			uint8_t data){
	uint8_t readback;

	spi_send_request write_features_request = {
		.opcode = 0x1F,
		.is_write = true,
		.addr = &register_select,
		.addr_length = 1,
		.data = &data,
		.data_length = 1
	};

	int ret = spi_nand_access(dev, &write_features_request);
	if (ret != 0) {
		return ret;
	}
	ret = get_features(dev, register_select, &readback);
	if (ret != 0) {
		return ret;
	}
	return readback == data ? 0 : -EIO;
}



/**
 * @brief Wait until the flash is ready
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * This function should be invoked after every ERASE, PROGRAM, or
 * WRITE_STATUS operation before continuing.  This allows us to assume
 * that the device is ready to accept new commands at any other point
 * in the code.
 *
 * @param dev The device structure
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_wait_until_ready(const struct device *dev,
				     uint32_t timeout_us,
				     uint8_t *final_status)
{
	k_timepoint_t deadline;
	int ret;

	if (final_status == NULL) {
		return -EINVAL;
	}
	deadline = sys_timepoint_calc(K_USEC(timeout_us));

	while (true) {
		ret = get_status(dev, final_status);
		if (ret != 0) {
			return ret;
		}
		if ((*final_status & SPI_NOR_WIP_BIT) == 0U) {
			return 0;
		}
		if (!sys_timepoint_expired(deadline)) {
			k_sleep(K_USEC(NAND_STATUS_POLL_INTERVAL_US));
			continue;
		}

		/* The busy value may have been sampled before the deadline and only
		 * returned after it. Confirm with a new read initiated after expiry.
		 */
		uint8_t confirmation_status;

		ret = get_status(dev, &confirmation_status);
		if (ret != 0) {
			return ret;
		}
		*final_status = confirmation_status;
		if ((confirmation_status & SPI_NOR_WIP_BIT) == 0U) {
			return 0;
		}
		LOG_ERR("NAND ready timeout after %u us: status=0x%02x",
			timeout_us, confirmation_status);
		return -ETIMEDOUT;
	}
}



/* Everything necessary to acquire owning access to the device.
*
* This means taking the lock and, if necessary, waking the device
* from deep power-down mode.
*/
static void acquire_device(const struct device *dev)
{
	storage_spi_bus_lock();

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nor_data *const driver_data = dev->data;

		k_sem_take(&driver_data->sem, K_FOREVER);
	}

}



/* Everything necessary to release access to the device.
*
* This means (optionally) putting the device into deep power-down
* mode, and releasing the lock.
*/
static void release_device(const struct device *dev)
{

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nor_data *const driver_data = dev->data;

		k_sem_give(&driver_data->sem);
	}

	storage_spi_bus_unlock();
}




static int spi_unlock_memory(const struct device* dev){

	int ret = set_features(dev, REGISTER_BLOCKLOCK, 0);
	return ret;
}

// static bad block detection which attempts to figure out whether there were bad blocks set by the manufacturer via a bad block marking.
int detect_manufacturer_bad_blocks(const struct device* dev){
	int page_addr = 0;
	int bad_blocks = 0;
	uint8_t dest;
	uint8_t status;
	off_t error_address = 4096;
	int total_device_size = (dev_flash_size(dev) / dev_page_size(dev)) /
				NAND_PAGES_PER_ERASE_BLOCK;
	for (int x = 0; x < total_device_size; x++){
	page_addr = convert_block_to_singledie_address(x);
	acquire_device(dev);
	current_reads++;
	//LOG_DBG("reading bytes at address %d", page_addr);
	nrfx_err_t res = 0;


	__ASSERT(data != NULL, "null destination");

	uint8_t addr_buf[] = {
		page_addr >> 16,
		page_addr >> 8,
		page_addr,
	};
	
	uint8_t buffer_address[] = {
		error_address >> 16,
		error_address >> 8,
		error_address
	};

	
	spi_send_request pread_cinstr_cfg = {
		.opcode = SPI_NAND_PAGE_READ,
		.addr = addr_buf,
		.addr_length = 3,
	};


	spi_send_request cread_cinstr_cfg = {
		.opcode = SPI_NOR_CMD_READ,
		.addr = buffer_address,
		.addr_length = 3,
		.data = &dest,
		.data_length = 1
	};

	res = spi_nand_access(dev, &pread_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("read transfer err: %x", res);
		release_device(dev);
		return res;
	}
	res = spi_nand_wait_until_ready(dev, NAND_PAGE_READ_TIMEOUT_US,
					&status);
	if (res != 0) {
		LOG_WRN("read completion err: %x", res);
		release_device(dev);
		return res;
	}

	res = spi_nand_access(dev, &cread_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("buff transfer err: %x", res);
		release_device(dev);
		return res;
	}
	if (status != 0){
	LOG_WRN("read with stat %i", status);
	}
	release_device(dev);
	//LOG_DBG("bad block value: %i", dest);
	if (dest != 255){
		bad_blocks++;
	}
	}
	LOG_INF("tot bad block: %d", bad_blocks);
	if (bad_blocks > 0){
		LOG_WRN("bad block count > 0");
	}
	return bad_blocks;
}


int spi_nand_parameter_page_read(const struct device* dev, void* dest){
	uint8_t current_config;
	int ret = get_features(dev, REGISTER_CONFIGURATION, &current_config);

	if (ret != 0) {
		return ret;
	}
	uint8_t current_config_mask = current_config | 0x5; 
	uint8_t code = current_config_mask & 0x3;
	ret = set_features(dev, REGISTER_CONFIGURATION, code);
	if (ret != 0) {
		return ret;
	}

	ret = spi_nand_page_read(dev, 0x01, dest);
	int restore_ret = set_features(dev, REGISTER_CONFIGURATION, current_config);
	return ret != 0 ? ret : restore_ret;
}

// since spi_nand_page_read only works on one flash, we have to do work to make it work 
int multi_nand_page_read(const struct device* dev, uint32_t page_number, void* buffer){
	int ret;
	if (current_reads % 5000 == 1000){
		print_bad_sect_info();
	}
	int non_corrupt_sector = get_sector_offset(page_number);
	off_t addr = convert_page_to_address(dev, non_corrupt_sector);
	if (addr < 0) {
		return (int)addr;
	}
	ret = spi_nand_page_read(dev, addr, buffer);
	if (ret != 0) {
		LOG_ERR("NAND read mapping: logical_page=%u mapped_page=%d rc=%d",
			page_number, non_corrupt_sector, ret);
	}
	if (ret == FLASH_TOO_MANY_ECC_ERROR){
		register_bad_sector(non_corrupt_sector);
	}
	return ret;
}

int spi_nand_page_read(const struct device* dev, off_t page_addr, void* dest){
	current_reads++;
	acquire_device(dev);
	LOG_DBG("reading bytes at address %ld", page_addr);
	int res = 0;
	int wait_res = 0;
	uint8_t reg_status = 0;
	uint8_t ecc_status;
	int status;

	uint8_t addr_buf[] = {
		page_addr >> 16,
		page_addr >> 8,
		page_addr,
	};
	uint8_t buffer_address[] = {0, 0, 0};

	
	spi_send_request pread_cinstr_cfg = {
		.opcode = SPI_NAND_PAGE_READ,
		.addr = addr_buf,
		.addr_length = 3,
	};


	spi_send_request cread_cinstr_cfg = {
		.opcode = SPI_NOR_CMD_READ,
		.addr = buffer_address,
		.addr_length = 3,
		.data = dest,
		.data_length = 4096
	};

	res = spi_nand_access(dev, &pread_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("read transfer error: %x", res);
		goto out;
	}
	wait_res = spi_nand_wait_until_ready(dev, NAND_PAGE_READ_TIMEOUT_US,
					   &reg_status);
	if (wait_res != 0) {
		LOG_WRN("read completion error: %d", wait_res);
		goto out;
	}

	res = spi_nand_access(dev, &cread_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("buffer transfer error: %x", res);
		goto out;
	}

out:
	ecc_status = (reg_status >> 4) & 0x07U;
	status = res != 0 ? (int)res : wait_res;

	LOG_DBG("finished read! with status 0x%02x", reg_status);
	switch (status == 0 ? ecc_status : 0U) {
	case 0:
		break;
	case 1:
	case 3:
	case 5:
		/* Micron M70A reports corrected bit-flip ranges with these codes. */
		ECC_corrections++;
		LOG_WRN("NAND ECC corrected: cs=%d die=%d die_page=%ld sr=0x%02x ecc=%u total=%d",
			current_flash, current_die[current_flash], page_addr,
			reg_status, ecc_status, ECC_corrections);
		break;
	case 2:
		ECC_err++;
		LOG_ERR("NAND ECC uncorrectable: cs=%d die=%d die_page=%ld die_block=%ld page_in_block=%ld sr=0x%02x ecc=%u total=%d",
			current_flash, current_die[current_flash], page_addr,
			(long)(page_addr / NAND_PAGES_PER_ERASE_BLOCK),
			(long)(page_addr % NAND_PAGES_PER_ERASE_BLOCK),
			reg_status, ecc_status, ECC_err);
		status = FLASH_TOO_MANY_ECC_ERROR;
		break;
	default:
		LOG_ERR("NAND unexpected status: cs=%d die=%d die_page=%ld sr=0x%02x ecc=%u oip=%u wel=%u erase_fail=%u prog_fail=%u",
			current_flash, current_die[current_flash], page_addr,
			reg_status, ecc_status, reg_status & BIT(0),
			(reg_status >> 1) & BIT(0), (reg_status >> 2) & BIT(0),
			(reg_status >> 3) & BIT(0));
		status = -EIO;
		break;
	}

	release_device(dev);
	
	return status;
}



int spi_nand_page_write(const struct device* dev, off_t page_address, const void* src, size_t size){
	int disable_ret;
	int ret;
	uint8_t status = 0;
	bool operation_busy_unknown = false;

	current_writes++;
	acquire_device(dev);
	LOG_DBG("writing %d bytes at address %ld", size, page_address);

	uint8_t pe_addr_buf[] = {
	page_address >> 16,
	page_address >> 8,
	page_address,	
	};

	uint8_t pl_addr_buf[] = {0, 0};
	// Program Load requires the data
	spi_send_request pl_cinstr_cfg = {
		.opcode = SPI_NAND_PL,
		.addr = pl_addr_buf,
		.addr_length = 2,
		.data = src,
		.data_length = size,
		.is_write = true
	};

	spi_send_request pe_cinstr_cfg = {
		.opcode = SPI_NAND_PE,
		.addr = pe_addr_buf,
		.addr_length = 3
	};
	ret = write_enable(dev);
	if (ret != 0) {
		goto cleanup;
	}

	ret = spi_nand_access(dev, &pl_cinstr_cfg);
	if (ret != 0) {
		LOG_WRN("program load failed: %d", ret);
		goto cleanup;
	}

	operation_busy_unknown = true;
	status = SPI_NOR_WIP_BIT;
	ret = spi_nand_access(dev, &pe_cinstr_cfg);
	if (ret != 0) {
		int settle_ret;

		LOG_WRN("program execute failed: %d", ret);
		settle_ret = spi_nand_wait_until_ready(dev,
					 NAND_PAGE_PROGRAM_TIMEOUT_US, &status);
		operation_busy_unknown = (settle_ret != 0) &&
					 ((status & SPI_NOR_WIP_BIT) != 0U);
		if (settle_ret != 0) {
			LOG_WRN("program settle failed: %d", settle_ret);
		}
		goto cleanup;
	}

	ret = spi_nand_wait_until_ready(dev, NAND_PAGE_PROGRAM_TIMEOUT_US,
					&status);
	operation_busy_unknown = (ret != 0) &&
				 ((status & SPI_NOR_WIP_BIT) != 0U);
	if ((ret == 0) && ((status & NAND_STATUS_PROGRAM_FAIL) != 0U)) {
		LOG_ERR("program failed: status=0x%02x", status);
		ret = -EIO;
	} else if (ret == 0) {
		LOG_DBG("program status=0x%02x", status);
	}

cleanup:
	if (operation_busy_unknown) {
		LOG_ERR("NAND busy or unknown; skipping program write disable");
		disable_ret = 0;
	} else {
		disable_ret = write_disable(dev);
	}
	if (ret == 0) {
		ret = disable_ret;
	} else if (disable_ret != 0) {
		LOG_WRN("program write disable failed: %d", disable_ret);
	}
	release_device(dev);
	LOG_DBG("write completed with result %d", ret);
	if (ret != 0) {
		LOG_WRN("page write returned %d", ret);
	}
	return ret;
}



// addr is the first page of the block
static int spi_nand_block_erase_internal(const struct device* dev, off_t addr,
					uint8_t *completed_fail_status){
	int disable_ret;
	int ret;
	uint8_t status = 0;
	bool operation_busy_unknown = false;
	bool completed_erase_fail = false;

	if (completed_fail_status != NULL) {
		*completed_fail_status = 0;
	}

	acquire_device(dev);
	current_erases++;

	uint8_t pe_addr_buf[] = {
	addr >> 16,
	addr >> 8,
	addr,	
	};
	
	spi_send_request erase = {
		.opcode = SPI_NOR_CMD_BE,
		.addr = pe_addr_buf,
		.addr_length = 3
	};

	ret = write_enable(dev);
	if (ret != 0) {
		goto cleanup;
	}

	operation_busy_unknown = true;
	status = SPI_NOR_WIP_BIT;
	ret = spi_nand_access(dev, &erase);
	if (ret != 0) {
		int settle_ret;

		LOG_WRN("block erase command failed: %d", ret);
		settle_ret = spi_nand_wait_until_ready(dev,
					 NAND_BLOCK_ERASE_TIMEOUT_US, &status);
		operation_busy_unknown = (settle_ret != 0) &&
					 ((status & SPI_NOR_WIP_BIT) != 0U);
		if (settle_ret != 0) {
			LOG_WRN("block erase settle failed: %d", settle_ret);
		}
		goto cleanup;
	}

	ret = spi_nand_wait_until_ready(dev, NAND_BLOCK_ERASE_TIMEOUT_US,
					&status);
	operation_busy_unknown = (ret != 0) &&
				 ((status & SPI_NOR_WIP_BIT) != 0U);
	if ((ret == 0) && ((status & NAND_STATUS_ERASE_FAIL) != 0U)) {
		LOG_ERR("block erase failed: status=0x%02x", status);
		completed_erase_fail = true;
		ret = -EIO;
	} else if (ret == 0) {
		LOG_DBG("block erase status=0x%02x", status);
	}

cleanup:
	if (operation_busy_unknown) {
		LOG_ERR("NAND busy or unknown; skipping erase write disable");
		disable_ret = 0;
	} else {
		disable_ret = write_disable(dev);
	}
	if (ret == 0) {
		ret = disable_ret;
	} else if (disable_ret != 0) {
		LOG_WRN("erase write disable failed: %d", disable_ret);
	}
	if (completed_fail_status != NULL && completed_erase_fail && disable_ret == 0) {
		*completed_fail_status = status;
	}
	release_device(dev);
	LOG_DBG("erase completed with result %d", ret);
	return ret;
}

int spi_nand_block_erase(const struct device* dev, off_t addr){
	return spi_nand_block_erase_internal(dev, addr, NULL);
}


static int spi_nand_die_erase_for_format(const struct device *dev, int package,
					int die, unsigned int *skipped_total)
{
	int block_count = (dev_die_size(dev) / dev_page_size(dev)) /
			  NAND_PAGES_PER_ERASE_BLOCK;
	unsigned int skipped_die = 0;

	LOG_INF("NAND format package=%d die=%d blocks=%d", package, die,
		block_count);
	for (int block = 0; block < block_count; block++) {
		off_t page = convert_block_to_singledie_address(block);
		uint8_t fail_status;
		int ret = spi_nand_block_erase_internal(dev, page, &fail_status);

		if (fail_status != 0U) {
			LOG_ERR("NAND_FORMAT_ERASE_FAIL package=%d die=%d block=%d page=%ld status=0x%02x",
				package, die, block, (long)page, fail_status);
			if (skipped_die >= NAND_FORMAT_MAX_SKIPPED_BLOCKS_PER_DIE) {
				LOG_ERR("NAND format stopped: over %u failed blocks on package=%d die=%d",
					NAND_FORMAT_MAX_SKIPPED_BLOCKS_PER_DIE,
					package, die);
				return -EIO;
			}
			skipped_die++;
			(*skipped_total)++;
			continue;
		}
		if (ret != 0) {
			LOG_ERR("NAND format stopped: package=%d die=%d block=%d error=%d",
				package, die, block, ret);
			return ret;
		}
	}
	if (skipped_die != 0U) {
		LOG_WRN("NAND format package=%d die=%d skipped_blocks=%u",
			package, die, skipped_die);
	}
	return 0;
}

static int spi_nand_whole_chip_erase(const struct device *dev, int package,
				     unsigned int *skipped_total)
{
	const struct spi_flash_config *cfg = dev->config;
	int ret = 0;

	for (int die = 0; die < cfg->dies_per_flash; die++) {
		ret = set_die(dev, die);
		if (ret != 0) {
			break;
		}
		ret = spi_nand_die_erase_for_format(dev, package, die, skipped_total);
		if (ret != 0) {
			break;
		}
	}
	int restore_ret = set_die(dev, 0);
	return ret != 0 ? ret : restore_ret;
}

// resets the bad block storage.
int spi_nand_multi_chip_reset_bad_block(const struct device* dev){
	int ret = erase_bad_sectors_arr();
	if (ret != 0){
		LOG_ERR("fail to erase bad sect");
	}
	return ret;
}


int spi_nand_multi_chip_erase(const struct device* dev){
	const struct spi_flash_config* cfg = dev->config;
	unsigned int skipped_total = 0;
	int ret = 0;
	for (int i = 0; i < cfg->num_flashes; i++) {
		ret = set_flash(dev, i);
		if (ret != 0) {
			break;
		}
		ret = spi_nand_whole_chip_erase(dev, i, &skipped_total);
		if (ret != 0) {
			break;
		}
		LOG_INF("chip %i erased.", i + 1);
		k_sleep(K_MSEC(500));
	}
	int restore_ret = set_flash(dev, 0);
	if (ret != 0) {
		return ret;
	}
	if (restore_ret != 0) {
		return restore_ret;
	}
	LOG_INF("erasing file table (nor)");
	ret = erase_file_table();
	if (ret != 0){
		LOG_ERR("failed to erase file table");
		return ret;
	}
	if (skipped_total != 0U) {
		LOG_WRN("NAND_FORMAT_RESULT skipped_blocks=%u", skipped_total);
	}
	LOG_INF("format erase complete");
	return 0;
}


static int spi_read_jedec_id(const struct device *dev,
				uint8_t *id)
{
	if (id == NULL) {
		return -EINVAL;
	}

	acquire_device(dev);	
	int ret = spi_cmd(dev, SPI_NOR_CMD_RDID, id, SPI_MAX_ID_LEN);

	release_device(dev);

	return ret;
}

static int flash_reset_and_unlock(const struct device *dev)
{
	const struct spi_flash_config *cfg = dev->config;
	uint8_t status = 0;
	int restore_ret;
	int ret;

	/* Check for block protect bits that need to be cleared.  This
	* information cannot be determined from SFDP content, so the
	* devicetree node property must be set correctly for any device
	* that powers up with block protect enabled.
	*/
	acquire_device(dev);
	current_die[current_flash] = NAND_DIE_UNKNOWN;
	ret = spi_nand_reset(dev);
	/* A RESET transfer error is ambiguous, so keep the bus quiet regardless. */
	k_sleep(K_USEC(NAND_RESET_NO_COMMAND_US));
	if (ret != 0) {
		goto out;
	}
	ret = spi_nand_wait_until_ready(dev, NAND_RESET_POLL_TIMEOUT_US,
					&status);
	if (ret != 0) {
		goto out;
	}

	current_die[current_flash] = 0;
	for (int die = 0; die < cfg->dies_per_flash; die++) {
		ret = set_die(dev, die);
		if (ret != 0) {
			break;
		}
		ret = spi_unlock_memory(dev);
		if (ret != 0) {
			break;
		}
		LOG_DBG("NAND package %d die %d unlocked; reset status=0x%02x",
			current_flash, die, status);
	}

	restore_ret = set_die(dev, 0);
	if (ret == 0) {
		ret = restore_ret;
	} else if (restore_ret != 0) {
		LOG_WRN("failed to restore die 0: %d", restore_ret);
	}

out:
	release_device(dev);
	return ret;
}


/**
 * @brief Configure the flash
 *
 * @param dev The flash device structure
 * @param info The flash info structure
 * @return 0 on success, negative errno code otherwise
 */
static int spi_configure(const struct device *dev, const struct spi_flash_config* cfg)
{
	
	uint8_t jedec_id[SPI_MAX_ID_LEN];
	int rc;

	/* Validate bus and CS is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		return -ENODEV;
	}

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	if (!gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("Reset pin not ready");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE)) {
		LOG_ERR("Couldn't configure reset pin");
		return -ENODEV;
	}
	rc = gpio_pin_set_dt(&cfg->reset, 0);
	if (rc) {
		return rc;
	}
#endif

	/* After a soft-reset the flash might be in DPD or busy writing/erasing.
	* Exit DPD and wait until flash is ready.
	*/

	/* now the spi bus is configured, we can verify SPI
	* connectivity by reading the JEDEC ID.
	*/

	rc = spi_read_jedec_id(dev, jedec_id);
	if (rc != 0) {
		LOG_ERR("JEDEC ID read failed: %d", rc);
		return -ENODEV;
	}

	/* For minimal and devicetree we need to check the JEDEC ID
	* against the one from devicetree, to ensure we didn't find a
	* device that has different parameters.
	*/
	jedec_id[0] = 0xff;
	if (memcmp(jedec_id, cfg->jedec_id, sizeof(jedec_id)) != 0) {
		LOG_ERR("Device id %02x %02x %02x does not match config %02x %02x %02x",
			jedec_id[0], jedec_id[1], jedec_id[2],
			cfg->jedec_id[0], cfg->jedec_id[1], cfg->jedec_id[2]);
		return -EINVAL;
	}

	return flash_reset_and_unlock(dev);
}

#ifdef CONFIG_PM_DEVICE

static int spi_nor_pm_control(const struct device *dev, enum pm_device_action action)
{
	int rc = 0;

	switch (action) {
#ifdef CONFIG_SPI_NOR_IDLE_IN_DPD
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_RESUME:
		break;
#else
	case PM_DEVICE_ACTION_SUSPEND:
		acquire_device(dev);
		rc = enter_dpd(dev);
		release_device(dev);
		break;
	case PM_DEVICE_ACTION_RESUME:
		acquire_device(dev);
		rc = exit_dpd(dev);
		release_device(dev);
		break;
#endif /* CONFIG_SPI_NOR_IDLE_IN_DPD */
	case PM_DEVICE_ACTION_TURN_ON:
		/* Coming out of power off */
		rc = spi_nor_configure(dev);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		rc = -ENOSYS;
	}

	return rc;
}

#endif /* CONFIG_PM_DEVICE */

/**
 * @brief Initialize and configure the flash
 *
 * @param name The flash name
 * @return 0 on success, negative errno code otherwise
 */
int spi_init(const struct device *dev)
{
	int ret;
	const struct spi_flash_config* cfg = dev->config;

	for (int i = 0; i < cfg->num_flashes; i++) {
		current_die[i] = NAND_DIE_UNKNOWN;
	}
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nor_data* const driver_data = dev->data;

		k_sem_init(&driver_data->sem, 1, K_SEM_MAX_LIMIT);
		k_sem_init(&driver_data->sem_inner, 1, K_SEM_MAX_LIMIT);
	}
	ret = spi_configure(dev, cfg);
	if (ret != 0)
		return ret;
	// we set the correct cs pin already via set_flash which works through spi_nand_access, so no need for a custom config struct.
	for (int i = 1; i < cfg->num_flashes; i++) {
		ret = set_flash(dev, i);
		if (ret != 0) {
			break;
		}
		ret = spi_configure(dev, cfg);
		if (ret != 0) {
			break;
		}
	}

	int restore_ret = set_flash(dev, 0);
	if ((ret == 0) && (restore_ret == 0)) {
		LOG_INF("NAND initialized: %d packages, JEDEC %02x %02x %02x",
			cfg->num_flashes, cfg->jedec_id[0], cfg->jedec_id[1],
			cfg->jedec_id[2]);
	}
	return ret != 0 ? ret : restore_ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

static void spi_nor_pages_layout(const struct device *dev,
				const struct flash_pages_layout **layout,
				size_t *layout_size)
{
	/* Data for runtime, const for devicetree and minimal. */


	const struct spi_flash_config *cfg = dev->config;

	*layout = &cfg->layout;

	*layout_size = 1;
}

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters* flash_nor_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_nor_parameters;
}


void print_page_hex(uint8_t* data_buf, int size, bool shorten){
	// can easily modify this to support other types like char or int
	if (shorten && size > 250){
		size = 250;
	}
	printk("data: ");
	for (int i = 0; i < size; i ++){
		printk("%02x ", data_buf[i]);
		if (i % 19 == 18) {
			printk("\n");
		}
		// just to clear the buffer
		if (i % 300 == 299) {
			k_sleep(K_MSEC(400));
		}
	}
	printk("\n end \n");
}
