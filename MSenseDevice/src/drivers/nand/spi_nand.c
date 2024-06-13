/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 * Copyright (c) 2020 Peter Bigot Consulting, LLC
 * Copyright (c) 2024 SENSE Lab Ohio State
 * 
 * This driver is heavily inspired from the spi_flash_w25qxxdv.c SPI NOR driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#define CONFIG_NORDIC_QSPI_NOR_STACK_WRITE_BUFFER_SIZE 4


#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/pm/device.h>

#include "spi_nand.h"
#include "jesd216.h"
#include "flash_priv.h"

int erase_file_table();

LOG_MODULE_REGISTER(spi_nand, CONFIG_FLASH_LOG_LEVEL);

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


#ifdef CONFIG_SPI_NOR_SFDP_MINIMAL
/* The historically supported erase sizes. */
static const struct jesd216_erase_type minimal_erase_types[JESD216_NUM_ERASE_TYPES] = {
	{
		.cmd = SPI_NOR_CMD_BE,
		.exp = 16,
	},
	{
		.cmd = SPI_NOR_CMD_SE,
		.exp = 12,
	},
};
#endif /* CONFIG_SPI_NOR_SFDP_MINIMAL */


int current_writes = 0;
int current_reads = 0;
int current_erases = 0;


int current_die = 0;

// Second die for the second flash.
int current_die2 = 0;


// parameter for multiple flashes.
int current_flash = 0;

// TODO: put these in device tree
const int num_of_flashes = 2;
const int die_per_flash = 2;

static int spi_nor_write_protection_set(const struct device *dev,
					bool write_protect);

struct jesd216_erase_type erasetype = {
		.cmd = SPI_NOR_CMD_BE
	};
/* Get pointer to array of supported erase types.  Static const for
 * minimal, data for runtime and devicetree.
 */
static inline const struct jesd216_erase_type* dev_erase_types(const struct device *dev)
{
	
	return &erasetype;
}

/* Get the size of the flash device.  Data for runtime, constant for
 * minimal and devicetree.
 */
inline uint32_t dev_flash_size(const struct device *dev)
{
#ifdef CONFIG_SPI_NOR_SFDP_RUNTIME
	const struct spi_nor_data *data = dev->data;

	return data->flash_size;
#else /* CONFIG_SPI_NOR_SFDP_RUNTIME */
	const struct spi_flash_config* cfg = dev->config;

	return cfg->flash_size;
#endif /* CONFIG_SPI_NOR_SFDP_RUNTIME */
}

inline int dev_die_size(const struct device* dev){
	return dev_flash_size(dev) / (num_of_flashes*die_per_flash);
}

/* Get the flash device page size.  Constant for minimal, data for
 * runtime and devicetree.
 */
inline uint16_t dev_page_size(const struct device *dev)
{
#ifdef CONFIG_SPI_NOR_SFDP_MINIMAL
	return 256;
#else /* CONFIG_SPI_NOR_SFDP_MINIMAL */
	const struct spi_nor_data *data = dev->data;
	uint16_t size = data->page_size;
	if (size == 0){
		return 4096;
	}
	else{
		return size;
	}
#endif /* CONFIG_SPI_NOR_SFDP_MINIMAL */
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

/* Indicates that an access command includes bytes for the address.
 * If not provided the opcode is not followed by address bytes.
 */
#define NOR_ACCESS_ADDRESSED BIT(0)

/* Indicates that addressed access uses a 24-bit address regardless of
 * spi_nor_data::flag_32bit_addr.
 */
#define NOR_ACCESS_24BIT_ADDR BIT(1)

/* Indicates that addressed access uses a 32-bit address regardless of
 * spi_nor_data::flag_32bit_addr.
 */
#define NOR_ACCESS_32BIT_ADDR BIT(2)

/* Indicates that an access command is performing a write.  If not
 * provided access is a read.
 */
#define NOR_ACCESS_WRITE BIT(7)


off_t convert_to_address(uint32_t page, uint32_t block){
	return page + (block * 64);
}

// The pages representing a block are from block - 65.
// 4 gigabit is 536870912 bytes / 4096 = 131072 pages (131071 is last address)
off_t convert_page_to_address(const struct device* dev, uint32_t page){

	// total number of sectors per die. 
	int die_size = 131072;
	int selected_die_num = page / die_size;
	LOG_DBG("die/flash  number: %d", selected_die_num);
	if (page < die_size){
		
		set_flash(dev, 0);
		set_die(dev, 0);
		
	}
	else if (page < die_size*2){
		
		set_flash(dev, 0);
		set_die(dev, 1);
	}
	else if (page < die_size*3) {
		
		set_flash(dev, 1);
		set_die(dev, 0);
		
	}
	else if (page < die_size*4){
		
		set_flash(dev, 1);
		set_die(dev, 1);
		
	}
	return page - die_size*selected_die_num;

}

off_t convert_block_to_address(uint32_t block){
	//werid fix because of noticed offsets, perhaps there is another issue we are unaware of.
	return (block * 64);
}

int convert_block_to_first_page_address(uint32_t block){

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
	struct spi_nor_data *const driver_data = dev->data;

	// get parameters needed for spi_transceive and spi_write
	struct spi_dt_spec spi_spec = driver_cfg->spi;
	struct spi_config spi_flash_cfg = spi_spec.config;
	if (current_flash){
		gpio_pin_t pin2 = 4;
		spi_flash_cfg.cs.gpio.pin = pin2;
	}
	int ret;
	
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



static uint8_t get_status(const struct device* dev){

	uint8_t data;
	uint8_t reg_cmd = 0xC0;

	data = get_features(dev, reg_cmd);
	return data;

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
			LOG_WRN("write disable failed");
	}
	return ret;
}


int reset(const struct device* dev){

	spi_cmd(dev, SPI_NAND_RESET, NULL, 0);
	
	//spi_rdsr(dev);

	return 0;
}

int set_die(const struct device* dev, int die_select){
	
	uint8_t feature = 0x0;
    if (die_select == 1) {
		feature = 0x40;
	}
	int ret = set_features(dev, REGISTER_DIESELECT, feature);
	if (ret == 0){
		if (current_flash){
			current_die2 = die_select;
		}
		else {
			current_die = die_select;
		}
		LOG_DBG("current die %d second flash current die: %d", current_die, current_die2);
	}
	else{
		LOG_WRN("error with die setting");
	}
	return ret;

}




int set_flash(const struct device* dev, int flash_id){
	const struct spi_flash_config* const driver_cfg = dev->config;
	current_flash = flash_id;
	return 0;
}


uint8_t get_features(const struct device* dev, uint8_t register_select){

	uint8_t data;

	spi_send_request request = {
		.opcode = SPI_NAND_GF,
		.addr = &register_select,
		.addr_length = 1,
		.data = &data,
		.data_length = 1,
	};

	int res = spi_nand_access(dev, &request);

	if (res == 0){
		return data;
	}
	else {
		LOG_WRN("get features error");
		return 253;
	}
}

// Should only be used to set all features. to only set an induvidual feature, use set_feature 
int set_features(const struct device* dev, uint8_t register_select, uint8_t data){
	 
	//LOG_INF("setting features for value: %d", data);
	uint8_t data_arr[] = {
		register_select,
		data

	};
	spi_send_request write_features_request = {
		.opcode = 0x1F,
		.is_write = true,
		.addr = &register_select,
		.addr_length = 1,
		.data = &data,
		.data_length = 1
	};

	int res = spi_nand_access(dev, &write_features_request);
	if (res == 0){
		uint8_t readback = get_features(dev, register_select);
	if (readback == data){
		return 0;
	}
	else{
		return NRFX_ERROR_NOT_SUPPORTED;
	}
	}
	else {
		return -1;
	}

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
int spi_flash_wait_until_ready(const struct device *dev)
{
	int waitcycles = 0;
	int ret = 0;
	uint8_t reg;

	do {
		waitcycles++;
		reg = get_status(dev);
	} while (!ret && (reg & SPI_NOR_WIP_BIT));
	LOG_DBG("wait completed with %i cycles", waitcycles);
	return ret;
}



/* Everything necessary to acquire owning access to the device.
 *
 * This means taking the lock and, if necessary, waking the device
 * from deep power-down mode.
 */
static void acquire_device(const struct device *dev)
{
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
}




/**
 * @brief Read the status register.
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * @param dev Device struct
 *
 * @return the non-negative value of the status register, or an error code.
 */
uint8_t spi_rdsr(const struct device *dev)
{
	uint8_t status = get_status(dev);
	if (status > 3){
	 LOG_WRN("status register: %d", status);
	}
	
	return status;
}

/**
 * @brief Write the status register.
 *
 * @note The device must be externally acquired before invoking this
 * function.
 *
 * @param dev Device struct
 * @param sr The new value of the status register
 *
 * @return 0 on success or a negative error code.
 */
int spi_nor_wrsr(const struct device *dev,
			uint8_t sr)
{	
	int ret = set_features(dev, REGISTER_STATUS, sr);
	spi_flash_wait_until_ready(dev);
	

	return ret;
}

int spi_unlock_memory(const struct device* dev){

	set_features(dev, REGISTER_BLOCKLOCK, 0);
}

int detect_bad_blocks(const struct device* dev){
	int page_addr = 0;
	int bad_blocks = 0;
	uint8_t dest;
	off_t error_address = 4096;
	int total_device_size = (dev_flash_size(dev) / dev_page_size(dev)) / 64;
	for (int x = 0; x < total_device_size; x++){
	page_addr = convert_block_to_address(x);
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
		LOG_WRN("read transfer error: %x", res);
		continue;
	}
	spi_flash_wait_until_ready(dev);

	res = spi_nand_access(dev, &cread_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("buffer transfer error: %x", res);
		continue;
	}
	uint8_t status = spi_rdsr(dev);
	if (status != 0){
	LOG_WRN("finished read! with status %i", status);
	}
	release_device(dev);
	//LOG_DBG("bad block value: %i", dest);
	if (dest != 255){
		bad_blocks++;
	}
	}
	LOG_DBG("total bad blocks: %d", bad_blocks);
	

}


int spi_nand_parameter_page_read(const struct device* dev, void* dest){
	
	uint8_t current_config = get_features(dev, REGISTER_CONFIGURATION);
	uint8_t current_config_mask = current_config | 0x5; 
	uint8_t code = current_config_mask & 0x3;
	int ret = set_features(dev, REGISTER_CONFIGURATION, code);

	spi_nand_page_read(dev, 0x01, dest);
	ret = set_features(dev, REGISTER_CONFIGURATION, current_config);
	
	return ret;
}


int spi_nand_page_read(const struct device* dev, off_t page_addr, void* dest){
	current_reads++;
	acquire_device(dev);
	LOG_DBG("reading bytes at address %d", page_addr);
	nrfx_err_t res = 0;


	__ASSERT(data != NULL, "null destination");

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
	spi_flash_wait_until_ready(dev);

	res = spi_nand_access(dev, &cread_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("buffer transfer error: %x", res);
		goto out;
	}

out:
	uint8_t status = spi_rdsr(dev);
	LOG_DBG("finished read! with status %i", status);
	release_device(dev);
	return status;
}

static int spi_nand_read(const struct device *dev, off_t addr, void *dest,
			size_t size)
{
	
	const size_t flash_size = dev_die_size(dev);
	int ret;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((addr + size) > flash_size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	//CODE GOES HERE
	

	release_device(dev);
	return ret;
}


int spi_nand_page_write(const struct device* dev, off_t page_address, const void* src, size_t size){
	current_writes++;
	acquire_device(dev);
	LOG_DBG("writing %d bytes at address %d", size, page_address);
	nrfx_err_t res = 0;

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
	res = write_enable(dev);

	res = spi_nand_access(dev, &pl_cinstr_cfg);
	if (res != 0) {
		LOG_WRN("load error: %x", res);
		release_device(dev);
		return res;
	}

	//LOG_DBG("load completed!");

	//Start Execute Process

	res = spi_nand_access(dev, &pe_cinstr_cfg);
	if (res != 0){
		LOG_WRN("lfm_start: %x", res);
		release_device(dev);
		return res;
	}
	// wait for operation to finish, issue the get feature command.
	spi_flash_wait_until_ready(dev);
	//k_sleep()
	uint8_t status = spi_rdsr(dev);
	write_disable(dev);
	release_device(dev);
	
	LOG_DBG("write completed! with status %i", status);
	
	
	return status;

}



static int spi_nand_write(const struct device *dev, off_t addr,
			 const void *src,
			 size_t size)
{
	
	const size_t flash_size = dev_die_size(dev);
	const uint16_t page_size = dev_page_size(dev);
	int ret = 0;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > flash_size)) {
		return -EINVAL;
	}

	acquire_device(dev);
	
	ret = spi_nor_write_protection_set(dev, false);
	if (ret == 0) {
		while (size > 0) {
			size_t to_write = size;

			/* Don't write more than a page. */
			if (to_write >= page_size) {
				to_write = page_size;
			}

			/* Don't write across a page boundary */
			if (((addr + to_write - 1U) / page_size)
			!= (addr / page_size)) {
				to_write = page_size - (addr % page_size);
			}

			write_enable(dev);
			//ret = spi_nor_cmd_addr_write(dev, SPI_NOR_CMD_PP, addr,
			//			src, to_write);
			spi_nand_page_write(dev, addr, src, to_write);
			if (ret != 0) {
				break;
			}

			size -= to_write;
			src = (const uint8_t *)src + to_write;
			addr += to_write;

			spi_flash_wait_until_ready(dev);
		}
	}

	int ret2 = spi_nor_write_protection_set(dev, true);

	if (!ret) {
		ret = ret2;
	}

	release_device(dev);
	return ret;
}

int spi_nand_block_erase(const struct device* dev, off_t block_addr){
	acquire_device(dev);
	current_erases++;

	//LOG_DBG("erasing block at %d", block_addr);
	write_enable(dev);
	uint8_t pe_addr_buf[] = {
	block_addr >> 16,
	block_addr >> 8,
	block_addr,	
	};
	
	spi_send_request erase = {
		.opcode = SPI_NOR_CMD_BE,
		.addr = pe_addr_buf,
		.addr_length = 3
	};
	

	spi_nand_access(dev, &erase);
	spi_flash_wait_until_ready(dev);
	int status = spi_rdsr(dev);
	write_disable(dev);
	LOG_DBG("erase completed! with status %i", status);
	release_device(dev);

	return status;

}


int spi_nand_chip_erase(const struct device* device) {
	

	//set_die(device, 0);
	// Get the total size in bytes of the flash, and divide by 2 because there are 2 die
	size_t size = dev_die_size(device);
	off_t block_address;
	int status = -1;
	int page_size = dev_page_size(device);
	//Divide by page size to get the total pages, then by pages per block to get block size
	int block_count = (size / page_size);
	block_count /= 64;
	//block_count = 4096;
	LOG_INF("chip erase start, for %i blocks", block_count);
	for (int current_block = 0; current_block <= block_count; current_block++){
		block_address = convert_block_to_address(current_block);
		status = spi_nand_block_erase(device, block_address);
		if (status != 0){
			LOG_WRN("error in chip erase: %i", status);
			continue;
		}
	}
	LOG_INF("chip erase complete! with status, %i", status); 
	return status;
}


int spi_nand_whole_chip_erase(const struct device* dev){
	
	int ret = 0;
	ret = set_die(dev, 0);
	ret = spi_nand_chip_erase(dev);
	ret = set_die(dev, 1);
	ret = spi_nand_chip_erase(dev);
	ret = set_die(dev, 0);
	
	return ret;
}

int spi_nand_multi_chip_erase(const struct device* dev){

	set_flash(dev, 0);
	spi_nand_whole_chip_erase(dev);
	LOG_INF("first chip erase complete, starting second...");
	k_sleep(K_MSEC(500));
	set_flash(dev, 1);
	spi_nand_whole_chip_erase(dev);
	set_flash(dev, 0);
	erase_file_table();
	LOG_INF("all chip erases complete!");
}


static int spi_nand_erase(const struct device *dev, off_t addr, size_t size)
{
	current_erases++;
	const size_t flash_size = dev_die_size(dev);
	int ret = 0;

	/* erase area must be subregion of device */
	if ((addr < 0) || ((size + addr) > flash_size)) {
		return -EINVAL;
	}

	/* address must be sector-aligned */
	if (!SPI_NOR_IS_SECTOR_ALIGNED(addr)) {
		return -EINVAL;
	}

	/* size must be a multiple of sectors */
	if ((size % SPI_NOR_SECTOR_SIZE) != 0) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = spi_nor_write_protection_set(dev, false);

	while ((size > 0) && (ret == 0)) {
		write_enable(dev);

		if (size == flash_size) {
			/* chip erase */
			//spi_nor_cmd_write(dev, SPI_NOR_CMD_CE);
			size -= flash_size;
		} else {
			const struct jesd216_erase_type *erase_types =
				dev_erase_types(dev);
			const struct jesd216_erase_type *bet = NULL;

			for (uint8_t ei = 0; ei < JESD216_NUM_ERASE_TYPES; ++ei) {
				const struct jesd216_erase_type *etp =
					&erase_types[ei];

				if ((etp->exp != 0)
				    && SPI_NOR_IS_ALIGNED(addr, etp->exp)
				    && (size >= BIT(etp->exp))
				    && ((bet == NULL)
					|| (etp->exp > bet->exp))) {
					bet = etp;
				}
			}
			if (bet != NULL) {
				//spi_nor_cmd_addr_write(dev, bet->cmd, addr, NULL, 0);
				addr += BIT(bet->exp);
				size -= BIT(bet->exp);
			} else {
				LOG_DBG("Can't erase %zu at 0x%lx",
					size, (long)addr);
				ret = -EINVAL;
			}
		}

#ifdef __XCC__
		/*
		 * FIXME: remove this hack once XCC is fixed.
		 *
		 * Without this volatile return value, XCC would segfault
		 * compiling this file complaining about failure in CGPREP
		 * phase.
		 */
		volatile int xcc_ret =
#endif
		spi_flash_wait_until_ready(dev);
	}

	int ret2 = spi_nor_write_protection_set(dev, true);

	if (!ret) {
		ret = ret2;
	}

	release_device(dev);

	return ret;
}

/* @note The device must be externally acquired before invoking this
 * function.
 */
static int spi_nor_write_protection_set(const struct device *dev,
					bool write_protect)
{
	int ret;

	if (write_protect){
	ret = write_enable(dev); 
	}
	else{ 
		ret = write_disable(dev);
	}

	if (IS_ENABLED(DT_INST_PROP(0, requires_ulbpr))
	    && (ret == 0)
	    && !write_protect) {
		//ret = spi_nor_cmd_write(dev, SPI_NOR_CMD_ULBPR);
	}

	return ret;
}

#if defined(CONFIG_FLASH_JESD216_API) || defined(CONFIG_SPI_NOR_SFDP_RUNTIME)

static int spi_nor_sfdp_read(const struct device *dev, off_t addr,
			     void *dest, size_t size)
{
	acquire_device(dev);

	int ret = read_sfdp(dev, addr, dest, size);

	release_device(dev);

	return ret;
}

#endif /* CONFIG_FLASH_JESD216_API || CONFIG_SPI_NOR_SFDP_RUNTIME */

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

/* Put the device into the appropriate address mode, if supported.
 *
 * On successful return spi_nor_data::flag_access_32bit has been set
 * (cleared) if the device is configured for 4-byte (3-byte) addresses
 * for read, write, and erase commands.
 *
 * @param dev the device
 *
 * @param enter_4byte_addr the Enter 4-Byte Addressing bit set from
 * DW16 of SFDP BFP.  A value of all zeros or all ones is interpreted
 * as "not supported".
 *
 * @retval -ENOTSUP if 4-byte addressing is supported but not in a way
 * that the driver can handle.
 * @retval negative codes if the attempt was made and failed
 * @retval 0 if the device is successfully left in 24-bit mode or
 *         reconfigured to 32-bit mode.
 */
static int spi_nor_set_address_mode(const struct device* dev,
				    uint8_t enter_4byte_addr)
{
	int ret = 0;

	/* Do nothing if not provided (either no bits or all bits
	 * set).
	 */
	if ((enter_4byte_addr == 0)
	    || (enter_4byte_addr == 0xff)) {
		return 0;
	}

	LOG_DBG("Checking enter-4byte-addr %02x", enter_4byte_addr);

	/* This currently only supports command 0xB7 (Enter 4-Byte
	 * Address Mode), with or without preceding WREN.
	 */
	if ((enter_4byte_addr & 0x03) == 0) {
		return -ENOTSUP;
	}

	acquire_device(dev);

	if ((enter_4byte_addr & 0x02) != 0) {
		/* Enter after WREN. */
		ret = write_enable(dev);
	}
	
	if (ret == 0) {
		ret = spi_cmd(dev, SPI_NOR_CMD_4BA, NULL, 0);
	}

	if (ret == 0) {
		struct spi_nor_data *data = dev->data;

		data->flag_access_32bit = true;
	}

	release_device(dev);

	return ret;
}

static int flash_reset_and_unlock(const struct device* dev){
	

	/* Check for block protect bits that need to be cleared.  This
	 * information cannot be determined from SFDP content, so the
	 * devicetree node property must be set correctly for any device
	 * that powers up with block protect enabled.
	 */
	acquire_device(dev);
	reset(dev);
	spi_flash_wait_until_ready(dev);
	spi_unlock_memory(dev);
	uint8_t status = spi_rdsr(dev);
	uint8_t configuration = get_features(dev, REGISTER_CONFIGURATION);
	uint8_t blocklock = get_features(dev, REGISTER_BLOCKLOCK);
	LOG_INF("status register: %d", status);
	LOG_INF("Configuration: %i", configuration);
	LOG_INF("BlockLock: %i", blocklock);
	release_device(dev);
	

}


/**
 * @brief Configure the flash
 *
 * @param dev The flash device structure
 * @param info The flash info structure
 * @return 0 on success, negative errno code otherwise
 */
static int spi_configure(const struct device *dev, struct spi_flash_config* cfg)
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

#ifndef CONFIG_SPI_NOR_SFDP_RUNTIME
	/* For minimal and devicetree we need to check the JEDEC ID
	 * against the one from devicetree, to ensure we didn't find a
	 * device that has different parameters.
	 */

	if (memcmp(jedec_id, cfg->jedec_id, sizeof(jedec_id)) != 0) {
		LOG_ERR("Device id %02x %02x %02x does not match config %02x %02x %02x",
			jedec_id[0], jedec_id[1], jedec_id[2],
			cfg->jedec_id[0], cfg->jedec_id[1], cfg->jedec_id[2]);
		return -EINVAL;
	}
	else {
		LOG_INF("ID %02x %02x %02x correct!", jedec_id[0], jedec_id[1], jedec_id[2]);
	}
#endif

	flash_reset_and_unlock(dev); 
	
	/*
	if (cfg->has_lock != 0) {
		acquire_device(dev);

		uint8_t status = spi_rdsr(dev);
		LOG_INF("Status Register: %d", status);
		if (rc > 0) {
			//rc = spi_nor_wrsr(dev, rc & ~cfg->has_lock);
		}

		if (rc != 0) {
			LOG_ERR("BP clear failed: %d\n", rc);
			return -ENODEV;
		}
	*/
		
	

#ifdef CONFIG_SPI_NOR_SFDP_MINIMAL
	/* For minimal we support some overrides from specific
	 * devicertee properties.
	 */
	if (cfg->enter_4byte_addr != 0) {
		rc = spi_nor_set_address_mode(dev, cfg->enter_4byte_addr);

		if (rc != 0) {
			LOG_ERR("Unable to enter 4-byte mode: %d\n", rc);
			return -ENODEV;
		}
	}

#else 

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	//rc = setup_pages_layout(dev);
	if (rc != 0) {
		LOG_ERR("layout setup failed: %d", rc);
		return -ENODEV;
	}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#endif /* CONFIG_SPI_NOR_SFDP_MINIMAL */

/*
#if DT_INST_NODE_HAS_PROP(0, mxicy_mx25r_power_mode)
	
	(void) mxicy_configure(dev, jedec_id);
#endif 

	if (IS_ENABLED(CONFIG_SPI_NOR_IDLE_IN_DPD)
	    && (enter_dpd(dev) != 0)) {
		return -ENODEV;
	}
*/
	return 0;
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
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nor_data* const driver_data = dev->data;

		k_sem_init(&driver_data->sem, 1, K_SEM_MAX_LIMIT);
		k_sem_init(&driver_data->sem_inner, 1, K_SEM_MAX_LIMIT);
	}
	struct spi_flash_config *cfg = dev->config;
	// TODO: go through device tree and get multiple config options
	ret = spi_configure(dev, cfg);
	if (ret == 0){
		// Configure the second spi_flash
		gpio_pin_t second_pin = 4;
		struct spi_cs_control cs_control = cfg->spi.config.cs;
		cs_control.gpio.pin = second_pin;
		set_flash(dev, 1);
		ret = spi_configure(dev, cfg);
		set_flash(dev, 0); 
	}
	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

static void spi_nor_pages_layout(const struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	/* Data for runtime, const for devicetree and minimal. */
#ifdef CONFIG_SPI_NOR_SFDP_RUNTIME
	const struct spi_nor_data *data = dev->data;

	*layout = &data->layout;
#else /* CONFIG_SPI_NOR_SFDP_RUNTIME */
	const struct spi_flash_config *cfg = dev->config;

	*layout = &cfg->layout;
#endif /* CONFIG_SPI_NOR_SFDP_RUNTIME */

	*layout_size = 1;
}

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_nor_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_nor_parameters;
}


