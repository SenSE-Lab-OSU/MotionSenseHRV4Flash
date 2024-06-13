/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NOR_H__
#define __SPI_NOR_H__

#include <zephyr/sys/util.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/flash.h>


//This is the raw nand driver, which does not use any FTL or heap (sram caching.) partial programs are limited to 4.


#define SPI_MAX_ID_LEN	3

#define SPI_BLOCK_COUNT 4

//Page Program
#define SPI_NAND_PL 0x02
#define SPI_NAND_PE 0x10
#define SPI_NAND_GF 0x0F
#define SPI_NAND_CMD_RDID 0x9F
#define SPI_NAND_PAGE_READ 0x13
#define SPI_NAND_RESET 0xFF

//Configuration Registers 
#define REGISTER_BLOCKLOCK 0xA0
#define REGISTER_CONFIGURATION 0xB0
#define REGISTER_STATUS 0xC0
#define REGISTER_DIESELECT 0xD0

#define FLASH_DIE_ZERO 0x00
#define FLASH_DIE_ONE 0x40

//SHARED COMMANDS: BLOCK ERASE, WRITE ENABLE, WRITE DISABLE, PAGE_TRANSFER(qspi nor is PAGE READ), 


/* Status register bits */
#define SPI_NOR_WIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NOR_WEL_BIT         BIT(1)  /* Write enable latch */

/* Flash opcodes */
#define SPI_NOR_CMD_WRSR        0x01    /* Write status register */
#define SPI_NOR_CMD_RDSR        0x05    /* Read status register */
#define SPI_NOR_CMD_WRSR2       0x31    /* Write status register 2 */
#define SPI_NOR_CMD_RDSR2       0x35    /* Read status register 2 */
#define SPI_NOR_CMD_RDSR3       0x15    /* Read status register 3 */
#define SPI_NOR_CMD_WRSR3       0x11    /* Write status register 3 */
#define SPI_NOR_CMD_READ        0x03    /* Read data */
#define SPI_NOR_CMD_READ_FAST   0x0B    /* Read data */
#define SPI_NOR_CMD_DREAD       0x3B    /* Read data (1-1-2) */
#define SPI_NOR_CMD_2READ       0xBB    /* Read data (1-2-2) */
#define SPI_NOR_CMD_QREAD       0x6B    /* Read data (1-1-4) */
#define SPI_NOR_CMD_4READ       0xEB    /* Read data (1-4-4) */
#define SPI_NOR_CMD_WREN        0x06    /* Write enable */
#define SPI_NOR_CMD_WRDI        0x04    /* Write disable */
#define SPI_NOR_CMD_PP          0x02    /* Page program */
#define SPI_NOR_CMD_PP_1_1_2    0xA2    /* Dual Page program (1-1-2) */
#define SPI_NOR_CMD_PP_1_1_4    0x32    /* Quad Page program (1-1-4) */
#define SPI_NOR_CMD_PP_1_4_4    0x38    /* Quad Page program (1-4-4) */
#define SPI_NOR_CMD_RDCR        0x15    /* Read control register */
#define SPI_NOR_CMD_SE          0x20    /* Sector erase */
#define SPI_NOR_CMD_SE_4B       0x21    /* Sector erase 4 byte address*/
#define SPI_NOR_CMD_BE_32K      0x52    /* Block erase 32KB */
#define SPI_NOR_CMD_BE          0xD8    /* Block erase */
#define SPI_NOR_CMD_CE          0xC7    /* Chip erase */
#define SPI_NOR_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NOR_CMD_ULBPR       0x98    /* Global Block Protection Unlock */
#define SPI_NOR_CMD_4BA         0xB7    /* Enter 4-Byte Address Mode */
#define SPI_NOR_CMD_DPD         0xB9    /* Deep Power Down */
#define SPI_NOR_CMD_RDPD        0xAB    /* Release from Deep Power Down */
#define SPI_NOR_CMD_WR_CFGREG2  0x72    /* Write config register 2 */
#define SPI_NOR_CMD_RD_CFGREG2  0x71    /* Read config register 2 */
#define SPI_NOR_CMD_RESET_EN    0x66    /* Reset Enable */
#define SPI_NOR_CMD_RESET_MEM   0x99    /* Reset Memory */
#define SPI_NOR_CMD_BULKE       0x60    /* Bulk Erase */
#define SPI_NOR_CMD_READ_4B      0x13  /* Read data 4 Byte Address */
#define SPI_NOR_CMD_READ_FAST_4B 0x0C  /* Fast Read 4 Byte Address */
#define SPI_NOR_CMD_DREAD_4B     0x3C  /* Read data (1-1-2) 4 Byte Address */
#define SPI_NOR_CMD_2READ_4B     0xBC  /* Read data (1-2-2) 4 Byte Address */
#define SPI_NOR_CMD_QREAD_4B     0x6C  /* Read data (1-1-4) 4 Byte Address */
#define SPI_NOR_CMD_4READ_4B     0xEC  /* Read data (1-4-4) 4 Byte Address */
#define SPI_NOR_CMD_PP_4B        0x12  /* Page Program 4 Byte Address */
#define SPI_NOR_CMD_PP_1_1_4_4B  0x34  /* Quad Page program (1-1-4) 4 Byte Address */
#define SPI_NOR_CMD_PP_1_4_4_4B  0x3e  /* Quad Page program (1-4-4) 4 Byte Address */

/* Flash octal opcodes */
#define SPI_NOR_OCMD_SE         0x21DE  /* Octal Sector erase */
#define SPI_NOR_OCMD_CE         0xC738  /* Octal Chip erase */
#define SPI_NOR_OCMD_RDSR       0x05FA  /* Octal Read status register */
#define SPI_NOR_OCMD_DTR_RD     0xEE11  /* Octal IO DTR read command */
#define SPI_NOR_OCMD_RD         0xEC13  /* Octal IO read command */
#define SPI_NOR_OCMD_PAGE_PRG   0x12ED  /* Octal Page Prog */
#define SPI_NOR_OCMD_WREN       0x06F9  /* Octal Write enable */
#define SPI_NOR_OCMD_NOP        0x00FF  /* Octal No operation */
#define SPI_NOR_OCMD_RESET_EN   0x6699  /* Octal Reset Enable */
#define SPI_NOR_OCMD_RESET_MEM  0x9966  /* Octal Reset Memory */
#define SPI_NOR_OCMD_WR_CFGREG2 0x728D  /* Octal Write configuration Register2 */
#define SPI_NOR_OCMD_RD_CFGREG2 0x718E  /* Octal Read configuration Register2 */
#define SPI_NOR_OCMD_BULKE      0x609F  /* Octa Bulk Erase */

 /* Page, sector, and block size are standard, not configurable. */
 #define SPI_NOR_PAGE_SIZE    0x0100U
 #define SPI_NOR_SECTOR_SIZE  0x1000U
 #define SPI_NOR_BLOCK_SIZE   0x10000U

/* Flash Auto-polling values */
#define SPI_NOR_WREN_MATCH    0x02
#define SPI_NOR_WREN_MASK     0x02

#define SPI_NOR_WEL_MATCH     0x00
#define SPI_NOR_WEL_MASK      0x02

#define SPI_NOR_MEM_RDY_MATCH 0x00
#define SPI_NOR_MEM_RDY_MASK  0x01

#define SPI_NOR_AUTO_POLLING_INTERVAL   0x10

/* Flash Dummy Cycles values */
#define SPI_NOR_DUMMY_RD                8U
#define SPI_NOR_DUMMY_RD_OCTAL          6U
#define SPI_NOR_DUMMY_RD_OCTAL_DTR      6U
#define SPI_NOR_DUMMY_REG_OCTAL         4U
#define SPI_NOR_DUMMY_REG_OCTAL_DTR     5U


/* Memory registers address */
#define SPI_NOR_REG2_ADDR1              0x0000000
#define SPI_NOR_CR2_STR_OPI_EN          0x01
#define SPI_NOR_CR2_DTR_OPI_EN          0x02
#define SPI_NOR_REG2_ADDR3              0x00000300
#define SPI_NOR_CR2_DUMMY_CYCLES_66MHZ  0x07

/* Test whether offset is aligned to a given number of bits. */
#define SPI_NOR_IS_ALIGNED(_ofs, _bits) (((_ofs) & BIT_MASK(_bits)) == 0)
#define SPI_NOR_IS_SECTOR_ALIGNED(_ofs) SPI_NOR_IS_ALIGNED(_ofs, 12)
#define SPI_NOR_IS_32K_ALIGNED(_ofs) SPI_NOR_IS_ALIGNED(_ofs, 15)
#define SPI_NOR_IS_64K_ALIGNED(_ofs) SPI_NOR_IS_ALIGNED(_ofs, 16)





/* Build-time data associated with the device. */
struct spi_flash_config {
	/* Devicetree SPI configuration */
	struct spi_dt_spec spi;

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	const struct gpio_dt_spec reset;
#endif

	/* Runtime SFDP stores no static configuration. */

#ifndef CONFIG_SPI_NOR_SFDP_RUNTIME
	/* Size of device in bytes, from size property */
	uint32_t flash_size;

#ifdef CONFIG_FLASH_PAGE_LAYOUT
	/* Flash page layout can be determined from devicetree. */
	struct flash_pages_layout layout;
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	/* Expected JEDEC ID, from jedec-id property */
	uint8_t jedec_id[SPI_MAX_ID_LEN];

#if defined(CONFIG_SPI_NOR_SFDP_MINIMAL)
	/* Optional support for entering 32-bit address mode. */
	uint8_t enter_4byte_addr;
#endif /* CONFIG_SPI_NOR_SFDP_MINIMAL */

#if defined(CONFIG_SPI_NOR_SFDP_DEVICETREE)
	/* Length of BFP structure, in 32-bit words. */
	uint8_t bfp_len;

	/* Pointer to the BFP table as read from the device
	 * (little-endian stored words), from sfdp-bfp property
	 */
	const struct jesd216_bfp *bfp;
#endif /* CONFIG_SPI_NOR_SFDP_DEVICETREE */
#endif /* CONFIG_SPI_NOR_SFDP_RUNTIME */

	/* Optional bits in SR to be cleared on startup.
	 *
	 * This information cannot be derived from SFDP.
	 */
	uint8_t has_lock;
};

typedef struct spi_send_request {

	uint8_t opcode;

	bool is_write;
	void* addr;
	size_t addr_length; 
	void* data;
	size_t data_length;

} spi_send_request;

/**
 * struct spi_nor_data - Structure for defining the SPI NOR access
 * @sem: The semaphore to access to the flash
 */
struct spi_nor_data {
	struct k_sem sem;

	struct k_sem sem_inner;
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	/* Low 32-bits of uptime counter at which device last entered
	 * deep power-down.
	 */
	uint32_t ts_enter_dpd;
#endif

	/* Miscellaneous flags */

	/* If set addressed operations should use 32-bit rather than
	 * 24-bit addresses.
	 *
	 * This is ignored if the access parameter to a command
	 * explicitly specifies 24-bit or 32-bit addressing.
	 */
	bool flag_access_32bit: 1;

	/* Minimal SFDP stores no dynamic configuration.  Runtime and
	 * devicetree store page size and erase_types; runtime also
	 * stores flash size and layout.
	 */
#ifndef CONFIG_SPI_NOR_SFDP_MINIMAL

	/* Number of bytes per page */
	uint16_t page_size;

#ifdef CONFIG_SPI_NOR_SFDP_RUNTIME
	/* Size of flash, in bytes */
	uint32_t flash_size;

#ifdef CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout layout;
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#endif /* CONFIG_SPI_NOR_SFDP_RUNTIME */
#endif /* CONFIG_SPI_NOR_SFDP_MINIMAL */
};

uint16_t dev_page_size(const struct device *dev);

uint32_t dev_flash_size(const struct device* dev);

int spi_flash_wait_until_ready(const struct device *dev);

off_t convert_to_address(uint32_t page, uint32_t block);

off_t convert_page_to_address(const struct device* dev, uint32_t page);

off_t convert_block_to_address(uint32_t block);

uint8_t get_features(const struct device* dev, uint8_t register_select);

int set_features(const struct device* dev, uint8_t register_select, uint8_t data);


/* Set the Die. if 0, use the first die, if 1, use the second. */
int set_die(const struct device* dev, int die);

int set_flash(const struct device* dev, int flash_id);

uint8_t spi_rdsr(const struct device *dev);

int spi_nor_wrsr(const struct device *dev,
			uint8_t sr);

int detect_bad_blocks(const struct device* dev);

int spi_nand_parameter_page_read(const struct device* dev, void* dest);

int spi_nand_page_read(const struct device* dev, off_t page_addr, void* dest);

int spi_nand_page_write(const struct device* dev, off_t page_address, const void* src, size_t size);

int spi_nand_block_erase(const struct device * dev, off_t block_addr);

 
int spi_nand_chip_erase(const struct device* device);

int spi_nand_whole_chip_erase(const struct device* dev);

int spi_nand_multi_chip_erase(const struct device* dev);

int spi_init(const struct device *dev);

#endif /*__SPI_NOR_H__*/
