# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(include)

zephyr_library_sources(device_identity.c)
zephyr_library_sources_ifdef(CONFIG_USB_MASS_STORAGE msc_media.c)
zephyr_library_sources_ifdef(CONFIG_QSPI_NOR_CUSTOM_DRIVER drivers/nor/custom_qspi.c)
zephyr_library_sources_ifdef(CONFIG_DISK_DRIVER_RAW_NAND drivers/nand/bad_page.c)
zephyr_library_sources_ifdef(CONFIG_DISK_DRIVER_RAW_NAND
	drivers/nand/nand_disk.c
	drivers/nand/spi_nand.c
)
zephyr_library_sources_ifdef(CONFIG_SPI_NOR_CUSTOM_DRIVER drivers/nor/spi_nor.c)
zephyr_library_sources_ifdef(CONFIG_LOG_BACKEND_FS_BUFFER storage_log_backend.c)
zephyr_library_sources_ifdef(CONFIG_DISK_DRIVER_DHARA
	drivers/dhara/error.c
	drivers/dhara/journal.c
	drivers/dhara/map.c
	drivers/dhara/nand.c
)
