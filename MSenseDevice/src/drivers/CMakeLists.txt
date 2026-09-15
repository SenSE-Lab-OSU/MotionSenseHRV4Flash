zephyr_library()

zephyr_library_sources_ifdef(CONFIG_QSPI_NOR_CUSTOM_DRIVER jdec_nor/custom_qspi.c)
zephyr_library_sources_ifdef(CONFIG_DISK_DRIVER_RAW_NAND nand/nand_disk.c)

# the chip level driver and the bad block table are shared by both translation
# layers: dhara's nand callbacks are built on top of the same multi_nand_* calls.
if(CONFIG_DISK_DRIVER_RAW_NAND OR CONFIG_DISK_DRIVER_DHARA)
  zephyr_library_sources(nand/spi_nand.c nand/bad_page.c)
endif()
zephyr_library_sources_ifdef(CONFIG_SPI_NOR_CUSTOM_DRIVER jdec_nor/spi_nor.c)
zephyr_library_sources_ifdef(CONFIG_DISK_DRIVER_DHARA dhara/error.c dhara/journal.c dhara/map.c dhara/nand.c dhara/dhara_disk.c)
