zephyr_library()

#zephyr_library_sources(jdec_nor/custom_qspi.c)
zephyr_library_sources_ifdef(CONFIG_QSPI_NOR_CUSTOM_DRIVER jdec_nor/custom_qspi.c)
