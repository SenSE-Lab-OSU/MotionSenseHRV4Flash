#pragma once
#include <stdint.h>

int fpga_spi_init(void);
int fpga_spi_whoami(uint8_t rx[3]);
