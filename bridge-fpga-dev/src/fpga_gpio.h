#ifndef FPGA_GPIO_H
#define FPGA_GPIO_H
#include "vme.h"
int fpga_gpio_init(void);
int fpga_done(void);
extern const struct vme_io fpga_io;
void fpga_message(const char *text);
#endif
