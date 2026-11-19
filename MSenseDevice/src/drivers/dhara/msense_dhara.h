#ifndef MSENSE_DHARA_H_
#define MSENSE_DHARA_H_

#include <zephyr/device.h>
#include "nand.h"

/* dhara_nand is meant to be embedded in a larger structure for context (see the
 * comment on the struct in nand.h). Ours carries the zephyr device that the four
 * chained SPI NAND chips hang off, so the dhara_nand_* callbacks can recover it
 * from the pointer dhara hands them.
 *
 * base must stay first: nand_dev() casts straight through it.
 */
struct msense_dhara_nand {
	dhara_nand base;
	const struct device *dev;
};

/* Fills in the geometry from the device and binds it to the context. Returns the
 * dhara_nand to hand to dhara_map_init(). Call after spi_init() has run, since the
 * bad block table has to be loaded before dhara starts asking about blocks.
 */
dhara_nand *msense_dhara_nand_init(const struct device *dev);

#endif /* MSENSE_DHARA_H_ */
