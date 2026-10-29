#ifndef BRIDGE_JTAG_H
#define BRIDGE_JTAG_H
#include "vme.h"
int jtag_identify(const struct vme_io *io, uint32_t *id);
#endif
