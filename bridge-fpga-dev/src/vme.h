#ifndef BRIDGE_VME_H
#define BRIDGE_VME_H

#include <stddef.h>
#include <stdint.h>

/* Single-threaded, one programming attempt per boot. No automatic retry. */
struct vme_io {
	int (*write)(unsigned char pins, unsigned char value);
	int (*read)(void);
	void (*delay_us)(uint32_t us);
	uint64_t (*now_ms)(void);
	void (*message)(const char *text);
	/* Optional trace observer, used by host tests; never substitutes real TDO. */
	void (*scan)(int ir, unsigned bits, const uint8_t *tdi,
		     const uint8_t *tdo, const uint8_t *mask);
};

enum { VME_TDI = 1, VME_TCK = 2, VME_TMS = 4 };
enum { VME_IO_ERROR = -20, VME_TIMEOUT = -21, VME_MEMORY = -22,
	VME_POLICY = -23, VME_ALREADY_RUN = -24 };

int vme_check(const uint8_t *data, size_t size);
int vme_run(const uint8_t *data, size_t size, const struct vme_io *io);

#endif
