#include "jtag.h"

static int tick(const struct vme_io *io, int tms, int tdi, int *tdo)
{
	if (io->write(VME_TMS, tms) < 0 || io->write(VME_TDI, tdi) < 0) return -1;
	io->delay_us(5);
	int value = io->read();
	if (value < 0) return -1;
	if (tdo) *tdo = value;
	if (io->write(VME_TCK, 1) < 0) return -1;
	io->delay_us(5);
	return io->write(VME_TCK, 0);
}

static int reset(const struct vme_io *io)
{
	if (io->write(VME_TCK, 0) < 0) return -1;
	for (int i = 0; i < 6; i++) if (tick(io, 1, 1, NULL) < 0) return -1;
	return tick(io, 0, 1, NULL);
}
static int enter_shift(const struct vme_io *io, int ir)
{
	if (tick(io, 1, 0, NULL) < 0) return -1;
	if (ir && tick(io, 1, 0, NULL) < 0) return -1;
	if (tick(io, 0, 0, NULL) < 0) return -1;
	return tick(io, 0, 0, NULL);
}
static int finish(const struct vme_io *io)
{
	if (tick(io, 1, 0, NULL) < 0) return -1;
	return tick(io, 0, 0, NULL);
}

int jtag_identify(const struct vme_io *io, uint32_t *id)
{
	uint32_t value = 0;
	int bit;
	if (reset(io) < 0 || enter_shift(io, 0) < 0) return VME_IO_ERROR;
	for (int i = 0; i < 32; i++) {
		if (tick(io, i == 31, 0, &bit) < 0) return VME_IO_ERROR;
		value |= (uint32_t)bit << i;
	}
	*id = value;
	if (finish(io) < 0) return VME_IO_ERROR;
	if (value != 0x012b8043) { reset(io); return VME_POLICY; }
	if (enter_shift(io, 1) < 0) return VME_IO_ERROR;
	unsigned capture = 0;
	for (int i = 0; i < 8; i++) {
		if (tick(io, i == 7, 1, &bit) < 0) return VME_IO_ERROR;
		capture |= (unsigned)bit << i;
	}
	if (finish(io) < 0 || enter_shift(io, 0) < 0) return VME_IO_ERROR;
	/* BYPASS must be exactly one bit, not an extra device or a stuck wire. */
	uint32_t pattern = 0xa5c3695a;
	int previous = 0, bad = (capture & 3) != 1;
	for (int i = 0; i < 64; i++) {
		int input = (pattern >> (i % 32)) & 1;
		if (tick(io, i == 63, input, &bit) < 0) return VME_IO_ERROR;
		bad |= bit != previous;
		previous = input;
	}
	if (finish(io) < 0 || reset(io) < 0) return VME_IO_ERROR;
	return bad ? VME_POLICY : 0;
}
