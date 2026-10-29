#include "jtag.h"
#include <assert.h>
#include <stdio.h>

static unsigned state, levels, instruction, extra, fault;
static uint64_t reg;
static uint32_t chip_id;
static const unsigned next[16][2] = {
	{1,0},{1,2},{3,9},{4,5},{4,5},{6,8},{6,7},{4,8},
	{1,2},{10,0},{11,12},{11,12},{13,15},{13,14},{11,15},{1,2}
};
static int write_pin(unsigned char pins, unsigned char value)
{
	if (fault) return -1;
	unsigned before = levels;
	if (value) levels |= pins; else levels &= ~pins;
	if (!(before & VME_TCK) && (levels & VME_TCK)) {
		unsigned tdi = !!(levels & VME_TDI);
		if (state == 4) {
			unsigned bits = instruction == 0xe0 ? 32 : 1 + extra;
			reg = (reg >> 1) | ((uint64_t)tdi << (bits - 1));
		}
		if (state == 11) reg = (reg >> 1) | (tdi << 7);
		state = next[state][!!(levels & VME_TMS)];
		if (state == 0) instruction = 0xe0;
		if (state == 3) reg = instruction == 0xe0 ? chip_id : 0;
		if (state == 10) reg = 1;
		if (state == 15) instruction = reg & 255;
	}
	return 0;
}
static int read_pin(void) { return reg & 1; }
static void delay_us(uint32_t us) { assert(us >= 5); }
static const struct vme_io io = {.write=write_pin, .read=read_pin, .delay_us=delay_us};
int main(void)
{
	uint32_t id;
	chip_id = 0x012b8043;
	assert(jtag_identify(&io, &id) == 0 && id == chip_id);
	extra = 1;
	assert(jtag_identify(&io, &id) == VME_POLICY);
	extra = 0; chip_id = 0xffffffff;
	assert(jtag_identify(&io, &id) == VME_POLICY);
	chip_id = 0;
	assert(jtag_identify(&io, &id) == VME_POLICY);
	fault = 1;
	assert(jtag_identify(&io, &id) == VME_IO_ERROR);
	puts("PASS: correct ID, extra bypass device, stuck-high/low TDO, GPIO failure");
}
