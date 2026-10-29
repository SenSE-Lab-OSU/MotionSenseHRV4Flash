/* Board adapter for Lattice ispVM Embedded. Vendor notices remain in ispvm/. */
#include "vme.h"
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(__ZEPHYR__) && defined(VME_OFFLINE_AUDIT)
#error "Offline audit is forbidden in firmware"
#endif
#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#define alloc(n) k_calloc(1, (n))
#define release(p) k_free(p)
#else
#define alloc(n) calloc(1, (n))
#define release(p) free(p)
#endif

static void scan_begin(void);
static void scan_end(int result);
static void fail(int code);
static void compare_failed(unsigned bit, unsigned expected, unsigned actual);
static void poll_exhausted(unsigned attempts);
/* Keep the full interpreter and its TAP/scan/polling algorithms together. */
#pragma push_macro("MAX")
#undef MAX
#include "ispvm/ivm_core.c"
#pragma pop_macro("MAX")

_Static_assert(CHAR_BIT == 8 && sizeof(short) == 2 && sizeof(int) == 4,
	       "ispVM requires 8-bit bytes, 16-bit short, 32-bit int");
/* unsigned long is only used by the vendor checksum accumulator, masked to 16 bits. */

const unsigned char g_ucPinTDI = VME_TDI, g_ucPinTCK = VME_TCK;
const unsigned char g_ucPinTMS = VME_TMS, g_ucPinENABLE = 8;
const unsigned char g_ucPinTRST = 16, g_ucPinTDO = 64;
static const struct vme_io *port;
static const uint8_t *image;
static size_t image_size, cursor;
static jmp_buf escape;
static uint64_t deadline;
static unsigned scans, comparisons;
static unsigned first_bad, expected_bit, actual_bit;
static uint8_t instruction;
static int used;
static struct { void *ptr; size_t size; } buffers[11];

static void fail(int code) { longjmp(escape, -code); }
static void checkpoint(void)
{
	if (port->now_ms() > deadline) fail(VME_TIMEOUT);
}

void writePort(unsigned char pins, unsigned char value)
{
	checkpoint();
	if (pins & ~(VME_TDI | VME_TCK | VME_TMS)) fail(VME_POLICY);
	if (port->write(pins, value) < 0) fail(VME_IO_ERROR);
}

unsigned char readPort(void)
{
	checkpoint();
	int value = port->read();
	if (value < 0 || value > 1) fail(VME_IO_ERROR);
	return (unsigned char)value;
}

void sclock(void)
{
	/* Conservative <=100 kHz, including minimum setup/hold on both phases. */
	port->delay_us(5);
	writePort(g_ucPinTCK, 1);
	port->delay_us(5);
	writePort(g_ucPinTCK, 0);
}

void ispVMDelay(unsigned short delay)
{
	checkpoint();
	port->delay_us((delay & 0x8000) ? (uint32_t)(delay & 0x7fff) * 1000 : delay);
	checkpoint();
}

/* Vendor file comments are informational; keep programming output concise. */
void vme_out_char(unsigned char c) { (void)c; }
void vme_out_hex(unsigned char c) { char s[3]; snprintf(s, sizeof(s), "%02X", c); port->message(s); }
void vme_out_string(char *s) { port->message(s); }

unsigned char GetByte(void)
{
	checkpoint();
	if (g_usDataType & HEAP_IN) {
		if (g_iHeapCounter >= g_iHEAPSize) fail(VME_FILE_READ_FAILURE);
		return g_pucHeapMemory[g_iHeapCounter++];
	}
	if (g_usDataType & LHEAP_IN) {
		if (g_usIntelDataIndex >= g_usIntelBufferSize) fail(VME_FILE_READ_FAILURE);
		return g_pucIntelBuffer[g_usIntelDataIndex++];
	}
	if (cursor >= image_size) fail(VME_FILE_READ_FAILURE);
	return image[cursor++];
}

static void *buffer(unsigned slot, size_t size)
{
	if (size > 65537) fail(VME_MEMORY);
	if (buffers[slot].size < size) {
		void *p = alloc(size);
		if (!p) fail(VME_MEMORY);
		if (buffers[slot].ptr) memcpy(p, buffers[slot].ptr, buffers[slot].size);
		release(buffers[slot].ptr);
		buffers[slot].ptr = p;
		buffers[slot].size = size;
	}
	return buffers[slot].ptr;
}

void ispVMMemManager(signed char target, unsigned short size)
{
	size_t bytes = (size_t)size / 8 + 2;
	switch (target) {
	case XTDI: case TDI:
		g_pucInData = buffer(0, bytes);
		/* Vendor TDI allocation also provisions the next expected-data buffer. */
		g_pucOutData = buffer(1, bytes); break;
	case XTDO: case TDO: g_pucOutData = buffer(1, bytes); break;
	case MASK: g_pucOutMaskData = buffer(2, bytes); break;
	case DMASK: g_pucOutDMaskData = buffer(3, bytes); break;
	case HIR: g_pucHIRData = buffer(4, bytes); break;
	case TIR: g_pucTIRData = buffer(5, bytes); break;
	case HDR: g_pucHDRData = buffer(6, bytes); break;
	case TDR: g_pucTDRData = buffer(7, bytes); break;
	case HEAP: g_pucHeapMemory = buffer(8, (size_t)size + 2); break;
	case LHEAP: g_pucIntelBuffer = buffer(9, (size_t)size + 2); break;
	case LVDS: g_pLVDSList = buffer(10, (size_t)size * sizeof(LVDSPair)); break;
	default: break; /* Vendor calls this for non-allocation data tokens too. */
	}
}

static uint8_t reverse8(uint8_t value)
{
	uint8_t out = 0;
	for (int i = 0; i < 8; i++) { out = (out << 1) | (value & 1); value >>= 1; }
	return out;
}

static void scan_begin(void)
{
	char line[96];
	first_bad = UINT_MAX;
	if (!g_usiDataSize || g_usiDataSize > SCANMAX ||
	    (g_usFlowControl & VERIFYUES) || (g_usDataType & (CMASK_DATA | DMASK_DATA)))
		fail(VME_POLICY);
	if (g_usDataType & SIR_DATA) {
		if (g_usiDataSize != 8 || !(g_usDataType & TDI_DATA)) fail(VME_POLICY);
		instruction = reverse8(g_pucInData[0]);
		/* Security/OTP/feature programming are never silently admitted. */
		switch (instruction) {
		case 0xe0: case 0x1c: case 0x3c: case 0xc6: case 0x0e:
		case 0xff: case 0xf0: case 0x46: case 0x70: case 0x73:
		case 0x5e: case 0x26: case 0xc0: case 0xc2:
		case 0xe4: case 0xe7: case 0xf8: case 0xfb:
			break;
		default:
#ifndef VME_OFFLINE_AUDIT
			snprintf(line, sizeof(line), "Blocked unreviewed JTAG instruction 0x%02x\n", instruction);
			port->message(line); fail(VME_POLICY);
#endif
			break;
		}
	}
	/* Match reviewed erase/feature payloads; never broaden an erase target. */
	if (g_usDataType & SDR_DATA) {
		const uint8_t *d = g_pucInData;
		if (!(g_usDataType & TDI_DATA)) fail(VME_POLICY);
		if (instruction == 0x0e && (g_usiDataSize != 8 ||
		    (reverse8(d[0]) != 1 && reverse8(d[0]) != 14))) fail(VME_POLICY);
		if (instruction == 0xe4 && (g_usiDataSize != 64 ||
		    memcmp(d, (uint8_t[8]){0}, 8))) fail(VME_POLICY);
		if (instruction == 0xf8 && (g_usiDataSize != 16 ||
		    reverse8(d[0]) != 0x20 || reverse8(d[1]) != 0x06)) fail(VME_POLICY);
	}
	++scans;
	if (g_usDataType & TDO_DATA) ++comparisons;
	if (port->scan) port->scan(!!(g_usDataType & SIR_DATA), g_usiDataSize,
		g_usDataType & TDI_DATA ? g_pucInData : NULL,
		g_usDataType & TDO_DATA ? g_pucOutData : NULL,
		g_usDataType & MASK_DATA ? g_pucOutMaskData : NULL);
}

static void scan_end(int result)
{
	/* A busy response inside the VME polling loop is expected; exhaustion
	 * remains an error reported by poll_exhausted(). */
	if (result < 0 && !(instruction == 0xf0 && (g_usDataType & LHEAP_IN))) {
		char line[160];
		snprintf(line, sizeof(line), "TDO mismatch scan=%u IR=%02x bits=%u first-bit=%u expected=%u actual=%u%s\n",
			 scans, instruction, g_usiDataSize, first_bad, expected_bit, actual_bit,
			 g_usDataType & LHEAP_IN ? " (poll may retry)" : "");
		port->message(line);
	}
}

static void compare_failed(unsigned bit, unsigned expected, unsigned actual)
{
	if (first_bad == UINT_MAX) {
		first_bad = bit; expected_bit = expected; actual_bit = actual;
	}
}

static void poll_exhausted(unsigned attempts)
{
	char line[80];
	snprintf(line, sizeof(line), "Polling retry limit exhausted: %u attempts, IR=%02x\n", attempts, instruction);
	port->message(line);
}

int vme_check(const uint8_t *data, size_t size)
{
	if (size < 13 || data[0] != FILE_CRC) return VME_INVALID_FILE;
	if (memcmp(data + 3, "____12.1", 8)) return VME_VERSION_FAILURE;
	if (data[11] != 0xf2) return VME_POLICY; /* Reviewed uncompressed Full VME. */
	g_usCalculatedCRC = 0;
	for (size_t i = 3; i < size; ++i) ispVMCalculateCRC32(data[i]);
	if (g_usCalculatedCRC != ((unsigned)data[1] << 8 | data[2])) return VME_CRC_FAILURE;
	return 0;
}

int vme_run(const uint8_t *data, size_t size, const struct vme_io *io)
{
	int result = vme_check(data, size);
	if (result) return result;
	if (used) return VME_ALREADY_RUN;
	used = 1;
	port = io; image = data; image_size = size; cursor = 11;
	deadline = port->now_ms() + 180000;
	int trapped = setjmp(escape);
	if (!trapped) {
		ispVMStart();
		result = ispVMCode();
		if (result == 0 && (cursor != size || !comparisons)) result = VME_INVALID_FILE;
		ispVMEnd();
	} else {
		result = -trapped;
		/* Do not issue cleanup programming commands after a failure. */
		port->write(VME_TCK, 0);
		port->write(VME_TMS | VME_TDI, 1);
	}
	for (unsigned i = 0; i < 11; i++) release(buffers[i].ptr);
	if (result) {
		char line[96];
		snprintf(line, sizeof(line), "VME failed: result=%d scan=%u file=%u/%u IR=%02x\n",
			 result, scans, (unsigned)cursor, (unsigned)image_size, instruction);
		port->message(line);
	}
	return result;
}
