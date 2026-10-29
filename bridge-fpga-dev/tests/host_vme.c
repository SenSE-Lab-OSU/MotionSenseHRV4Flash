/* Offline simulation only: expected TDO is injected, never hardware evidence. */
#include "vme.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t *expected;
static unsigned bit, scan_no;
static unsigned fail_scan;
static uint64_t elapsed;
static unsigned char ir;
static FILE *trace;
static unsigned mode, clocks, shifted, scan_bits, levels;
static const uint8_t *input;
static int transport_bad;
static int busy_once;
static unsigned state;
/* IEEE 1149.1 TAP transitions, indexed by current state and TMS. */
static const unsigned next[16][2] = {
	{1,0},{1,2},{3,9},{4,5},{4,5},{6,8},{6,7},{4,8},
	{1,2},{10,0},{11,12},{11,12},{13,15},{13,14},{11,15},{1,2}
};
static int write_pin(unsigned char pins, unsigned char value)
{
	if (mode == 3) return -1;
	unsigned before = levels;
	if (value) levels |= pins; else levels &= ~pins;
	if (!(before & VME_TCK) && (levels & VME_TCK)) {
		++clocks;
		if ((state == 4 || state == 11) && shifted < scan_bits) {
			int want = input ? !!(input[shifted / 8] & (0x80 >> (shifted % 8))) : 0;
			transport_bad |= want != !!(levels & VME_TDI);
			transport_bad |= (!!(levels & VME_TMS)) != (shifted + 1 == scan_bits);
			++shifted;
		}
		state = next[state][!!(levels & VME_TMS)];
	}
	return 0;
}
static int read_pin(void)
{
	int value = expected ? !!(expected[bit / 8] & (0x80 >> (bit % 8))) : 0;
	if (mode == 2 && ir == 0xf0) value = 1;
	if (mode == 6 && ir == 0xf0 && !busy_once) { value = 1; busy_once = 1; }
	if (mode == 5 && bit == 7) value = !value;
	++bit;
	return scan_no == fail_scan ? !value : value;
}
static void delay_us(uint32_t us) { elapsed += us; }
static uint64_t now_ms(void) { return mode == 4 && clocks > 50 ? 200000 : elapsed / 1000; }
static void message(const char *s) { fputs(s, stdout); }
static unsigned reverse(unsigned v)
{
	unsigned out = 0;
	for (unsigned i = 0; i < 8; i++) { out = (out << 1) | (v & 1); v >>= 1; }
	return out;
}
static void scan(int is_ir, unsigned bits, const uint8_t *tdi,
		 const uint8_t *tdo, const uint8_t *mask)
{
	if (shifted != scan_bits) transport_bad = 1;
	expected = tdo; bit = 0; ++scan_no;
	input = tdi; shifted = 0; scan_bits = bits;
	transport_bad |= state != (is_ir ? 11 : 4);
	if (is_ir && tdi) ir = reverse(tdi[0]);
	fprintf(trace, "%u %s IR=%02X bits=%u", scan_no, is_ir ? "SIR" : "SDR", ir, bits);
	const uint8_t *fields[] = {tdi, tdo, mask};
	const char *names[] = {" TDI=", " TDO=", " MASK="};
	for (int f = 0; f < 3; f++) if (fields[f]) {
		fputs(names[f], trace);
		for (unsigned b = 0; b < (bits + 7) / 8; b++) fprintf(trace, "%02X", reverse(fields[f][b]));
	}
	fputc('\n', trace);
}
int main(int argc, char **argv)
{
	if (argc < 3) return 2;
	FILE *file = fopen(argv[1], "rb");
	if (!file) return 2;
	fseek(file, 0, SEEK_END); long len = ftell(file); rewind(file);
	uint8_t *data = malloc(len);
	if (!data || fread(data, 1, len, file) != (size_t)len) return 2;
	fclose(file);
	trace = fopen(argv[2], "w"); if (!trace) return 2;
	if (argc > 3) fail_scan = strtoul(argv[3], NULL, 0);
	if (argc > 4) mode = strtoul(argv[4], NULL, 0);
	const struct vme_io io = {write_pin, read_pin, delay_us, now_ms, message, scan};
	int result = vme_run(data, len, &io);
	if (shifted != scan_bits && !result) transport_bad = 1;
	printf("TRANSPORT bit-order/final-TMS/state=%s clocks=%u\n", transport_bad ? "FAIL" : "PASS", clocks);
	printf("SIMULATION ONLY result=%d elapsed_us=%llu\n", result, (unsigned long long)elapsed);
	fclose(trace); free(data);
	return result || transport_bad ? 1 : 0;
}
