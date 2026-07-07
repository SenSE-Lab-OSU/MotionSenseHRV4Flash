#ifndef MAX30001_H
#define MAX30001_H

#include <stdint.h>

/*
 * Probes the MAX30001 SPI interface by issuing a NO_OP transaction followed by
 * an INFO register read. Returns 0 when the INFO fixed pattern is present.
 *
 * If info is non-NULL, the raw 24-bit INFO register value is stored there.
 */
int max30001_probe(uint32_t *info);

#endif /* MAX30001_H */
