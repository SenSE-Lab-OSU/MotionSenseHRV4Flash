# MAX30001 ECG block format (v1)

## Status

This document is the normative specification for the proposed version 1 ECG
block format. It is intended to be sufficient for independent firmware and
host implementations of:

- the MAX30001-to-block encoder;
- a decoder for ECG blocks read from a file;
- a decoder for the same ECG blocks transported over the sensor stream;
- file recovery after an interrupted recording; and
- validation and interoperability tests.

The words **must**, **must not**, **required**, **shall**, **shall not**,
**should**, **should not**, and **may** have their usual normative meanings.

## Intent

The format has the following goals, in priority order:

1. Preserve sample timing alignment exactly. The timestamp in each data block
   identifies the first payload sample, not the time at which firmware drained
   the FIFO, finalized the block, or wrote NAND.
2. Use the same self-contained 4,096-byte ECG data block on NAND and over the
   sensor stream. A block decoder must not need to know which transport
   delivered the block.
3. Match the physical 4,096-byte NAND page/sector size used by ECGv0.
4. Store each complete MAX30001 FIFO word in three bytes rather than storing a
   32-bit in-memory representation or a per-sample timestamped frame.
5. Detect corrupt, torn, misordered, duplicated, and missing blocks.
6. Keep steady-state encoder work bounded, allocation-free, and suitable for
   the nRF5340.
7. Keep FAT-specific metadata outside the common data-block format and outside
   stream payloads.

The format intentionally does not compress ECG values. Fixed-width packing is
predictable, inexpensive, random-access friendly, and robust in the presence
of corruption.

## Important hardware and timing facts

### Sample representation

The MAX30001 ECG FIFO word is 24 bits, not a 19-bit sample. It contains:

| Bits | Width | Meaning |
|---|---:|---|
| 23:6 | 18 | Signed two's-complement ECG voltage sample |
| 5:3 | 3 | ETAG, ECG FIFO status tag |
| 2:0 | 3 | PTAG, pace-event tag |

All 24 bits are preserved. There is no need for a separate tag byte, and a
decoder does not lose ETAG or PTAG information by using a three-byte sample.

### Common physical clock

The MAX30001 and nRF5340 RTC0 share the same physical 32.768 kHz clock. RTC0
runs at 512 Hz using a prescaler of 63. The MAX30001 is configured for a 512 Hz
ECG output rate. Consequently, one ECG FIFO sample interval is exactly one
RTC0 tick and there is no relative long-term clock drift to estimate.

This common-clock guarantee is a required property of the ECGv0 hardware and
is part of the timing model of this format. A different board that does not
share the clock must not claim conformance without defining a new timing
algorithm and format version.

### Timestamp event

The session timing anchor is the RTC0 tick captured at the first post-`SYNCH`
MAX30001 `SAMP` pulse. `SAMP` represents the instant at which the corresponding
filtered ECG output sample is placed in the FIFO.

Timestamps therefore describe filtered FIFO-output time. They do not describe
the analog electrode instant and do not compensate MAX30001 analog or digital
filter group delay.

FIFO drain latency, recorder-thread scheduling, CRC time, filesystem latency,
and NAND program latency must never be included in sample timestamps.

## Integer, byte-order, and arithmetic conventions

Unless a field explicitly says otherwise:

- Unsigned integers use ordinary binary representation.
- Multibyte metadata integers are little-endian.
- Magic values are literal ASCII bytes in the listed order.
- Reserved bytes are written as zero. A version 1 decoder requires them to be
  zero after CRC validation. A later format version may assign different
  semantics in its own decoder path.
- RTC tick and sample-index arithmetic is unsigned modulo `2^32`.
- A valid data block is exactly 4,096 bytes. A shorter or longer byte string is
  not a version 1 data block.
- Byte offsets are zero-based from the beginning of the structure being
  described.

Comparing wrapping 32-bit values must use modular arithmetic. For example, the
expected next tick is `(previous_first_tick + previous_sample_count) &
0xffffffff`.

## CRC-32 definition

Every version 1 structure uses CRC-32/ISO-HDLC, also commonly called standard
Ethernet, ZIP, zlib, or IEEE CRC-32:

| Parameter | Value |
|---|---|
| Width | 32 bits |
| Polynomial | `0x04C11DB7` |
| Reflected polynomial used by a right-shifting implementation | `0xEDB88320` |
| Initial register | `0xFFFFFFFF` |
| Reflect input | Yes |
| Reflect output | Yes |
| Final XOR | `0xFFFFFFFF` |
| Check value for ASCII `123456789` | `0xCBF43926` |

For every structure in this document:

1. Construct its complete encoded byte sequence at its required size.
2. Write four zero bytes into that structure's CRC field.
3. Compute CRC-32/ISO-HDLC over the entire structure, including magic,
   metadata, valid payload, unused payload capacity, and reserved bytes.
4. Store the resulting 32-bit value little-endian in the CRC field.

A decoder validates a structure by saving the encoded CRC, treating the CRC
field as four zero bytes during calculation, and comparing the calculated and
encoded values. It must not rely on the residue produced by calculating across
the nonzero encoded CRC field.

The intended firmware API in the managed NCS 2.9.3 workspace is:

```c
#include <zephyr/sys/crc.h>

uint32_t crc = crc32_ieee(bytes, length);
```

The nRF5340 has no general-purpose hardware CRC engine for arbitrary RAM
buffers. Its RADIO CRC is packet-specific and its internal CRC state is not
available to the CPU. The NCS 2.9.3 `crc32_ieee()` software implementation is
therefore the required baseline. A future implementation may transparently
use a conforming hardware implementation, but it must produce identical bytes.

CRC-16 is not used. Saving two bytes once per 4 KiB would be immaterial, while
CRC-32 gives substantially stronger accidental-corruption detection.

## Common 4,096-byte ECG data block

### Geometry

```text
block bytes             = 4096
block header bytes      = 20
payload capacity bytes  = 4074
sample bytes            = 3
samples per full block  = 1358
trailing padding bytes  = 2
```

The arithmetic is exact:

```text
20 + (1358 * 3) + 2 = 4096
```

### Layout

| Offset | Size | Field | Required value or meaning |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `ECB1`, bytes `45 43 42 31` |
| 4 | 4 | `first_rtc_tick` | RTC0 tick of payload sample zero |
| 8 | 4 | `first_sample_index` | Session-local index of payload sample zero |
| 12 | 2 | `sample_count` | Number of valid payload samples, `1..1358` |
| 14 | 1 | `sample_format` | `1`, meaning MAX30001 raw FIFO word, MSB first |
| 15 | 1 | `flags` | Version 1 block flags defined below |
| 16 | 4 | `block_crc32` | CRC-32 of all 4,096 bytes |
| 20 | 4,074 | `sample_payload` | Capacity for 1,358 consecutive 3-byte samples |
| 4,094 | 2 | `reserved_tail` | Zero |

For a short block, only the first `sample_count * 3` bytes beginning at offset
20 contain samples. Every remaining byte through offset 4,095 must be zero.
This includes unused sample capacity and `reserved_tail`. A decoder must reject
a block with a valid CRC but nonzero unused bytes; this catches nonconforming
encoders and prevents future extensions from being mistaken for version 1.

`sample_count == 0` is invalid. An encoder must not emit an empty block.

### Flags

Version 1 defines one flag:

| Bit | Name | Meaning |
|---:|---|---|
| 0 | `DISCONTINUITY_BEFORE` | Timing/sample continuity with the previous block is intentionally broken and this block begins at a newly captured SAMP/RTC anchor |
| 7:1 | Reserved | Encoder writes zero; decoder must reject if nonzero |

Normal ECGv0 recording policy treats FIFO overflow as a collection fault and
does not continue in the same session. Therefore ordinary version 1 blocks
have `flags == 0`. `DISCONTINUITY_BEFORE` is reserved for an explicitly
implemented future recovery policy; using it requires reacquiring a physical
SAMP anchor before accepting the block's first sample.

There is deliberately no `FINAL` flag. When a recording stops exactly on a
full-block boundary, the already-written block cannot be changed safely.
Stream termination is represented by the stream protocol's terminal message,
and clean file termination is represented by the file trailer.

### Packed sample encoding

Each sample is the complete 24-bit MAX30001 FIFO register value stored most
significant byte first:

```text
payload[3*i + 0] = (raw24 >> 16) & 0xff
payload[3*i + 1] = (raw24 >>  8) & 0xff
payload[3*i + 2] =  raw24        & 0xff
```

This is the only big-endian item in the format. It matches the natural visual
ordering of the MAX30001 24-bit register and the existing ECG frame's raw-word
ordering. Metadata remains little-endian.

A decoder reconstructs and separates the fields as follows:

```text
raw24 = (byte0 << 16) | (byte1 << 8) | byte2
etag  = (raw24 >> 3) & 0x07
ptag  = raw24 & 0x07
u18   = (raw24 >> 6) & 0x3ffff
ecg   = u18 - 0x40000 if (u18 & 0x20000) != 0 else u18
```

`ecg` is then a signed integer in `[-131072, 131071]`. This specification does
not define conversion to volts because that conversion depends on MAX30001
gain and calibration settings. A decoder must preserve the raw signed count
even if it also calculates engineering units.

ETAG interpretation for payload validation is:

| ETAG | Meaning | Occupies a sample time slot? | ECG value usable? |
|---:|---|---|---|
| 0 | Valid | Yes | Yes |
| 1 | Fast/recovery | Yes | No; preserve and advance time |
| 2 | Valid, FIFO EOF | Yes | Yes |
| 3 | Fast/recovery, FIFO EOF | Yes | No; preserve and advance time |
| 4–5 | Reserved/unused | Must not be encoded | No |
| 6 | FIFO empty marker | Must not be encoded | No |
| 7 | FIFO overflow marker | Must not be encoded | No |

ETAG values 2 and 3 describe the MAX30001 FIFO drain boundary, not an ECG
block boundary. A decoder must not stop a block at an EOF-tagged sample.

Every encoded payload word has ETAG `0..3`; every such word increments
`sample_count`, the session sample index, and the derived RTC tick exactly
once. A fast/recovery sample remains in the block so the time axis does not
collapse.

### Timestamp and sample-index semantics

The first time-valid FIFO word after the session's captured SAMP edge has:

```text
sample_index = 0
rtc_tick     = captured_session_anchor_tick
```

For every later time-valid FIFO word in an uninterrupted session:

```text
sample_index = previous_sample_index + 1 modulo 2^32
rtc_tick     = previous_rtc_tick + 1 modulo 2^32
```

When a new block receives its first sample, the encoder records that sample's
already-derived tick and index into `first_rtc_tick` and
`first_sample_index`. It must not read RTC0 at block allocation, finalization,
queueing, or write time.

For zero-based sample position `i`, where `0 <= i < sample_count`:

```text
sample_index(i) = (first_sample_index + i) modulo 2^32
rtc_tick(i)     = (first_rtc_tick + i) modulo 2^32
seconds(i)      = rtc_tick(i) / 512 relative to the collection RTC epoch
```

The integer tick is authoritative. `seconds(i)` is a presentation conversion;
an implementation should keep integer ticks for alignment and comparison.

Because the MAX30001 and RTC share the physical clock, a decoder must not fit
or estimate a sample period. It must use exactly one RTC tick per encoded
sample.

### Continuity validation

After accepting two adjacent normal blocks, a decoder should require:

```text
current.first_sample_index ==
    (previous.first_sample_index + previous.sample_count) modulo 2^32

current.first_rtc_tick ==
    (previous.first_rtc_tick + previous.sample_count) modulo 2^32
```

Failure means at least one block is missing, duplicated, reordered, from a
different session, or has incorrectly generated timing metadata. CRC success
does not override a continuity failure.

If `DISCONTINUITY_BEFORE` is set, a decoder does not apply the second equation
across that boundary. It should still report the discontinuity. The meaning of
the sample index across such a boundary must be defined by the recovery policy;
the preferred policy is to continue the index so missing storage blocks remain
detectable.

## Encoder procedure

An encoder must implement the following behavior:

1. Before starting the MAX30001, clear block state and set the next session
   sample index to zero.
2. Issue MAX30001 `SYNCH`, capture the first post-`SYNCH` SAMP edge in the RTC0
   domain, and use the captured tick for the first FIFO sample.
3. Allocate or claim a zero-filled 4,096-byte block buffer before accepting the
   first sample for that block.
4. On the block's first sample, write `ECB1`, `first_rtc_tick`,
   `first_sample_index`, sample format 1, and flags. Leave `sample_count` and
   CRC zero until finalization.
5. For each FIFO word with ETAG `0..3`, append exactly three bytes in the
   encoding above, then increment the block count, next sample index, and next
   RTC tick. Do not append EMPTY, OVERFLOW, or reserved ETAG words.
6. When the count reaches 1,358, finalize the block. On a clean recording stop,
   finalize a nonempty short block in the same way.
7. To finalize, write `sample_count`, confirm every unused byte is zero, zero
   the CRC field, calculate CRC-32 across all 4,096 bytes, and write the CRC
   little-endian.
8. Publish the finalized immutable block to both authorized consumers: the
   storage writer and the stream history/forward capture path. Neither
   consumer may observe or transmit a partially filled block.
9. Do not reuse the buffer until every consumer that retained it has released
   ownership.

The storage and stream paths must consume identical finalized block bytes.
They must not independently re-encode samples, alter flags, change padding, or
recalculate timestamps.

### Recommended firmware buffering

Use a fixed pool of at least four 4,096-byte buffers, following the existing
accelerometer recorder pattern. Suggested states are:

```text
FREE -> FILLING -> FINALIZED -> QUEUED/REFERENCED -> WRITING/SENDING -> FREE
```

Reference counting or explicit dual-consumer ownership is required if storage
and streaming can retain the same block concurrently. Copying a finalized
block into the stream ring is also valid and simpler, provided the 4 KiB copy
is done outside interrupt context.

The FIFO interrupt handler must only wake the recorder. SPI FIFO draining,
packing, finalization, CRC, and consumer publication occur in thread/workqueue
context. No dynamic allocation is required.

At 512 Hz, one full block is finalized every 1,358 samples, or
2.65234375 seconds. CRC work is one linear 4 KiB pass at that cadence.

## Standalone block-decoder procedure

Given exactly 4,096 candidate bytes, a conforming decoder performs these steps
in order:

1. Require bytes `0..3` to equal ASCII `ECB1`.
2. Read `sample_count` little-endian and require `1 <= sample_count <= 1358`.
3. Require `sample_format == 1`.
4. Require `(flags & 0xfe) == 0`.
5. Require all bytes from `20 + 3*sample_count` through 4,095 to be zero.
6. Validate CRC-32 using the procedure in this document.
7. Decode exactly `sample_count` three-byte words beginning at offset 20.
8. Require each word's ETAG to be in `0..3`.
9. Derive each sample tick and index from the first values plus its zero-based
   position.
10. If a previous block in the same sequence is available, apply the continuity
    checks above unless the discontinuity flag explicitly relaxes them.

A decoder must return an error for a failed required check. Recovery tools may
report partial diagnostic information, but must not label a failed block as
valid ECG data.

## FAT file container

The file container is used only on the filesystem. The sensor stream sends
data blocks and does not send the file header, preallocation gap, or file
trailer.

### Naming and chunking

Use the existing collection ID and zero-based four-digit chunk suffix:

```text
<patient>ecg<session_id>_0000.bin
<patient>ecg<session_id>_0001.bin
...
```

`session_id` is the existing collection identifier:

```text
session_id = (unix_time_seconds * 1000) + (uptime_milliseconds modulo 1000)
```

Chunk indices are independent between ECG and accelerometer files. Existing
files must never be overwritten.

Every ECG file is preallocated to exactly 4,194,304 bytes and is never
truncated. Its layout is:

```text
offset 0x000000   4,096-byte ECG file header
offset 0x001000   up to 1,022 complete ECG data blocks
offset 0x3ff000   4,096-byte ECG file trailer
file size         4,194,304 bytes
```

There are no short physical writes in the data region. A final short logical
block is still a complete zero-padded 4,096-byte block.

### File header (`ECF1`)

The file header occupies bytes `0..4095` and has this exact layout:

| Offset | Size | Field | Required value or meaning |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `ECF1` |
| 4 | 2 | `format_version` | `1` |
| 6 | 2 | `defined_header_bytes` | `48` |
| 8 | 4 | `sample_rate_numerator` | `512` |
| 12 | 4 | `sample_rate_denominator` | `1` |
| 16 | 4 | `rtc_tick_hz` | `512` |
| 20 | 2 | `data_block_bytes` | `4096` |
| 22 | 2 | `data_block_header_bytes` | `20` |
| 24 | 2 | `samples_per_full_block` | `1358` |
| 26 | 1 | `sample_bytes` | `3` |
| 27 | 1 | `sample_format` | `1` |
| 28 | 4 | `file_flags` | Bit 0 set: MAX30001 and RTC share the physical clock; other bits zero |
| 32 | 8 | `session_id` | Collection ID used in the filename |
| 40 | 4 | `header_crc32` | CRC of all 4,096 header bytes |
| 44 | 4 | `chunk_index` | Zero-based file chunk index |
| 48 | 4,048 | `reserved` | Zero |

The encoder writes and synchronizes this header after successful preallocation
and before it writes any ECG data block. It never rewrites the header.

A decoder must validate all fixed geometry and rate values. It must require
file flag bit 0 and reject nonzero unknown file-flag bits for version 1.

### Data region

Data block `j`, where `0 <= j < 1022`, begins at:

```text
file_offset(j) = 4096 * (j + 1)
```

Every occupied slot contains one complete common `ECB1` block. The first
sample index continues across file chunks; it does not reset at a chunk
boundary. Therefore the first block in chunk N+1 must follow the last block in
chunk N according to the continuity equations.

Unused preallocated data slots are not ECG data. Their contents are ignored
when a valid trailer is available.

### File trailer (`ECT1`)

The file trailer occupies bytes `0x3ff000..0x3fffff`:

| Offset | Size | Field | Required value or meaning |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `ECT1` |
| 4 | 2 | `format_version` | `1` |
| 6 | 2 | `defined_trailer_bytes` | `40` |
| 8 | 4 | `valid_block_count` | Number of valid data blocks, `0..1022` |
| 12 | 8 | `valid_sample_count` | Sum of `sample_count` in this file |
| 20 | 4 | `valid_data_bytes` | `valid_block_count * 4096` |
| 24 | 4 | `first_sample_index` | First sample index in the file; zero if no blocks |
| 28 | 4 | `next_sample_index` | Index expected after the final sample; zero if no blocks |
| 32 | 4 | `trailer_flags` | Bit 0 `CLEAN_CLOSE`; other bits zero |
| 36 | 4 | `trailer_crc32` | CRC of all 4,096 trailer bytes |
| 40 | 4,056 | `reserved` | Zero |

For a normal rotation or collection stop, set `CLEAN_CLOSE`, calculate the
trailer CRC, write the complete trailer page, synchronize, and close the file.

`valid_sample_count` is 64-bit so aggregate tooling need not infer rollover.
Within a conforming file it cannot exceed `1022 * 1358 = 1,387,876`.

For a nonempty file:

```text
next_sample_index ==
    (first_sample_index + valid_sample_count) modulo 2^32
```

The trailer contains no absolute Unix timestamp. Session identity and any
wall-clock mapping remain collection metadata concerns; ECG alignment uses
the shared RTC tick basis.

### Clean file decoding

A file decoder must:

1. Require the file size to equal 4,194,304 bytes.
2. Validate the `ECF1` header, all required values, reserved bytes, and CRC.
3. Validate the `ECT1` trailer, all required values, reserved bytes, and CRC.
4. Require `CLEAN_CLOSE` and `valid_data_bytes == valid_block_count * 4096`.
5. Decode exactly `valid_block_count` blocks from consecutive data slots.
6. Validate every block independently and validate block-to-block continuity.
7. Sum all block sample counts and require equality with
   `valid_sample_count`.
8. Validate the trailer's first and next sample indices.
9. Ignore all unused preallocated data slots after the valid blocks.

### Interrupted-file recovery

If the header is valid but the trailer is absent, corrupt, or not marked
`CLEAN_CLOSE`, the file is interrupted. A recovery decoder may scan consecutive
data slots starting at offset `0x1000`.

It accepts blocks only while all of the following hold:

- the complete 4,096-byte block is readable;
- standalone block validation succeeds; and
- continuity with the previously recovered block succeeds.

Recovery stops at the first failed slot. It must not scan past the failure and
resume later, because later bytes may be stale preallocation data or belong to
an earlier filesystem allocation. All accepted blocks remain independently
CRC-protected.

A valid short logical block is recoverable because it is physically 4,096
bytes and carries `sample_count`; unlike the accelerometer v3 format, recovery
does not depend on a trailer to determine a short block's encoded length.

### File-writer synchronization policy

The intended policy is:

- preallocate 4 MiB before writing the header;
- write and `fs_sync()` the header before data acquisition is considered
  active;
- write only aligned 4,096-byte data blocks;
- `fs_sync()` every eight full blocks, matching the current accelerometer
  recorder policy;
- synchronize at chunk rotation and collection stop; and
- write the trailer only after all data-block writes for that chunk complete.

CRC detects torn pages but does not make FAT directory or allocation metadata
atomic. More frequent synchronization may reduce the filesystem recovery
window at the cost of latency and NAND traffic; changing that policy does not
change the binary format.

## Sensor-stream transport

### Layering rule

The stream's ECG sensor payload is a concatenation of complete common `ECB1`
blocks. It does not contain `ECF1` headers, `ECT1` trailers, FAT preallocation
bytes, filenames, or filesystem offsets.

After transport reassembly, the byte sequence supplied to the standalone
block decoder must be byte-for-byte identical to the finalized blocks supplied
to the storage writer. A stream decoder should therefore have two layers:

1. transport decoder: validates stream messages and reassembles 4,096-byte
   block byte strings;
2. ECG block decoder: applies the standalone decoder procedure without any
   FAT logic.

### Required protocol migration

The existing sensor-stream protocol v1 treats each 12-byte ECG frame as an
indivisible record. A 4,096-byte block cannot fit in one BLE notification, so
changing only `record_size` to 4,096 is invalid. The stream protocol must move
to version 2 and define byte-fragment semantics.

### Version 2 command and common envelope

The version 2 stream command remains exactly eight bytes:

| Offset | Size | Field | Required value or meaning |
|---:|---:|---|---|
| 0 | 1 | `magic0` | `0x4d`, ASCII `M` |
| 1 | 1 | `magic1` | `0x53`, ASCII `S` |
| 2 | 1 | `protocol_version` | `2` |
| 3 | 1 | `opcode` | `1` START or `2` CANCEL |
| 4 | 4 | `session_id` | Nonzero central-selected `uint32_le` |

Every version 2 TX notification begins with this exact 12-byte common header:

| Offset | Size | Field | Required value or meaning |
|---:|---:|---|---|
| 0 | 1 | `magic0` | `0x4d`, ASCII `M` |
| 1 | 1 | `magic1` | `0x53`, ASCII `S` |
| 2 | 1 | `protocol_version` | `2` |
| 3 | 1 | `message_type` | `0x81` START_ACK, `0x82` DATA, `0x83` END, or `0x84` RESULT |
| 4 | 4 | `session_id` | ID from the accepted START command, `uint32_le` |
| 8 | 2 | `payload_length` | Exact number of bytes following this header, `uint16_le` |
| 10 | 2 | `common_flags` | Zero |

The complete notification length must equal `12 + payload_length`. A decoder
rejects the message if magic, version, length, flags, or active session ID does
not match.

### Version 2 START_ACK

START_ACK is sent immediately after a START is accepted and the stream enters
the active/arming state. It does not mean that the next block boundary has
already occurred. Its payload is exactly 96 bytes:

| Payload offset | Size | Field | Required ECG value or meaning |
|---:|---:|---|---|
| 0 | 1 | `device_type` | `0x02`, ECG |
| 1 | 1 | `ecg_block_format_version` | `1` |
| 2 | 2 | `block_bytes` | `4096`, `uint16_le` |
| 4 | 4 | `sample_rate_numerator` | `512`, `uint32_le` |
| 8 | 4 | `sample_rate_denominator` | `1`, `uint32_le` |
| 12 | 4 | `history_block_count` | `2`, `uint32_le` |
| 16 | 4 | `forward_block_count` | `4`, `uint32_le` |
| 20 | 4 | `total_ecg_bytes` | `24576`, `uint32_le` |
| 24 | 8 | `device_id` | Raw `msense_device_identity_bytes()` byte order |
| 32 | 1 | `device_name_length` | `1..16` |
| 33 | 16 | `device_name` | UTF-8/ASCII, zero padded after the declared length |
| 49 | 40 | `git_commit` | Full lowercase hexadecimal firmware-repository commit |
| 89 | 1 | `git_tree_state` | `0` clean, `1` dirty, `2` unknown |
| 90 | 6 | `reserved` | Zero |

The START_ACK common-header `payload_length` is 96. Values at offsets 2, 12,
and 16 describe indivisible common blocks, not BLE notification records.

### Version 2 DATA

For a version 2 ECG `DATA` message, the 12-byte DATA prefix is:

| DATA payload offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | `message_sequence` | Starts at zero; increments once per DATA message |
| 4 | 4 | `stream_byte_offset` | Offset of the first fragment byte in the concatenated ECG block stream |
| 8 | 2 | `fragment_bytes` | Number of ECG bytes following this prefix |
| 10 | 1 | `phase` | `0` history, `1` forward |
| 11 | 1 | `fragment_flags` | Defined below |
| 12 | `fragment_bytes` | `fragment` | Consecutive bytes from one ECG block |

Fragment flags are:

| Bit | Name | Meaning |
|---:|---|---|
| 0 | `BLOCK_START` | Fragment begins at block offset zero |
| 1 | `BLOCK_END` | Fragment contains byte 4,095 as its final byte |
| 7:2 | Reserved | Must be zero |

Normative fragmentation rules:

- One fragment must never cross an ECG block boundary.
- `fragment_bytes` must be nonzero and must equal the actual bytes following
  the DATA prefix.
- `stream_byte_offset` must equal the total ECG fragment bytes accepted before
  this message.
- `message_sequence` and `stream_byte_offset` must both be contiguous.
- `BLOCK_START` must be set exactly when `stream_byte_offset % 4096 == 0`.
- `BLOCK_END` must be set exactly when
  `(stream_byte_offset + fragment_bytes) % 4096 == 0`.
- A phase transition may occur only between blocks.
- The largest fragment is determined by negotiated ATT/NUS capacity after the
  outer message header and DATA prefix are subtracted. The sender should use
  the largest permitted fragment except at a block boundary.
- BLE GATT notifications preserve order but are not application-acknowledged.
  A gap in message sequence or byte offset invalidates the stream capture.
- The block CRC is the end-to-end ECG integrity check. Outer BLE link CRCs do
  not replace it.

For DATA, common-header `payload_length` must equal
`12 + fragment_bytes`. The complete notification length must therefore equal
`24 + fragment_bytes`.

The negotiated ATT MTU must be at least 128, as in protocol version 1, because
START_ACK is a 108-byte complete notification. An ATT notification value may
contain at most `att_mtu - 3` bytes, so the maximum ECG fragment is:

```text
maximum_fragment_bytes = att_mtu - 3 - 12 - 12
                       = att_mtu - 27
```

The sender additionally limits that value so a fragment ends at, but never
crosses, the next 4,096-byte block boundary.

On a successful fixed-geometry stream, byte offsets `0..8191` have phase 0 and
offsets `8192..24575` have phase 1. The first message has sequence and byte
offset zero. The last fragment ends at byte offset 24,576.

### Version 2 RESULT and END

RESULT retains the existing four-byte payload and is used for a rejected or
completed command that did not start a DATA sequence:

| Payload offset | Size | Field |
|---:|---:|---|
| 0 | 2 | `status`, `uint16_le` |
| 2 | 1 | Current stream state |
| 3 | 1 | Reserved, zero |

END is terminal for an accepted stream session. No DATA may follow it for the
same session. Its payload is exactly 24 bytes:

| Payload offset | Size | Field | Meaning on successful ECG stream |
|---:|---:|---|---|
| 0 | 2 | `final_status` | Zero for success, `uint16_le` |
| 2 | 1 | `post_terminal_state` | Stream state after cleanup |
| 3 | 1 | Reserved | Zero |
| 4 | 4 | `history_blocks_sent` | `2`, `uint32_le` |
| 8 | 4 | `forward_blocks_captured` | `4`, `uint32_le` |
| 12 | 4 | `total_ecg_bytes_sent` | `24576`, `uint32_le` |
| 16 | 4 | `data_message_count` | Number of DATA messages, `uint32_le` |
| 20 | 4 | `detail` | Zero on success; otherwise signed Zephyr errno, `int32_le` |

The existing numeric status codes retain their version 1 assignments. In
particular: success `0x0000`, not recording `0x0001`, history not ready
`0x0002`, not subscribed `0x0003`, busy `0x0004`, MTU too small `0x0005`,
invalid command `0x0006`, unsupported version `0x0007`, cancelled `0x0008`,
storage error `0x0009`, internal error `0x000a`, not initialized `0x000b`,
wrong session `0x000c`, and disconnected `0x000d`.

Stream state values also retain their assignments: not recording `0x00`,
history filling `0x01`, ready `0x02`, active (including arming) `0x03`,
aborting `0x04`, and uninitialized `0x05`.

### Stream capture geometry

The required ECG stream geometry is:

| Phase | Blocks | Bytes | Samples when full | Nominal duration |
|---|---:|---:|---:|---:|
| History | 2 | 8,192 | 2,716 | 5.3046875 s |
| Forward | 4 | 16,384 | 5,432 | 10.609375 s |
| Total | 6 | 24,576 | 8,148 | 15.9140625 s |

This approximately preserves the existing ECG stream's 5.334-second history,
10.666-second forward, and 16-second total time windows while reducing sensor
payload and capture RAM from 98,304 bytes to 24,576 bytes.

If a product intentionally retains a 96 KiB ECG payload, it would contain 24
blocks: eight history and sixteen forward. That would increase the time window
to approximately 63.66 seconds. This is not the default version 2 geometry.

### Block-boundary request semantics

A request can arrive while the encoder is filling a block. Splitting that
block would violate the common-format and identical-byte requirements.
Therefore version 2 ECG stream activation is aligned to a block boundary:

1. While recording, keep a ring containing the last two completely finalized
   blocks.
2. On a valid START command, enter a pending/arming state. If a block already
   contains samples, continue filling it normally. If the current block is
   empty, the boundary is already established and no extra block is awaited.
3. When a partially filled current block is finalized, include it in the
   rolling history, then freeze the most recent two completed blocks as
   history. At an already-empty boundary, freeze the two most recent completed
   blocks immediately.
4. The boundary immediately after that finalized block is the history/forward
   boundary. The next four blocks are forward blocks.
5. Send the two history blocks followed by the four forward blocks. Every block
   remains in chronological order and every phase change occurs between
   blocks.

This gives precise phase semantics: no forward block contains a sample from
before the activation boundary. The START command may take up to one block
period, 2.65234375 seconds, to arm. START_ACK is sent immediately when the
request is accepted. The existing `ACTIVE` state includes this bounded arming
interval; DATA begins only after the block boundary is established.

If fewer than two completed blocks exist when requested, return the existing
`HISTORY_NOT_READY` status. Do not construct synthetic or partially filled
history blocks.

A normal recording stop may produce a short common block for disk. An active
stream may transmit that short block only if protocol terminal semantics allow
an early, explicitly reported capture; it must not count a short block as a
full forward block while still claiming the fixed geometry above. The default
behavior is to send END with final status `NOT_RECORDING` (`0x0001`), actual
completed block/byte counts, and no short block in the stream payload.

## Efficiency

### On-flash efficiency

Current ECG records are 12 bytes at 512 samples/s:

```text
current rate = 12 * 512 = 6144 bytes/s
```

For full version 1 blocks:

```text
block duration       = 1358 / 512 = 2.65234375 s
block storage rate   = 4096 * 512 / 1358
                     = 1544.29455081001 bytes/s
payload efficiency   = (1358 * 3) / 4096
                     = 99.462890625 percent
reduction vs current = 74.8649975454 percent
capacity multiplier  = 3.978515625
```

Including one file header and one trailer in every 4 MiB chunk:

```text
data blocks per file = 1022
samples per file     = 1022 * 1358 = 1,387,876
duration per file    = 1,387,876 / 512
                     = 2710.6953125 s
                     = 45.1782552083 minutes
effective file rate  = 4,194,304 / 2710.6953125
                     = 1547.31665364917 bytes/s
reduction vs current = 74.8158096737 percent
capacity multiplier  = 3.97074508667
```

A 4 MiB current-format payload represents about 11.38 minutes. The proposed
file represents about 45.18 minutes.

At continuous acquisition, approximate binary GiB per 24 hours changes from:

```text
current:  0.4944 GiB/day
proposed: 0.1245 GiB/day, including file header/trailer overhead
```

Compared only with a hypothetical packed `uint32_t` sample stream, the common
block format reduces the effective full-block byte rate by about 24.595%.

### Streaming efficiency

The common block removes per-sample sync, type, repeated timestamp, and CRC-8
bytes. The version 2 stream geometry also reduces captured ECG sensor bytes
from 96 KiB to 24 KiB while preserving approximately the same time window.
This gives:

- 75% less ECG history/forward capture RAM;
- 75% fewer sensor-payload bytes sent for a requested ECG capture;
- fewer producer-to-stream calls if publication occurs once per block rather
  than once per sample; and
- an end-to-end CRC-32 for every reassembled block.

Transport framing overhead depends on negotiated ATT MTU and is additional to
the 24,576 sensor bytes. It must be measured separately and does not alter the
block format.

### MCU work

Steady-state work consists of three byte stores per sample, integer counter
increments, a 4 KiB CRC pass every 2.652 seconds, and block ownership changes.
No division, floating point, compression search, entropy coding, or dynamic
allocation is needed in the acquisition path.

## Robustness properties and limitations

The format provides:

- block resynchronization using 4 KiB alignment and `ECB1` magic;
- detection of torn or corrupted blocks with CRC-32;
- exact logical length for short blocks;
- detection of missing, duplicated, or reordered blocks using sample index and
  timestamp continuity;
- preservation of MAX30001 invalid/fast samples without collapsing time; and
- interrupted-file recovery without requiring a valid trailer.

The format does not provide:

- error correction beyond lower-layer NAND ECC;
- cryptographic authenticity or protection against intentional modification;
- atomic FAT metadata updates;
- compensation for MAX30001 filter group delay;
- absolute UTC by itself; or
- recovery of samples lost in a MAX30001 FIFO overflow.

## Versioning and compatibility

The magic values are part of versioning:

| Magic | Structure |
|---|---|
| `ECF1` | Version 1 ECG file header |
| `ECB1` | Version 1 common ECG data block |
| `ECT1` | Version 1 ECG file trailer |

Any incompatible change requires new magic/version values and a separate
decoder path. Incompatible changes include changing byte order, CRC algorithm,
block size, header offsets, sample packing, timestamp meaning, or the handling
of unused bytes.

The existing 12-byte `A5 EC` ECG frames are a different format. A version 1
block decoder must not search for or interpret those frames inside an `ECB1`
payload. The sensor stream must increment its protocol version when switching
from framed records to block fragments so peers cannot silently confuse the
two encodings.

## Minimum interoperability tests

Encoder and decoder implementations must include tests for at least:

1. CRC check value `0xCBF43926` for ASCII `123456789`.
2. Full 1,358-sample block layout, CRC, and two zero tail bytes.
3. One-sample short block with all remaining bytes zero.
4. Rejection of sample counts 0 and 1,359.
5. Raw words `0x000000`, `0x7fffff`, `0x800000`, and `0xffffff`, including
   correct signed-18 extraction and tag extraction.
6. Rejection of payload ETAG 4, 5, 6, and 7.
7. RTC and sample-index wrap from `0xffffffff` to zero.
8. Adjacent-block continuity and deliberate missing/duplicate block failures.
9. Detection of a one-bit change in header, payload, padding, and CRC fields.
10. Rejection of nonzero reserved flags or unused payload bytes even after CRC
    is recomputed.
11. File header and trailer encode/decode with exact 4 MiB geometry.
12. Interrupted-file recovery stopping at the first invalid block.
13. Cross-chunk continuity between `_0000.bin` and `_0001.bin`.
14. Stream fragmentation and reassembly at several ATT MTUs, including a block
    requiring many fragments.
15. Detection of missing, duplicated, reordered, overlapping, and
    boundary-crossing stream fragments.
16. Byte-for-byte equality between a block read from disk and the corresponding
    reassembled streamed block.

## Implementation impact checklist

A future implementation must update all of the following together:

- `ECGv0/src/ecgRecordFormat.*`: replace 12-byte frame construction with the
  common block encoder.
- `ECGv0/src/ecgRecorder.*`: add fixed block-pool ownership, first-sample block
  metadata, finalization, and common publication.
- `ECGv0/src/zephyrfilesystem.*`: replace 8,196-byte ECG batching with aligned
  block writes plus `ECF1`/`ECT1` file lifecycle.
- `shared/include/msense_sensor_stream_protocol.h`: define sensor stream
  protocol version 2, block geometry, fragment semantics, and revised capture
  geometry.
- `shared/sensor_stream.c`: accept finalized blocks, maintain a two-block ECG
  history ring, collect four forward blocks, and fragment blocks for NUS.
- `central_nus_test`: parse protocol v2 fragments, enforce offsets/flags, and
  run the common block decoder after reassembly.
- Host extraction tools: add `ECF1` file and raw `ECB1` stream paths using one
  shared block-decoding implementation.
- Device information text and documentation: identify ECG block format v1
  rather than the temporary 12-byte frame format.
- Tests: replace temporary ECG frame tests and extend stream tests with the
  minimum interoperability cases above.

The format change and stream-protocol change must ship atomically. Firmware
must not advertise stream protocol v1 while sending `ECB1` fragments, and a
host must select its decoder from explicit protocol/file magic rather than
firmware-date heuristics.

## Non-goals

Version 1 does not add compression, per-sample timestamps, per-sample CRCs,
floating-point values, calibration coefficients, UTC timestamps, accelerometer
data, derived heart-rate metrics, or filter-delay correction. It does not
change MAX30001 acquisition configuration. These may be added only through an
explicitly versioned extension or separate metadata artifact.
