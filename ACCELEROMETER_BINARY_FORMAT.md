# Accelerometer binary recording format (v3)

## Purpose and scope

This document specifies the fixed-size ICM-20948 accelerometer files recorded
during an ECG collection session. The stream records raw X/Y/Z accelerometer
counts only. Version 3 replaces X bit 0 with a sampled FSYNC marker, so a
consumer must clear that bit before interpreting X as an acceleration count.
It never contains ECG, gyro, magnetometer, temperature, derived motion
metrics, PPG, ENMO, or BLE payloads.

Version 3 replaces the earlier fixed-layout v2 timing semantics. Every v3 file is a
4 MiB chunk and has a terminal record that identifies the valid encoded region
inside the preallocated file. This preserves a short final data block without
truncating the host-visible file.

## Recording configuration

| Property | Value |
|---|---|
| Sensor | InvenSense ICM-20948 |
| Sensor data | Accelerometer only, raw counts |
| Axis order | X, then Y, then Z |
| Data type | Signed 16-bit two's-complement little-endian; X bit 0 is FSYNC |
| Full-scale range | +/-2 g |
| Sensitivity | 16,384 counts/g |
| Nominal output-data rate | 562.5 Hz = 1,125 / 2 Hz |
| Sample period | 2 / 1,125 seconds |

No calibration or coordinate transform is applied. Convert Y and Z with
`raw_count / 16384.0`. Convert X with `(raw_x_bits & 0xfffe) / 16384.0`.
The marker replacement contributes at most one raw count, about 61 micro-g.

## Session filenames and chunking

The collection controller creates one shared ID:

```text
session_id = (unix_time_seconds * 1000) + (uptime_milliseconds modulo 1000)
```

Both ECG and accelerometer files use a zero-based, four-digit chunk suffix:

```text
<patient>ecg<session_id>_0000.bin
<patient>ecg<session_id>_0001.bin
<patient>ac<session_id>_0000.bin
<patient>ac<session_id>_0001.bin
```

The two streams have independent chunk indices. Equal indices do not imply
matching time spans because ECG and accelerometer streams fill at different
rates. No existing chunk is overwritten: an already-present filename is a
collection error.

Every file is preallocated to exactly 4,194,304 bytes (4 MiB), remains that
size after stop, and is never truncated. A full accelerometer chunk is closed
and a new chunk is opened before accepting further samples.

ECG chunks use the same fixed size and naming convention. Its current
8,196-byte writer batch rotates after 511 batches (leaving unused
preallocated capacity) so the 512th batch can never extend the 4 MiB file.

## Fixed v3 layout

```text
offset 0x000000   4,096-byte header
offset 0x001000   4,186,112-byte data region
offset 0x3ff000   4,096-byte terminal record
file size         4,194,304 bytes exactly
```

The data region can contain at most 1,022 full 4 KiB data blocks. It can also
end with one short final block. Bytes after the valid data region and before
the terminal record are preallocation space; a decoder must ignore them.

## Common encoding rules

1. All fields are serialized explicitly in little-endian byte order.
2. Magic values are the listed ASCII bytes, not host-native integer literals.
3. CRC is CRC-32/ISO-HDLC (standard Ethernet/zlib CRC-32): polynomial
   `0x04C11DB7`, reflected input/output, initial value `0xFFFFFFFF`, and
   final XOR `0xFFFFFFFF`.
4. While calculating a CRC, serialize that CRC field itself as four zero
   bytes. Store the resulting value little-endian.
5. Data blocks are written before the terminal record. The terminal record is
   written and synchronized only at a clean chunk rotation or collection stop.

## Header

The header is always bytes `0..4095`.

| Offset | Size | Field | Required value / meaning |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `ACF3` |
| 4 | 2 | `format_version` | `3` |
| 6 | 2 | `sample_format` | `2` = signed 16-bit little-endian X/Y/Z with X bit 0 sampled from FSYNC |
| 8 | 4 | `odr_numerator` | `1125` |
| 12 | 4 | `odr_denominator` | `2` |
| 16 | 2 | `full_scale_g` | `2` |
| 18 | 2 | `counts_per_g` | `16384` |
| 20 | 4 | `header_crc32` | CRC of all 4,096 header bytes |
| 24 | 4 | `anchor_clock_hz` | `512`, the RTC0 tick rate |
| 28 | 4 | `fsync_edge_interval_ticks` | `32`, so edges occur every 62.5 ms |
| 32 | 1 | `fsync_axis` | `0` = X |
| 33 | 1 | `fsync_bit` | `0` |
| 34 | 1 | `timestamp_algorithm` | `1` = rolling endpoint-period midpoint projection |
| 35 | 1 | `timestamp_window` | `32` FSYNC transitions |
| 36 | 4,060 | `reserved` | Encoder writes zero; decoder ignores after CRC validation |

The header CRC is calculated with bytes `20..23` zeroed. The header is
written and synchronized after successful 4 MiB preallocation and before IMU
streaming starts. It is never rewritten.

## Data blocks

A full data block is exactly 4,096 bytes.

| Offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `ACB1` |
| 4 | 4 | `reserved_timer_output` | Estimated 32-bit wrap-extended RTC0 tick associated with the first payload sample |
| 8 | 4 | `first_sample_sequence` | Modulo-2^32 sample sequence for the first payload sample |
| 12 | 4 | `block_crc32` | CRC of this block's encoded length |
| 16 | 4,080 | `samples` | 680 consecutive raw X/Y/Z samples |

Each sample is packed without padding:

```text
x int16 LE with bit 0 equal to sampled FSYNC, y int16 LE, z int16 LE
```

The sequence value continues across chunks in one session. It begins at zero
for the first sample in chunk `0000`; a full block advances it by 680. This
provides continuity checking across both blocks and files.

`reserved_timer_output` is a 32-bit wrap-extended RTC0 counter value stored in
a 32-bit little-endian field. RTC0 runs from the 32.768 kHz LFCLK with
prescaler 63, so one tick is 1/512 second. The firmware extends the 24-bit
hardware counter across overflow; all 32 bits are meaningful and the derived
value wraps only after 2^32 ticks.

RTC0 compare channel 1 toggles FSYNC every 32 ticks through DPPI and GPIOTE.
The ICM-20948 copies that level into X bit 0. Firmware pairs each observed
X-bit transition with its scheduled RTC edge, estimates the IMU period from
the oldest and newest of the latest 32 transitions, and projects the nearest
transition midpoint to `first_sample_sequence`. The rounded 512 Hz estimate
is stored at offset 4, so FIFO batching does not affect the anchor.

The estimate includes unknown sub-sample FSYNC phase, integer-tick rounding,
and hardware FSYNC input latency. It does not compensate accelerometer
DLPF/analog group delay.

For a short final block, the same 16-byte metadata precedes 1 through 679
samples. Its encoded length is:

```text
16 + (6 * remaining_sample_count)
```

The short block has no internal padding. Its length is supplied by the
terminal record, not inferred from physical EOF.

## Terminal record

The last 4 KiB sector of every cleanly closed v3 chunk, at offset
`0x3ff000`, is a terminal record.

| Offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `ACT2` |
| 4 | 4 | `valid_data_length` | Number of encoded data-region bytes beginning at offset `0x1000` |
| 8 | 4 | `trailer_crc32` | CRC of the complete 4,096-byte terminal sector |
| 12 | 4,084 | `reserved` | Zero |

`valid_data_length` is in `0..4,186,112`. It is the only file-level
length indicator needed for a fixed-size file; no per-block sample count is
introduced. The trailer CRC covers all 4,096 terminal bytes with bytes
`8..11` zeroed.

## Decoder procedure

1. Require a 4,194,304-byte file. Validate the `ACF3` header and its CRC.
2. Read and validate the terminal record at offset `0x3ff000`. Reject its
   `valid_data_length` if it exceeds the data-region capacity.
3. Parse exactly that many bytes beginning at `0x1000`: consecutive full
   blocks followed, optionally, by one short final block. Validate each
   magic, sequence, RTC0 high byte, and CRC.
4. Ignore every byte between the valid data region and the terminal record.
   Those bytes are preallocated capacity, not samples.
5. If the terminal record is absent or invalid, treat the chunk as
   interrupted. Recover only consecutive valid full blocks from the start of
   the data region and discard an incomplete short tail.

Version 1 files use `ACF1` and physical EOF to find a short final block.
Version 2 files use `ACF2`, sample format 1, and a delayed FIFO-ingestion
timestamp at offset 4. They remain separate decoder paths; v3 decoders must
not apply FSYNC semantics to them.

## Writer requirements

- All filesystem calls run on `my_work_q`; neither the IMU worker nor GPIO
  ISR performs filesystem I/O.
- Before header write, call FatFs `f_expand(..., 4 MiB, 1)` on the new empty
  file. Allocation failure is a collection fault.
- Write and synchronize the header before enabling the IMU.
- At 1,022 full data blocks, finish the terminal record, synchronize and
  close the current chunk, then open/preallocate/header-sync the next indexed
  chunk. The bounded four-buffer pool absorbs FIFO data while this serialized
  rotation runs; exhausting it is a collection fault.
- On normal stop, drain the IMU FIFO, write the optional short block, write
  and synchronize the terminal record, then close the file. Do not truncate.
- Synchronize every eight full blocks, at every chunk close, and at collection
  stop.
- Any malformed FIFO batch, unavailable block, allocation failure, write,
  sync, trailer, close, or rotation failure stops the whole collection rather
  than silently continuing ECG-only.

## Non-goals

Version 3 does not store gyro, magnetometer, temperature, separate FSYNC
payload bytes, interrupt status, calibration values, per-sample timestamps,
derived values, ENMO, PPG, BLE payloads, or dropped-sample records. Do not add
them without a new format version and decoder path.
