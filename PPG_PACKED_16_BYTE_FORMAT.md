# PPGv2 packed 16-byte record format

This document records the persisted PPGv2 data contract implemented by
`PPGv2/src/ppgSensor.c`. It describes existing behavior; it does not authorize a
format migration.

## Record layout

Every PPG sample record is exactly 16 bytes. All fields are unsigned and
little-endian.

| Offset | Size | Field | Valid range |
| ---: | ---: | --- | --- |
| 0 | 3 | infrared channel 1 (`ir1`) | `0x00000`–`0x7ffff` |
| 3 | 3 | infrared channel 2 (`ir2`) | `0x00000`–`0x7ffff` |
| 6 | 3 | green channel 1 (`g1`) | `0x00000`–`0x7ffff` |
| 9 | 3 | green channel 2 (`g2`) | `0x00000`–`0x7ffff` |
| 12 | 4 | `global_tick_512hz` | `0x00000000`–`0xffffffff` |

The four optical channels contain 19 meaningful bits. Bits 19 through 23 of
each three-byte field remain zero. `global_tick_512hz` retains normal unsigned
32-bit rollover semantics; it is not a Unix timestamp.

The firmware enforces the record size with `PPG_NAND_RECORD_SIZE == 16U`,
masks samples with `PPG_SAMPLE_MASK == 0x7ffffU`, serializes the values with
explicit byte writes, and calls `store_data()` with exactly 16 bytes per PPG
sample. `uuid.txt` records the compatible format label:

```text
uint24_le ir1, uint24_le ir2, uint24_le g1, uint24_le g2, uint32_le global_tick_512hz
```

## Decoder rules

Decode each channel from three little-endian bytes and the tick from four
little-endian bytes. Reject a channel value with any nonzero bit above bit 18.
Keep complete-record boundaries: preallocated trailing data may only be
discarded in complete 16-byte units, never by trimming arbitrary bytes. This file
rule does not prohibit dropping an incomplete final streaming record: STOP or a
fault may leave 1–15 trailing bytes, which the stream receiver discards while
retaining all preceding complete records.

A useful interoperability vector is:

```text
ir1    = 0x000001
ir2    = 0x012345
g1     = 0x07ffff
g2     = 0x000100
tick   = 0x12345678

bytes  = 01 00 00 45 23 01 ff ff 07 00 01 00 78 56 34 12
```

## Transport compatibility

The shared NUS v0 transport preserves these 16-byte records and the existing
PPG filenames, tick meaning, and `uuid.txt` behavior. It changes only the stream
envelope and lifecycle: finite and INFINITY modes use ACK, uint64-offset DATA,
and status-only END. The old v1 NUS envelope is no longer emitted or accepted by
the lab central. The version byte is `0`, independent of the PPGv2 product name.
See [the shared streaming profile](ECG_BLOCK_FORMAT.md#15-ppg-profile).

PPG records have no CRC. A complete framed record is not equivalent to an ECG
block that has passed CRC and continuity validation. No new PPG record wrapper,
checksum or storage format is introduced by the transport change. Streaming
history is nevertheless zero-initialized and cleared on acquisition discontinuity;
those zero-filled 16-byte records count toward transport length and cannot reliably
be distinguished from legitimate records. Do not apply ECG's zero-block skip rule
or infer an exact valid-sample count from the number of nonzero records.
