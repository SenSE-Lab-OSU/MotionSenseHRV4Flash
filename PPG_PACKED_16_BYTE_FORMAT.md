# PPGv2 packed 16-byte record format

This draft records the persisted PPGv2 data contract implemented by
`src/ppgSensor.c`. It describes existing behavior; it does not authorize a
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
discarded in complete 16-byte units, never by trimming arbitrary bytes.

A useful interoperability vector is:

```text
ir1    = 0x000001
ir2    = 0x012345
g1     = 0x07ffff
g2     = 0x000100
tick   = 0x12345678

bytes  = 01 00 00 45 23 01 ff ff 07 00 01 00 78 56 34 12
```

## Compatibility boundary

Do not change this byte layout, PPG recording names, the 512 Hz tick meaning,
BLE packet layout, or the `uuid.txt` filename/creation timing as part of
internal CMake naming or source cleanup. A standalone PPG host decoder test is
not currently tracked in this consolidation tree; Phase 5 preserves the
implemented encoder and records the compatibility check for owner review rather
than inventing a new format or normalizing it with ECG records.
