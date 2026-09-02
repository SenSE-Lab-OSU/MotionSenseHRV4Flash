# Handoff: hard-coded 16-byte PPG NAND records

## Goal and scope

Replace the current `main` PPG NAND record format with a hard-coded 16-byte format. This is a direct format migration, not a Kconfig option: after the change, firmware must always encode 16-byte PPG records and the repository's data extractor must decode that format.

Make no unrelated changes relative to `main`. Preserve the 256 Hz PPG output rate, MAXM86161 configuration, two-sample averaging, FIFO framing, channel order, 512 Hz global tick semantics, BLE packets, IMU records, file naming, buffering, NAND behavior, logging, USB behavior, power configuration, and SDK patches. Do not add a Kconfig symbol or edit `prj.conf`.

## Current implementation to replace

In `MSenseDevice/src/ppgSensor.c`, `ppg_samples` is currently a five-element `uint32_t` array. `read_ppg_fifo_buffer()` fills it with:

1. `led1A[i]`: infrared channel 1
2. `led1B[i]`: infrared channel 2
3. `led2A[i]`: green channel 1
4. `led2B[i]`: green channel 2
5. `global_tick_512hz`

It then calls `store_data(ppg_samples, sizeof(ppg_samples), ppg)`, producing a 20-byte record. Replace this NAND record construction with the encoder below. Do not change the separate BLE packing path.

The FIFO decoder already masks every PPG sample with `0x7FFFF`, so each channel contains 19 meaningful bits. The new format allocates three bytes per channel. Bits 19 through 23 must always be zero.

## Canonical 16-byte format

Every field is unsigned and little-endian.

| Offset | Size | Field | Valid range |
|---:|---:|---|---:|
| 0 | 3 | Infrared channel 1 (`led1A`) | `0x00000`–`0x7FFFF` |
| 3 | 3 | Infrared channel 2 (`led1B`) | `0x00000`–`0x7FFFF` |
| 6 | 3 | Green channel 1 (`led2A`) | `0x00000`–`0x7FFFF` |
| 9 | 3 | Green channel 2 (`led2B`) | `0x00000`–`0x7FFFF` |
| 12 | 4 | Full `global_tick_512hz` | `0x00000000`–`0xFFFFFFFF` |

There is no Unix timestamp in each record. Timing remains:

```text
elapsed_seconds = (record_tick - collection_start_tick) / 512.0
```

Subtraction must use unsigned 32-bit rollover semantics. At 256 PPG records/s, normally adjacent records differ by two ticks; a different delta is useful for detecting delayed or missing records and must not be normalized away by the decoder.

## Firmware encoder

Use explicit byte operations. Do not use C bit fields, packed structs, or `memcpy()` of the low three bytes of a `uint32_t`, because those approaches make the persisted format dependent on compiler layout or CPU endianness.

Reference implementation:

```c
#define PPG_NAND_RECORD_SIZE 16U
#define PPG_SAMPLE_MASK 0x7FFFFU

static void ppg_put_u24_le(uint8_t *dst, uint32_t value)
{
	value &= PPG_SAMPLE_MASK;
	dst[0] = (uint8_t)value;
	dst[1] = (uint8_t)(value >> 8);
	dst[2] = (uint8_t)(value >> 16);
}

static void ppg_put_u32_le(uint8_t *dst, uint32_t value)
{
	dst[0] = (uint8_t)value;
	dst[1] = (uint8_t)(value >> 8);
	dst[2] = (uint8_t)(value >> 16);
	dst[3] = (uint8_t)(value >> 24);
}

static void ppg_encode_nand_record(uint8_t record[PPG_NAND_RECORD_SIZE],
				   uint32_t ir1, uint32_t ir2,
				   uint32_t green1, uint32_t green2,
				   uint32_t global_tick_512hz)
{
	ppg_put_u24_le(&record[0], ir1);
	ppg_put_u24_le(&record[3], ir2);
	ppg_put_u24_le(&record[6], green1);
	ppg_put_u24_le(&record[9], green2);
	ppg_put_u32_le(&record[12], global_tick_512hz);
}
```

The storage loop should use a 16-byte byte array and submit exactly its size:

```c
uint8_t ppg_record[PPG_NAND_RECORD_SIZE];

ppg_encode_nand_record(ppg_record,
	led1A[i], led1B[i], led2A[i], led2B[i], global_counter);
store_data(ppg_record, sizeof(ppg_record), ppg);
```

Keep the record local to the loop or otherwise ensure it cannot be overwritten before `store_data()` copies it. In current `main`, `store_data()` immediately copies the supplied bytes into its own aggregation buffer.

Add a compile-time size assertion near the encoder:

```c
BUILD_ASSERT(PPG_NAND_RECORD_SIZE == 16U, "PPG NAND record must be 16 bytes");
```

## Canonical decoders

### C decoder

```c
struct ppg_nand_record {
	uint32_t ir1;
	uint32_t ir2;
	uint32_t green1;
	uint32_t green2;
	uint32_t global_tick_512hz;
};

static uint32_t ppg_get_u24_le(const uint8_t *src)
{
	return ((uint32_t)src[0]) |
	       ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16);
}

static uint32_t ppg_get_u32_le(const uint8_t *src)
{
	return ((uint32_t)src[0]) |
	       ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16) |
	       ((uint32_t)src[3] << 24);
}

static bool ppg_decode_nand_record(const uint8_t src[16],
				   struct ppg_nand_record *out)
{
	uint32_t ir1 = ppg_get_u24_le(&src[0]);
	uint32_t ir2 = ppg_get_u24_le(&src[3]);
	uint32_t green1 = ppg_get_u24_le(&src[6]);
	uint32_t green2 = ppg_get_u24_le(&src[9]);

	/* Reject malformed records whose unused upper five channel bits are set. */
	if (((ir1 | ir2 | green1 | green2) & ~0x7FFFFU) != 0U) {
		return false;
	}

	out->ir1 = ir1;
	out->ir2 = ir2;
	out->green1 = green1;
	out->green2 = green2;
	out->global_tick_512hz = ppg_get_u32_le(&src[12]);
	return true;
}
```

### Python decoder for `DataExtraction/data_extraction.py`

Python's `struct` module has no three-byte integer format, so the existing generic list of six `"<i"` fields cannot represent this record. Add a dedicated PPG decoder rather than extending `struct_key` with a fake type.

```python
PPG_RECORD_SIZE = 16
PPG_SAMPLE_MASK = 0x7FFFF


def decode_ppg_record(record: bytes) -> tuple[int, int, int, int, int]:
    if len(record) != PPG_RECORD_SIZE:
        raise ValueError(f"PPG record must be {PPG_RECORD_SIZE} bytes")

    ir1 = int.from_bytes(record[0:3], "little", signed=False)
    ir2 = int.from_bytes(record[3:6], "little", signed=False)
    green1 = int.from_bytes(record[6:9], "little", signed=False)
    green2 = int.from_bytes(record[9:12], "little", signed=False)
    tick = int.from_bytes(record[12:16], "little", signed=False)

    if (ir1 | ir2 | green1 | green2) & ~PPG_SAMPLE_MASK:
        raise ValueError("PPG channel has nonzero reserved bits")

    return ir1, ir2, green1, green2, tick


def decode_ppg_file(data: bytes):
    # NAND files may have trailing erased/preallocated bytes. Trim only complete
    # erased records from the end; do not remove arbitrary 0xFF bytes from a
    # valid tick or sample field.
    while len(data) >= PPG_RECORD_SIZE and data[-PPG_RECORD_SIZE:] == b"\xFF" * PPG_RECORD_SIZE:
        data = data[:-PPG_RECORD_SIZE]

    if len(data) % PPG_RECORD_SIZE != 0:
        raise ValueError(
            f"PPG file has {len(data)} bytes after trimming; not divisible by {PPG_RECORD_SIZE}"
        )

    return [
        decode_ppg_record(data[offset:offset + PPG_RECORD_SIZE])
        for offset in range(0, len(data), PPG_RECORD_SIZE)
    ]
```

CSV columns for this format should be exactly:

```text
ir1, ir2, g1, g2, global_tick_512hz
```

Do not retain the old per-record `Timestamp` column. If a wall-clock start time is available from the file name or collection metadata, derive a presentation timestamp separately from the tick; do not alter the decoded raw fields.

The existing `calculate_file_end()` removes arbitrary trailing `0xFF` bytes and can cut bytes from a valid final record. The packed decoder must use record-aware trimming as shown above. If actual FAT files are truncated to written length rather than retaining preallocated erased records, no trimming is needed.

## Golden test vector

```text
IR1    = 0x000001
IR2    = 0x012345
Green1 = 0x07FFFF
Green2 = 0x000100
Tick   = 0x12345678

Expected 16 bytes:
01 00 00 45 23 01 FF FF 07 00 01 00 78 56 34 12
```

Both firmware encoder tests and host decoder tests must use this vector. Decoding the expected bytes must reproduce all five inputs exactly, and encoding those inputs must reproduce every byte exactly.

Also test:

- Every channel at `0`, `1`, `0xFF`, `0x100`, `0xFFFF`, `0x10000`, and `0x7FFFF`.
- Tick values `0`, `1`, `0xFF`, `0x100`, `0xFFFF`, `0x10000`, `0xFFFFFF`, `0x1000000`, and `0xFFFFFFFF`.
- Rejection of a channel encoding with bit 19 or higher set, for example `00 00 08`.
- A multi-record byte stream to verify exact 16-byte boundaries and field order.
- A tick transition from `0xFFFFFFFF` to `0` without treating it as corruption.
- A file with complete trailing all-`0xFF` records and a file whose last valid tick contains `0xFF` bytes.

## Files to change

The implementation should be limited to the format migration and its decoder:

- `MSenseDevice/src/ppgSensor.c`: replace the 20-byte NAND record construction with the hard-coded encoder.
- `MSenseDevice/src/zephyrfilesystem.c`: change the PPG `sensor_format` text to describe four little-endian 24-bit channel fields followed by a little-endian 32-bit 512 Hz tick.
- `DataExtraction/data_extraction.py`: route PPG files through the dedicated 16-byte decoder and emit the five CSV columns above.
- Focused encoder/decoder tests, placed with the repository's existing extraction or validation tooling.

Do not modify `MSenseDevice/Kconfig`, `MSenseDevice/prj.conf`, BLE payloads, or SDK files.

`uuid.txt` is only created when absent. Updating `sensor_format` describes clean/newly formatted storage but does not replace a stale `uuid.txt` on an upgraded device. Report this limitation; do not add an erase or metadata migration to this task.

## Data rate and capacity

At the unchanged rates:

- PPG: 256 records/s x 16 bytes = 4,096 bytes/s.
- IMU: 32 records/s x 26 bytes = 832 bytes/s.
- Combined sensor payload: 4,928 bytes/s.

Exactly 4 GiB provides a payload-only upper bound of approximately 242.1 hours, or 10 days 2 hours 5 minutes. Filesystem metadata, allocation, and filesystem logging reduce actual duration.

## Verification and completion criteria

1. Confirm every NAND PPG submission is exactly 16 bytes and there is no remaining 20-byte PPG write path.
2. Confirm the golden vector passes in both directions.
3. Run focused encoder/decoder boundary and malformed-input tests.
4. Decode a multi-record generated file and verify channel order, tick values, record count, and CSV columns.
5. Confirm PPG remains 256 Hz and IMU output remains unchanged.
6. Confirm the BLE PPG packet path and BLE constants are untouched.
7. Confirm `uuid.txt` format text exactly matches the new layout on a clean filesystem.
8. Review the final diff against `main`; reject unrelated cleanup or refactoring.
9. Compile the firmware after receiving explicit build approval, using the project-local `tools/ncs-build.ps1` wrapper. A pristine full build also requires explicit approval.

Do not use this work to fix existing logging, BLE, USB, storage-capacity, NAND, or style issues.
