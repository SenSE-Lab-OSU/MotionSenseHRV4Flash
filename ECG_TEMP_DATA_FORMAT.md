# MAX30001 ECG Data Format

The ECG recorder writes fixed-size 12-byte binary frames. Frames are designed to be easy to resynchronize after dropped or partial bytes by scanning for the two-byte sync word.

## Frame Layout

| Byte(s) | Field | Description |
| --- | --- | --- |
| 0 | `sync0` | Constant `0xA5` |
| 1 | `sync1` | Constant `0xEC` |
| 2 | `type` | `0x01` for a MAX30001 ECG sample frame |
| 3 | `flags` | Bits `[2:0]` = `ETAG`, bits `[5:3]` = `PTAG`, bits `[7:6]` reserved |
| 4-7 | `seq_le` | 32-bit sample sequence number, little-endian |
| 8-10 | `raw24` | Raw MAX30001 ECG FIFO word, MSB first |
| 11 | `crc8` | CRC-8 over bytes 2 through 10 |

## CRC

CRC field:

- Polynomial: `0x07`
- Initial value: `0x00`
- Input bytes: frame bytes `2..10` inclusive
- No reflection or final XOR

The sync bytes are not included in the CRC.

## Decoding Notes

- Each complete frame is exactly 12 bytes.
- Host tools should discard partial trailing frames.
- If alignment is unknown, scan for `0xA5 0xEC`, verify `type == 0x01`, then validate CRC before accepting the frame.
- `seq_le` increments once per time-valid ECG sample stored by firmware.
- `raw24` is preserved directly from the MAX30001 ECG FIFO so host-side tools can decode the ECG sample and tag bits using the datasheet rules.

Current firmware implementation: `MSenseDevice/src/ecgRecorder.c`.
