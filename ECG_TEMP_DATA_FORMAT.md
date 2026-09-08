# MAX30001 ECG Data Format

The ECG recorder writes fixed-size 12-byte binary frames. Frames are designed to be easy to resynchronize after dropped or partial bytes by scanning for the two-byte sync word.

## Frame Layout

| Byte(s) | Field | Description |
| --- | --- | --- |
| 0 | `sync0` | Constant `0xA5` |
| 1 | `sync1` | Constant `0xEC` |
| 2 | `type` | `0x01` for a MAX30001 ECG sample frame |
| 3 | `flags` | Bits `[2:0]` = `ETAG`, bits `[5:3]` = `PTAG`, bits `[7:6]` reserved |
| 4-7 | `rtc_tick_le` | 32-bit collection-local RTC0 tick, little-endian |
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
- `rtc_tick_le` is the 32-bit wrap-extended collection RTC0 tick associated
  with the filtered ECG sample at the MAX30001 `SAMP`/FIFO-output instant.
  RTC0 runs at 512 Hz, so an uninterrupted sequence of time-valid frames
  advances by one tick per frame and wraps naturally after `2^32` ticks.
- The timestamp is captured from the first post-`SYNCH` `SAMP` pulse on
  MAX30001 `INT2B`, then advanced in firmware while FIFO data is drained.
  FIFO batching therefore does not add timing delay to stored timestamps.
- These timestamps do not compensate MAX30001 ECG digital-filter group delay.
  They use the same output-timing convention as the IMU RTC anchors.
- Earlier firmware used this same type-`0x01` frame layout with `seq_le` in
  bytes 4-7. The two interpretations cannot be distinguished from frame bytes
  alone; host software must use firmware or collection provenance.
- `raw24` is preserved directly from the MAX30001 ECG FIFO so host-side tools can decode the ECG sample and tag bits using the datasheet rules.

Current firmware implementation: `ECGv0/src/ecgRecorder.c`.
