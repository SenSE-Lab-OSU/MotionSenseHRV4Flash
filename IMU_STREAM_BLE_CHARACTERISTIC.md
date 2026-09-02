# Real-time IMU stream BLE characteristic (demo)

A notify-only BLE GATT characteristic that streams live accelerometer data plus a
per-frame gyro delta-rotation quaternion at 32 Hz while the device is collecting.
Added to demonstrate real-time IMU access alongside the existing file-logging
pipeline; it does not replace or modify file storage.

## Location

Part of the always-compiled `update_service` (`MSenseDevice/src/BLEService.c`),
alongside the existing ENMO notify characteristics. Unlike the legacy
`ACC_GYRO_TX` characteristic in `data_service`, this path is independent of
`CONFIG_MSENSE3_BLUETOOTH_DATA_UPDATES` and does not share a work item with
ENMO reporting.

| | |
|---|---|
| Service UUID (`update_service`) | `DA39C950-1D81-48E2-9C68-D0AE4BBD351F` |
| Characteristic UUID | **`DA39C953-1D81-48E2-9C68-D0AE4BBD351F`** |
| Properties | Notify |
| Permissions | None (subscribe via CCCD to receive) |
| Payload size | 20 bytes, fixed |
| Rate | 32 Hz (one notification per IMU output record) |

The raw byte array as declared in `MSenseDevice/src/BLEService.h`:

```c
#define NOTIFY_IMU_STREAM_CHARACTERISTIC_UUID 0x1F, 0x35, 0xBD, 0x4B, 0xAE, 0xD0, 0x68, 0x9C, \
  0xE2, 0x48, 0x81, 0x1D, 0x53, 0xC9, 0x39, 0xDA
```

## When it streams

Notifications are only sent while a client is subscribed (CCCD written to
`0x0001`) **and** the device is actively collecting
(`start_stop_device_collection(true)` — i.e. the same 512 Hz RTC tick that
drives file logging). There is currently no streaming-only mode that runs
the IMU without also writing to flash.

## Packet layout

All multi-byte fields are little-endian. Built in
`motion_data_timeout_handler()` (`MSenseDevice/src/imuSensor.c`) from values
already computed for the on-flash record, then sent via `imu_stream_send()`
(`MSenseDevice/src/BLEService.c`).

| Offset | Size | Field | Type | Notes |
|---:|---:|---|---|---|
| 0 | 2 | Accel X (raw) | `int16` | Divide by the active accel sensitivity divisor (e.g. `1/8192` at ±4g) to get g |
| 2 | 2 | Accel Y (raw) | `int16` | same |
| 4 | 2 | Accel Z (raw) | `int16` | same |
| 6 | 4 | Delta-rotation quaternion q0 | `float32` | see below |
| 10 | 4 | Delta-rotation quaternion q1 | `float32` | see below |
| 14 | 4 | Delta-rotation quaternion q2 | `float32` | see below |
| 18 | 2 | Packet counter | `uint16` | low 16 bits of the 512 Hz `global_counter` |

**This is not an absolute orientation.** `quaternionResult_1`
(`MSenseDevice/src/imuSensor.c`) is reset to identity `{0,0,0,1}` immediately
after each 32 Hz readout and then re-accumulated from the 16 gyro samples
(512 Hz / 32 Hz) taken during the *next* output period
(`gyroscope_measurement()`, `motion_data_timeout_handler()`). So each
transmitted quaternion is the net incremental rotation over that single
~31.25 ms window, not orientation relative to device power-on or any other
fixed reference. There is also no accelerometer/magnetometer fusion — this is
pure gyro dead-reckoning integration for one window. A consumer that wants a
running absolute orientation must compose (quaternion-multiply) these
per-frame deltas across frames itself; treating them as absolute orientation
directly is incorrect.

**q3 is not transmitted.** The firmware always keeps q3 non-negative for each
delta quaternion (`gyroscope_measurement()` flips the sign of the whole
quaternion when `q3 < 0`), so it can be reconstructed on the receiving side:

```
q3 = sqrt(max(0, 1 - q0^2 - q1^2 - q2^2))
```

This matches the quaternion component count already written to flash in the
IMU's on-disk record — this characteristic is a repackaging of values the
firmware already computes, not a new derived signal.

## Decoding

### Python (e.g. with `bleak` for the BLE transport)

```python
import struct

def decode_imu_stream_packet(data: bytes):
    if len(data) != 20:
        raise ValueError(f"IMU stream packet must be 20 bytes, got {len(data)}")

    acc_x, acc_y, acc_z, q0, q1, q2, counter = struct.unpack("<hhhfffH", data)

    q3_sq = 1.0 - q0 * q0 - q1 * q1 - q2 * q2
    q3 = q3_sq ** 0.5 if q3_sq > 0 else 0.0

    return {
        "accel_raw": (acc_x, acc_y, acc_z),
        "delta_quaternion": (q0, q1, q2, q3),  # incremental rotation for this ~31.25ms frame, not absolute orientation
        "counter": counter,
    }
```

### C

```c
struct imu_stream_record {
    int16_t accel_x, accel_y, accel_z;
    float q0, q1, q2; /* delta-rotation quaternion for this frame, not absolute orientation */
    uint16_t counter;
};

static void decode_imu_stream_packet(const uint8_t data[20], struct imu_stream_record *out)
{
    memcpy(&out->accel_x, &data[0], 2);
    memcpy(&out->accel_y, &data[2], 2);
    memcpy(&out->accel_z, &data[4], 2);
    memcpy(&out->q0, &data[6], 4);
    memcpy(&out->q1, &data[10], 4);
    memcpy(&out->q2, &data[14], 4);
    memcpy(&out->counter, &data[18], 2);
}
```

## Accel scaling

Raw accel counts are converted to g the same way the firmware does
internally (`imuSensor.c`, `motion_data_timeout_handler()`), based on the
active `accelConfig.sensitivity`:

| Sensitivity | Divisor |
|---|---:|
| ±2g | 16384 |
| ±4g (default) | 8192 |
| ±8g | 4096 |
| ±16g | 2048 |

```
accel_g = accel_raw / divisor
```

## Bandwidth

32 Hz x 20 bytes ≈ 640 B/s (≈5.1 kbps) — well within standard BLE
notification throughput at default connection parameters; no MTU
renegotiation or connection-interval tuning is required for this
characteristic alone.

## Relevant source

- `MSenseDevice/src/BLEService.h` — UUID definition, `imu_stream_send()` declaration.
- `MSenseDevice/src/BLEService.c` — characteristic/CCC declaration in `update_service`, `imu_stream_send()`.
- `MSenseDevice/src/imuSensor.c` — `imuStreamPkt` buffer, packet fill + send in `motion_data_timeout_handler()`.
