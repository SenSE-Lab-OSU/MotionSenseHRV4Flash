# MotionSenseHRV4 Firmware

Firmware for MotionSense physiological data-collection devices based on the
Nordic nRF5340 and nRF Connect SDK.

This repository contains two independently buildable products:

| Product | Purpose | Primary sensor | Board target |
| --- | --- | --- | --- |
| [`PPGv2`](PPGv2/) | Optical physiological data collection | MAX86141 | `ppgv2/nrf5340/cpuapp` |
| [`ECGv0`](ECGv0/) | ECG and motion data collection | MAX30001 | `ecgv0/nrf5340/cpuapp` |

The products share selected infrastructure under [`shared`](shared/), but have
separate application behavior, hardware definitions, BLE interfaces, and
recording formats.

## Requirements

- The project-maintained nRF Connect SDK 2.9.3 workspace and patch set
- A compatible Nordic toolchain
- `west`, CMake, and Ninja configured through the nRF Connect SDK environment

A stock or differently patched SDK checkout may not build or behave identically.

## Building

Build each application from an initialized nRF Connect SDK environment. Use a
separate build directory for each product:

```sh
west build --sysbuild -b ppgv2/nrf5340/cpuapp PPGv2 -d build/ppgv2
west build --sysbuild -b ecgv0/nrf5340/cpuapp ECGv0 -d build/ecgv0
```

Add `--pristine` when a clean configuration is required.

Each sysbuild includes the application, MCUboot, and the nRF5340 network-core
HCI IPC image. The combined image is written to:

```text
<build-directory>/merged.hex
```

See [`PPGv2/HowToDevelop.md`](PPGv2/HowToDevelop.md) and
[`ECGv0/HowToDevelop.md`](ECGv0/HowToDevelop.md) for product-specific
development information.

## Data formats

The recorded data formats are part of the device compatibility contract:

- [PPG packed record format](PPG_PACKED_16_BYTE_FORMAT.md)
- [ECG block format](ECG_BLOCK_FORMAT.md)
- [Accelerometer binary format](ACCELEROMETER_BINARY_FORMAT.md)
- [Sensor-stream firmware guide](SENSOR_STREAM_FIRMWARE_HOWTO.md)
- [Sensor-stream central guide](SENSOR_STREAM_CENTRAL_HOWTO.md)

## Repository layout

```text
PPGv2/    PPG application and product configuration
ECGv0/    ECG application and product configuration
boards/   Custom nRF5340 board definitions
shared/   Code and Zephyr module infrastructure shared by both products
docs/     Design notes, validation records, and technical documentation
```

## Testing

Host-side tests are located under each product's `tests/host` directory and
under `shared/tests/host`. Zephyr test applications and hardware-in-the-loop
utilities are kept alongside the product they validate.

Some integration tests require the project-maintained SDK patch set or physical
MotionSense hardware.
