# MotionSenseHRV4 firmware

This repository contains two independently buildable nRF5340 firmware
products. They share only mechanisms with the same behavior and external
contract; they are not variants of one configurable application.

| Product | Purpose | Primary sensor | Board target |
| --- | --- | --- | --- |
| `PPGv2` | Optical physiological collection | MAX86141 | `ppgv2/nrf5340/cpuapp` |
| `ECGv0` | ECG and motion/timing collection | MAX30001 | `ecgv0/nrf5340/cpuapp` |

Both products also build MCUboot and an nRF5340 network-core HCI IPC image.
Keep the applications, their `main.c` files, product configuration, BLE
contracts, record encoders, and release artifacts separate.

## Required NCS environment

Build with the managed, project-patched nRF Connect SDK workspace at
`C:\ncs\SenSEv2.9.3` and Nordic toolchain at
`C:\ncs\toolchains\b620d30767`. Do not silently substitute a stock checkout
or a different NCS release.

Use the shared wrapper at `D:\senselab-tools\vscode-wrapper\ncs-build.ps1`.
It configures the required SDK/toolchain and temporary Git trust state. Run
builds serially; do not overlap PPGv2 and ECGv0 builds or share a build
directory.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash\PPGv2' `
  -Board 'ppgv2/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash\PPGv2\build-phase5'

powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash\ECGv0' `
  -Board 'ecgv0/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash\ECGv0\build-phase5'
```

Add `-Pristine` only when a clean build is intended. Each wrapper invocation
writes a timestamped log under `D:\senselab-tools\logs`; success means the
wrapper reports its final `Build exit code: 0`.

## Artifacts

For either `<product>` (`PPGv2` or `ECGv0`) and its separate
`<build-directory>`, expect:

```text
<build-directory>\<product>\zephyr\zephyr.{elf,hex,bin,signed.hex,signed.bin}
<build-directory>\mcuboot\zephyr\zephyr.{elf,hex,bin}
<build-directory>\hci_ipc\zephyr\zephyr.{elf,hex,bin}
<build-directory>\merged.hex
```

The application-child directory is product-specific, and the application
CMake cache must identify the matching project (`PPGv2` or `ECGv0`).

## Verification

Run the application-child CTests after a successful build:

```powershell
& 'C:\ncs\toolchains\b620d30767\opt\bin\ctest.exe' `
  --test-dir 'D:\MotionSenseHRV4Flash\PPGv2\build-phase5\PPGv2' `
  --output-on-failure

& 'C:\ncs\toolchains\b620d30767\opt\bin\ctest.exe' `
  --test-dir 'D:\MotionSenseHRV4Flash\ECGv0\build-phase5\ECGv0' `
  --output-on-failure

powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  'D:\MotionSenseHRV4Flash\tools\validate_phase5_build_matrix.ps1' `
  -PpgBuildDirectory 'D:\MotionSenseHRV4Flash\PPGv2\build-phase5' `
  -EcgBuildDirectory 'D:\MotionSenseHRV4Flash\ECGv0\build-phase5'
```

The CTests cover Git metadata, shared storage geometry, and the retained
compatibility contract. The build-matrix validator checks both custom boards,
the project names, MCUboot, HCI IPC, signed application artifacts, and
`merged.hex`.

## Record formats and compatibility

Recording bytes and file names are external contracts. Do not normalize the
PPG and ECG formats, change BLE names/UUIDs/DIS models, or alter the
boot-session `uuid.txt` labels or timing merely to align internal names.

- PPGv2 uses the documented 16-byte packed record in
  `PPGv2/PPG_PACKED_16_BYTE_FORMAT.md`; the encoder and `uuid.txt` format
  label are in `PPGv2/src/ppgSensor.c` and `PPGv2/src/zephyrfilesystem.c`.
- ECGv0 ECG frames are documented in `ECG_TEMP_DATA_FORMAT.md` and covered by
  `ECGv0/tests/ecg_record_format`.
- ECGv0 accelerometer files are documented in
  `ACCELEROMETER_BINARY_FORMAT.md` and covered by
  `ECGv0/tests/accel_record_format` and `ECGv0/tests/accel_recorder`.

## Shared and product-local code

`shared/` owns the common Zephyr module, storage mechanisms, stable identity
algorithm, build provenance, and sysbuild HCI IPC attachment mechanism. Board
definitions own hardware facts. Keep the following product-local unless a
future interface can preserve their behavior and external contract:

- each `main.c` and collection/storage transition ordering;
- MAX86141 PPG acquisition versus MAX30001 ECG acquisition;
- both separate ICM-20948 implementations;
- BLE/GATT service definitions, packet layouts, connection policy, and radio
  settings;
- battery/charger policy, record encoders, recording names, and ECG-only
  button-driven ship mode.

PPGv2 has no physical user button and must not gain ship-mode behavior. ECGv0
retains its MAX30001, separate IMU path, and button-driven ship-mode behavior.

## Adding a future third platform

Create a new product application and custom board target rather than copying a
complete existing product. Start with a separate `main.c`, CMake source list,
board description, sysbuild configuration, external identity, and record
format. Reuse a component from `shared/` only after its inputs and behavior are
explicitly product-neutral. Add the product/board/image entry to the local
build-matrix validator and compatibility tests, then validate it alongside the
two existing products.

Phase 5 evidence and deferred validation work are recorded in
`docs/consolidation-phase5.md`.
