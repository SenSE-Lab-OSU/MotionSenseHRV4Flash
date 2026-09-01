# Phase 2: shared module scaffold

Phase 2 establishes a single Zephyr module named `msense-shared` without
changing product configuration, record formats, product identities, or sensor
initialization policy. The module is registered through
`shared/zephyr/module.yml`, contributes one `Kconfig` entry point and CMake
library, and exposes its generic devicetree bindings through its DTS root.

## Moved common implementation

The following files were byte-identical in the PPGv2 and ECGv0 Phase 0
baselines and now have one maintained copy under `shared/`:

- the Dhara sources and headers in `shared/drivers/dhara/`;
- the custom NOR sources and headers in `shared/drivers/nor/`;
- the NAND bad-page helper and its generic support headers in
  `shared/drivers/nand/`;
- `sample_flash_driver.c`;
- the `senselab,nanddisk`, `senselab,qspinor`, `senselab,spinor`, and
  `vnd,customflash` bindings in `shared/dts/bindings/`.

The raw NAND transport (`spi_nand` and `nand_disk`) remains product-local for
the Phase 3 geometry reconciliation. The shared bad-page helper uses a narrow
transport declaration so it does not select either product's NAND
implementation. MAX30001 and its binding remain ECGv0-only. The textual ICM-
20948 binding variants also remain product-local; their common physical wiring
is already described by the common board DTSI and their retained C drivers are
intentionally separate.

Both applications list `shared` in `ZEPHYR_EXTRA_MODULES` before
`find_package(Zephyr ...)`. Their application CMake files now enumerate source
files explicitly. ECGv0 deliberately does not compile `ppgSensor.c`; its
BLE-visible legacy PPG configuration values are retained in
`ble_ppg_compat.c`, which does not initialize or access PPG hardware.

## Build validation

Both final incremental NCS 2.9.3 builds completed against the supplied,
pre-patched `C:\\ncs\\SenSEv2.9.3` workspace:

| Product | Board | Build log | App signed binary | MCUboot binary | HCI IPC binary |
| --- | --- | --- | --- | --- | --- |
| PPGv2 | `ppgv2/nrf5340/cpuapp` | `D:\\senselab-tools\\logs\\ncs-build-20260828-204150-049.log` | `PPGv2/build/PPGv2/zephyr/zephyr.signed.bin` | `PPGv2/build/mcuboot/zephyr/zephyr.bin` | `PPGv2/build/hci_ipc/zephyr/zephyr.bin` |
| ECGv0 | `ecgv0/nrf5340/cpuapp` | `D:\\senselab-tools\\logs\\ncs-build-20260828-204458-243.log` | `ECGv0/build/ECGv0/zephyr/zephyr.signed.bin` | `ECGv0/build/mcuboot/zephyr/zephyr.bin` | `ECGv0/build/hci_ipc/zephyr/zephyr.bin` |

Each generated application configuration contains exactly one
`CONFIG_ZEPHYR_MSENSE_SHARED_MODULE=y` entry and keeps
`CONFIG_DISK_DRIVER_RAW_NAND=y`. Their generated compile commands include the
shared bad-page source; PPGv2 alone compiles `ppgSensor.c`, and ECGv0 alone
compiles `drivers/ecg/max30001.c`. The existing `msense_git_metadata` CMake
test passed for both generated application build directories. No hardware
validation was performed for this path/build-system-only phase.
