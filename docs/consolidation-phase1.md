# Phase 1 custom-board migration

Phase 1 replaces the nRF5340 DK-named application overlays with two NCS 2.9.3
Hardware Model v2 board definitions:

| Product | Application target | Network target | Board directory |
| --- | --- | --- |
| PPGv2 | `ppgv2/nrf5340/cpuapp` | `ppgv2/nrf5340/cpunet` | `boards/senselab/ppgv2` |
| ECGv0 | `ecgv0/nrf5340/cpuapp` | `ecgv0/nrf5340/cpunet` | `boards/senselab/ecgv0` |

The nRF5340 SoC supplies the `cpuapp` and `cpunet` qualifiers; each `board.yml`
therefore declares only the nRF5340 SoC, while the matching board DTS and
defconfig files provide both target variants.

`PPGv2/sysbuild/CMakeLists.txt` and `ECGv0/sysbuild/CMakeLists.txt` publish
the repository `BOARD_ROOT` before Sysbuild discovers the application,
MCUboot, or HCI IPC child image. The application CMake files keep the same
early board root for direct builds. The shared build wrapper requires callers
to select a matching application root and custom board target, and no longer
passes a DK-named `DTC_OVERLAY_FILE`.

## Authoritative hardware inventory

The table below comes from the owner-confirmed Phase 1 inventory and the
imported overlays. Schematics and a BOM were not available to this migration.

| Hardware | PPGv2 | ECGv0 |
| --- | --- | --- |
| Primary SPI3 sensor | MAX86141, retained direct `ppgSensor` transport | MAX30001 with active-low INTB and INTB2 |
| ICM-20948 | SPI2; CS P0.26, INT P0.29 active-high, FSYNC P0.27 active-high | Same SPI2/CS/INT/FSYNC wiring |
| NAND | Four MT29 chip selects: P0.18, P0.04, P0.21, P0.19 | Two MT29 chip selects: P0.18, P0.04 |
| NAND population | Four 8-Gbit devices | Two 8-Gbit devices |
| NOR | MX25U80 on SPI0, CS P0.05 | Same |
| Battery / charger | 300 mAh, BQ25060 context, 30 mA taper | 200 mAh, BQ21040 context, 9 mA taper |
| User button | Not populated | P0.01 active-low, ECGv0 ship mode |
| HCI IPC TX power | +3 dBm | 0 dBm |
| LFXO | Crystal, 9 pF internal capacitance, bypass off | External source, bypass on through documented NCS patch |
| HFXO and regulators | Common HFXO capacitor setting; VREGMAIN/VREGRADIO LDO and VREGH disabled | Same |

Both boards model the ICM-20948 child and its interrupt/FSYNC GPIOs. The PPG
node is descriptive only: PPGv2 retains its direct SPI IMU implementation and
does not select a competing Zephyr driver. Product code now uses board aliases
for sensor buses, battery gauge, storage, GPIO controllers, and the ECG user
button. The old approximate PPG NAND size values were deliberately retained;
Phase 3 owns their replacement with one exact validated geometry model.

## NCS 2.9.3 build validation

Clean sysbuilds, followed by final incremental validations, completed on
2026-08-27 using the owner-supplied, managed and pre-patched
`C:\ncs\SenSEv2.9.3` workspace and
`C:\ncs\toolchains\b620d30767`. The SDK checkout and its patches were not
modified.

| Product | Board | Build log | App signed binary | MCUboot binary | HCI IPC binary |
| --- | --- | --- | --- | --- | --- |
| PPGv2 | `ppgv2/nrf5340/cpuapp` | `D:\senselab-tools\logs\ncs-build-20260828-003058-949.log` | `PPGv2/build/PPGv2/zephyr/zephyr.signed.bin` | `PPGv2/build/mcuboot/zephyr/zephyr.bin` | `PPGv2/build/hci_ipc/zephyr/zephyr.bin` |
| ECGv0 | `ecgv0/nrf5340/cpuapp` | `D:\senselab-tools\logs\ncs-build-20260827-204637-728.log` | `ECGv0/build/ECGv0/zephyr/zephyr.signed.bin` | `ECGv0/build/mcuboot/zephyr/zephyr.bin` | `ECGv0/build/hci_ipc/zephyr/zephyr.bin` |

Generated configuration confirmed `CONFIG_BT_HCI_IPC=y` in both application
images and `CONFIG_BT_CTLR=y` in their matching `hci_ipc` images. The child
image caches resolve to `ppgv2/nrf5340/cpunet` and
`ecgv0/nrf5340/cpunet`, respectively. Generated DTS confirms the custom USB
controller label, each product's aliases, LDO regulators with VREGH disabled,
four PPG NAND devices versus two ECG NAND devices, and the 300/30 mA versus
200/9 mA battery-gauge values. ECGv0's generated config has LFXO bypass and
external capacitance enabled; PPGv2 has the retained internal 9 pF setting.

On 2026-08-28, PPGv2's HCI IPC controller selection was corrected from the
non-selecting `CONFIG_BT_CTLR_TX_PWR_ANTENNA=3` setting to NCS 2.9.3's
`CONFIG_BT_CTLR_TX_PWR_PLUS_3=y`. The final pristine PPGv2 build generated
`CONFIG_BT_CTLR_TX_PWR_PLUS_3=y`, `CONFIG_BT_CTLR_TX_PWR_DBM=3`, and no
`CONFIG_BT_CTLR_TX_PWR_0`. Its new sysbuild hook explicitly passes the product
HCI IPC fragment to the automatic network-core image. ECGv0 retains the
default controller choice, with generated `CONFIG_BT_CTLR_TX_PWR_0=y` and
`CONFIG_BT_CTLR_TX_PWR_DBM=0`; its obsolete antenna-setting comment was
removed. The PPGv2 validation log is the build-log path recorded above.

The custom board includes the regulator facts formerly duplicated in each
`sysbuild/mcuboot.overlay`. The retained overlays now only disable
application-only SPI/I2C/USB nodes for the bootloader, which prevents MCUBoot
from linking unused external-flash and sensor drivers. The shared SenseLab
CPUAPP partition fragment uses equal 264 KiB MCUboot slots; the 256 KiB
reference slot overflowed by 1 KiB for PPGv2.

NCS still warns that the product has no `pm_static.yml`. The generated board
DTS has a fixed internal-flash layout, but release/DFU work must reconcile the
Partition Manager warning with the deployed bootloader map before production
updates are accepted; that is deferred to the later sysbuild/partition review.

On 2026-08-28, the owner reported successful hardware smoke tests on one
representative PPGv2 device and one representative ECGv0 device. Together with
the successful builds and generated-configuration review, this closes the
Phase 1 physical-acceptance gate. The detailed test procedure was not supplied,
so destructive storage endurance, production DFU compatibility, and exhaustive
recording-file compatibility remain governed by their later validation gates.
The known NAND geometry arithmetic warning remains deferred to the Phase 3
data-driven storage migration.

The root DK overlay was unreferenced after the product overlays and build
wrappers were converted, so it was removed. The NCS v2.9.3 LFXO-bypass patch
remains required for ECGv0 and is not applied to PPGv2.
