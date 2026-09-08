# Consolidation Phase 4 — subsystem reconciliation ledger

## Scope, baseline, and safety boundary

Phase 4 starts from firmware commit
`03949b577e5dbea0b35d5c8bd657c590bf67acb1` on
`codex/ppgv2-ecgv0-consolidation`. It reconciles only build provenance and
identity, battery monitoring, BLE/USB boundaries, sysbuild child images, and
small product-neutral helpers. The products remain two separate Zephyr
applications with separate `main.c` files.

The starting working tree contained owner documentation updates to
`FIRMWARE_CONSOLIDATION_HANDOFF.md` and `docs/consolidation-phase3.md`. They
are pre-existing, uncommitted changes and are deliberately not part of any
Phase 4 functional commit. They make baseline builds correctly report a dirty
tracked tree.

No media was formatted, erased, repaired, or otherwise accessed. No firmware
was flashed and no hardware test was performed during this phase.

## Sources reviewed

| Source commit | Product / line | Relevant intent | Phase 4 classification |
| --- | --- | --- | --- |
| `9a0a012` | PPG / main | Generate Git metadata and add it to `uuid.txt`. | Shared mechanism; preserve file contents and metadata fields. |
| `ab3b816` | PPG / main | Derive a stable factory-ID suffix and advertise it with the PPG prefix. | Shared algorithm with PPG-provided external name/model. |
| `940149d` | PPG / main | 300 mAh BQ27441 electrical configuration and status-unit corrections. | PPG-local battery policy and board facts. |
| `8e63285` | ECG | Adapt stable identity and provenance for ECG without changing its external identity. | Shared algorithm with ECG-provided external name/model. |
| `a1f0c91` | ECG | 200 mAh BQ27441 configuration and periodic maintenance. | ECG-local battery policy and board facts. |
| `f56d440` | ECG | Intentional ECG gauge rewrite: power-flow semantics, recovery visibility, and periodic summary policy. | ECG-local behavior; do not replace with PPG code. |

The historical changes were compared with the current consolidated source;
they are evidence of intent, not patches to cherry-pick. `1084876` and
`03949b5` remain the accepted Phase 3 storage/MSC and ECG compatibility
baseline and are not reimplemented here.

## Baseline build and provenance gate

The following fresh, product-specific builds completed before Phase 4 source
changes. They use the shared wrapper and leave existing build directories
untouched.

| Product | Board | Result | Wrapper log | Generated provenance |
| --- | --- | --- | --- | --- |
| PPGv2 | `ppgv2/nrf5340/cpuapp` | Pass | `D:\senselab-tools\logs\ncs-build-20260901-030439-939.log` | commit `03949b577e5dbea0b35d5c8bd657c590bf67acb1`, tree `dirty` |
| ECGv0 | `ecgv0/nrf5340/cpuapp` | Pass | `D:\senselab-tools\logs\ncs-build-20260901-030719-088.log` | commit `03949b577e5dbea0b35d5c8bd657c590bf67acb1`, tree `dirty` |

The generated headers are respectively under
`PPGv2/build-phase4-baseline/PPGv2/generated/` and
`ECGv0/build-phase4-baseline/ECGv0/generated/`. The tree state is expected:
only tracked documentation files were modified before these builds.

Both application-child CTest directories passed `msense_git_metadata` and
`msense_storage_geometry`. The sysbuild roots intentionally have no tests;
the application children own these CTests.

## BLE and USB comparison before service changes

This table is the Phase 4 reference for any future edit of `BLEService.c`, its
headers, or BLE-adjacent orchestration. UUID byte arrays use the shared
`1F35BD4B-AED0-689C-E248-811DxxC939DA` family, where `xx` is shown below in
source byte order for concise comparison.

| Dimension | PPGv2 | ECGv0 | Decision / preserved contract |
| --- | --- | --- | --- |
| Advertised/GAP name | `MSense4PPG-` plus the existing five-character Crockford-base32 CRC suffix of the 8-byte hardware ID; 16 bytes total. | `MSense4ECG-` plus the same suffix algorithm; 16 bytes total. | Keep both prefixes, suffix algorithm, dynamic-name settings, scan-response placement, and service-data device-ID bytes. Never derive either from directory/project name. |
| DIS identity | Manufacturer `SenseLab`; model `5.1.4 UProd`. | Manufacturer `SenseLab`; model `5.0.2 NAND`. | Product-local values; unchanged. |
| Control service | Service `xx=30`; enable `31`, date `32`, patient `33`, reset `34`, and settings/manual-file `35`. | Same UUIDs and properties. | Keep the common UUID surface and separate dispatch implementations. ECG UUID `35` remains for manual test-file commands, not PPG compatibility. |
| Status service | Service `40`; storage `41`, status `42`, uptime `43`; read/notify as defined in source. | Same UUIDs and properties. | Preserve exact read/notify layouts and status ordering. |
| Product stream / update service | Data service `20`: PPG notify `23`, accelerometer/gyro notify `24`, descriptors `27`/`28`; update service `50` with `51` and `52`. | Timing-update service `50` with notify characteristic `51`; no vestigial PPG acquisition/configuration surface. | Product GATT definitions remain local. Do not force a shared service definition or restore ECG PPG paths. |
| Control payloads | Enable: one byte; datetime: 8-byte value; patient: 4-byte integer; reset: one byte (`68`, `132`, `121`); settings/manual file preserves PPG brightness and supported file-test values. | Same enable/date/patient/reset lengths and reset values; UUID `35` accepts one byte `130` or `150` for active ECG manual files. | Preserve payload lengths, command values, storage-transition routing, and failure responses. |
| Connection preferences | 24–48 units, latency 3, timeout 800 units. | 24–40 units, latency 0, timeout 800 units. | Product-local connection policy. |
| PHY/data length | Generated application config enables automatic PHY and data-length updates; controller supports 2M/coded PHY. | Explicit application auto PHY/data-length updates; HCI IPC enables 2M/coded PHY. | Preserve each effective configuration; no normalization. |
| Controller TX power | HCI IPC configuration selects +3 dBm. | HCI IPC configuration retains 0 dBm. | Product-local radio fact; verify generated HCI configs. |
| Storage and USB | Uses accepted Phase 3 firmware/host ownership model; collection transition owns host-media publication. | Same shared MSC-media mechanism, with ECG-local transition/fault orchestration. | Preserve no-medium/read-only host publication, filesystem teardown before host access, and logging/collection ordering. |
| Ship mode | No physical user button and no ship-mode path. | Active-low P0.01 button; ECG-only ship-mode state flow retains MSC availability. | Keep ECG-only. Do not add a PPG button or ship mode. |
| Identity/build fields | `uuid.txt` is created during the boot filesystem session and includes BLE address, name, hardware ID, model, Git commit/tree state, and PPG/accelerometer format fields. | Same boot-session timing, with ECG format field. | Keep filename, timing, text labels, format fields, and existing recording metadata/filenames unchanged. |

## Reconciliation decisions

### Shared mechanisms

- Git metadata generation and its CTest are byte-identical in both products.
  A shared CMake mechanism may own the script and test while each application
  supplies its source and binary directory.
- The stable nRF5340 hardware-ID algorithm is byte-identical except for the
  externally visible product name prefix. A shared implementation must take
  explicit product identity parameters; it must not infer an identity from
  `PPGv2`, `ECGv0`, a board directory, or a CMake project name.
- The existing shared MSC-media implementation remains accepted as-is. Phase
  4 does not replace the Phase 3 ownership model.
- Both applications use MCUboot and the HCI IPC child image. The shared
  `shared/cmake/msense_sysbuild.cmake` helper conditionally attaches an
  app-provided fragment only when `SB_CONFIG_NETCORE_HCI_IPC` selected the
  child image. Generated NCS 2.9.3 outputs prove that each product's own
  fragment is still applied.

### Deliberately product-local

- `main.c`, MAX86141 PPG acquisition, MAX30001 ECG acquisition, record formats,
  recording names, and collection orchestration.
- Both ICM-20948 implementations and their timing/recorder modules.
- BQ27441 configuration and monitoring: PPG keeps its 300 mAh / BQ25060
  assumptions and simpler diagnostic path; ECG keeps its 200 mAh / BQ21040
  configuration, `f56d440` power-flow reporting, ITPOR visibility, and
  periodic-summary scheduling. A shared module would alter policy or
  concurrency semantics, so none is extracted in Phase 4.
- BLE service definitions, command dispatch, product stream notifications,
  connection preferences, TX power, and ship-mode behavior.
- Per-product HCI IPC configuration fragments and board clock/regulator facts.

## Battery-monitor comparison and decision

The battery monitor was compared at the function and scheduling level rather
than by file name. The NCS BQ274xx binding confirms that design capacity and
taper current are board electrical properties, expressed in mAh and mA; the
custom-board DTS files are therefore the source of those facts.

| Concern | PPGv2 | ECGv0 | Reconciliation decision |
| --- | --- | --- | --- |
| Board configuration | BQ27441 at I2C address `0x55`, standard-mode I2C, 3700 mV, 300 mAh, 30 mA taper, 3000 mV terminate, chemistry `0x0128`; BQ25060 threshold context. | BQ27441 at `0x55`, standard-mode I2C, 3700 mV, 200 mAh, 9 mA taper, 3000 mV terminate; BQ21040 threshold context. | Keep entirely board-local. Do not parameterize these electrical facts in a generic application module. |
| Scheduling | `battery_maintenance()` calls `dt_update_battery(dev, true)` from the existing periodic main loop. | The same loop calls maintenance, but only every fourth maintenance cycle requests a summary. | Preserve each cadence and callback context. |
| Normal sampling | Fetches voltage, average current, max-load current, state of charge, then optional diagnostic channels. | Fetches voltage first to trigger/recover driver configuration, then flags, average current, state of charge, and average power. | Different read ordering has recovery semantics in ECG; no common read sequence. |
| Charge-state policy | Uses the fuel-gauge Flags register bit-zero rule and emits its established diagnostics. | Uses AveragePower sign, exposes ITPOR/reset uncertainty, maintains charging/discharging/idle state, and logs state transitions. | Keep the intentional `f56d440` ECG behavior intact. It must not be replaced with the PPG policy. |
| BLE/status effects | Updates BAS state and PPG status notification behavior from the established monitor. | Updates BAS state, only sends status notification on charging-state change, and carries ECG-specific power-flow logging. | Preserve separate reporting policy and notification timing. |
| Error/logging behavior | Existing `printk`-based diagnostic path, including additional debug reads. | `LOG_*` path with per-channel errors and a bounded periodic summary. | Do not create a shared abstraction that would change logging volume, recovery visibility, or timing. |

No battery source was changed in Phase 4. The source comparison established
that sharing only a helper would either obscure the different charge-state
semantics or add a speculative interface. This is an intentional product-local
decision, not an omitted extraction.

Required owner hardware checks are: plausible state of charge and capacity,
charging indication transitions, low-battery behavior, and the expected
charger-specific taper completion for both boards; for ECGv0 also verify the
ITPOR/reconfiguration indication and power-flow transition logs after a
gauge-reset scenario.

## Generated sysbuild evidence at baseline

- Application targets resolve to the custom boards
  `ppgv2/nrf5340/cpuapp` and `ecgv0/nrf5340/cpuapp`; their `BOARD_DIR` values
  are the matching `boards/senselab` directories.
- Both generated application configurations enable MCUboot and partition
  manager, and both output application, MCUboot, HCI IPC, and `merged.hex`
  artifacts.
- PPGv2 generated app/MCUboot configuration keeps LFXO crystal mode with 9 pF
  internal capacitance; ECGv0 keeps external LFXO capacitance and bypass with
  20 ppm selection. Both retain internal HFXO capacitor value 25 and generated
  DTS shows VREGMAIN/VREGRADIO LDO plus disabled VREGH.
- HCI IPC output is active in both products. Its generated configuration shows
  PPGv2 `CONFIG_BT_CTLR_TX_PWR_PLUS_3=y` / 3 dBm and ECGv0
  `CONFIG_BT_CTLR_TX_PWR_0=y` / 0 dBm. ECGv0's 2M and coded PHY options are
  present in its generated network-core configuration.

## Commits created during Phase 4

| Commit | Scope | Validation recorded |
| --- | --- | --- |
| `bbbaf74c67b4777e5197fcd4bdbf866b15d640f3` | Shared Git-provenance generator/CTest and stable hardware-ID mechanism with explicit product BLE prefix/model parameters. | Both wrapper builds passed; both `msense_git_metadata` and `msense_storage_geometry` CTests passed; generated headers embed this commit and report the expected dirty source tree. |
| `cd913de` | Ledger checkpoint recording the battery function-level comparison and intentional product-local decision. | The prior identity build artifacts were revalidated with both product wrapper builds; no battery source was changed. |
| `b1e7de0` | Static Phase 4 BLE/USB compatibility test, registered in both application CTest suites. | Both products build and all three tests pass: Git metadata, storage geometry, and `msense_phase4_contracts`. |
| `ee2173ef5d9877dce284d121a4cdd58e92b4c9f8` | Shared conditional HCI IPC-overlay helper with explicit product fragment paths. | Both wrapper builds, all three CTests per product, generated configurations, partition maps, and child artifacts pass inspection. This is the final functional firmware commit. |

## Sysbuild reconciliation result

`shared/cmake/msense_sysbuild.cmake` centralizes only the NCS 2.9.3
mechanism: call `add_overlay_config(hci_ipc ...)` when
`SB_CONFIG_NETCORE_HCI_IPC` is selected. `PPGv2/sysbuild.cmake` and
`ECGv0/sysbuild.cmake` each pass their own
`child_image/hci_ipc.conf` path, so no controller, PHY, power, clock, or board
fact was merged or normalized.

Generated final outputs at `ee2173e` confirm all of the following:

- Both application configurations set `CONFIG_BOOTLOADER_MCUBOOT=y` and
  `CONFIG_BT_HCI_IPC=y`; the HCI IPC child image and MCUboot image are built.
- Both final partition maps retain MCUboot at `0x25000`, matching primary and
  secondary image slots of `0x6c000`, and an application image size of
  `0x6be00`.
- Both applications retain custom flash mapping and a present external-flash
  driver (`CONFIG_FLASH_MAP_CUSTOM=y`,
  `CONFIG_PM_EXTERNAL_FLASH_HAS_DRIVER=y`, and QSPI layout page size 4096).
- PPGv2 retains crystal LFXO with 9 pF internal capacitance, internal HFXO
  capacitor value 25, +3 dBm HCI-controller TX power, and 2M/coded PHY.
- ECGv0 retains bypass/external LFXO, internal HFXO capacitor value 25, 0 dBm
  HCI-controller TX power, and 2M/coded PHY.
- Generated DTS retains `VREGMAIN` and `VREGRADIO` at initial mode `0` (LDO)
  and `VREGH` disabled for both products.

## Common-utility review

No additional utility was extracted after the provenance/identity and sysbuild
mechanisms:

- `PPGv2/src/common.h` contains PPG packet lengths, PPG packet buffers, and
  PPG-specific globals; ECGv0 has no counterpart. It remains local.
- Filesystem timer helpers use different APIs and ownership semantics: PPG
  passes a timer reference and emits its existing diagnostics, while ECG owns
  a product-local timer. Extracting a helper would change callback/context
  expectations.
- Logging and filesystem policy remain coupled to each product's recording,
  storage-owner, and fault-reporting behavior. The accepted Phase 3 shared
  MSC-media backend remains unchanged.

This completes the required common-utility review without creating a generic
miscellaneous module or changing timer, logging, recording, or storage policy.

## Final clean functional validation

The starting checkout necessarily remained dirty because of the two preserved
owner documentation edits. To verify a genuinely clean firmware tree, a
detached validation worktree was created at
`D:\MotionSenseHRV4Flash-phase4-validation` at final functional commit
`ee2173ef5d9877dce284d121a4cdd58e92b4c9f8`. No owner file was moved, reset,
or edited.

| Product | Fresh wrapper build | CTest result | Generated provenance |
| --- | --- | --- | --- |
| PPGv2 | Pass — `D:\senselab-tools\logs\ncs-build-20260901-035607-297.log` | 3/3 pass: `msense_git_metadata`, `msense_storage_geometry`, `msense_phase4_contracts` | `MSENSE_GIT_COMMIT "ee2173ef5d9877dce284d121a4cdd58e92b4c9f8"`; `MSENSE_GIT_TREE_STATE "clean"` |
| ECGv0 | Pass — `D:\senselab-tools\logs\ncs-build-20260901-035828-184.log` | 3/3 pass: `msense_git_metadata`, `msense_storage_geometry`, `msense_phase4_contracts` | `MSENSE_GIT_COMMIT "ee2173ef5d9877dce284d121a4cdd58e92b4c9f8"`; `MSENSE_GIT_TREE_STATE "clean"` |

The successful final build commands use the shared wrapper with separate
build directories:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash-phase4-validation\PPGv2' `
  -Board 'ppgv2/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash-phase4-validation\PPGv2\build-phase4-final'

powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash-phase4-validation\ECGv0' `
  -Board 'ecgv0/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash-phase4-validation\ECGv0\build-phase4-final'
```

The application-child CTest command was run for each corresponding final
directory with `C:\ncs\toolchains\b620d30767\opt\bin\ctest.exe --test-dir
<application-child-build-dir> --output-on-failure`, under the wrapper's
temporary Git-trust environment.

Final signed application artifacts, MCUboot artifacts, HCI IPC artifacts, and
merged image are present at:

- `D:\MotionSenseHRV4Flash-phase4-validation\PPGv2\build-phase4-final\PPGv2\zephyr\zephyr.{elf,hex,bin,signed.hex,signed.bin}`,
  `...\mcuboot\zephyr\zephyr.{elf,hex,bin}`,
  `...\hci_ipc\zephyr\zephyr.{elf,hex,bin}`, and `...\merged.hex`.
- `D:\MotionSenseHRV4Flash-phase4-validation\ECGv0\build-phase4-final\ECGv0\zephyr\zephyr.{elf,hex,bin,signed.hex,signed.bin}`,
  `...\mcuboot\zephyr\zephyr.{elf,hex,bin}`,
  `...\hci_ipc\zephyr\zephyr.{elf,hex,bin}`, and `...\merged.hex`.

`msense_phase4_contracts` is a static compatibility gate. It checks the
accepted shared storage compilation path; Git metadata and `uuid.txt` source
contract; BLE names, DIS models, key UUID bytes, and product service shape;
the absence of ECG PPG compatibility/acquisition paths; MAX86141 versus
MAX30001 source selection; separate IMU implementations; PPG's lack of ship
mode/button wiring; ECG's ship-mode/button wiring; expected board clocks;
controller power/PHY settings; application/child artifacts; and MCUboot/HCI
IPC activation. It does not exercise hardware or change media.

The documentation evidence commit made after these builds intentionally does
not alter firmware. Therefore the final artifacts correctly embed `ee2173e`,
not the later documentation-only commit.

## Deferred findings and owner hardware acceptance

The known production-storage risks in `STORAGE_AUDIT.md` and
`docs/ecg-filesystem-corruption-review.md` are explicitly deferred. They were
not changed as part of Phase 4, including raw-NAND bad-block management,
automatic-format safety, power-loss-safe metadata, bounded NAND polling,
legacy async writer ownership, and storage endurance/fault coverage.

Owner smoke testing is still required after implementation, without flashing
or media access by this agent:

1. PPGv2: boot all images; verify `MSense4PPG-xxxxx`, DIS model, stable ID,
   PPG/IMU streaming, battery/charger behavior, connection parameters,
   PHY/data length, +3 dBm radio behavior, repeated collection/USB ownership
   transitions, `uuid.txt`, recording filenames/bytes, and no ship-mode path.
2. ECGv0: boot all images; verify `MSense4ECG-xxxxx`, DIS model, stable ID,
   ECG/IMU/timing notifications, 200 mAh battery/charger behavior, connection
   parameters, PHY/data length, 0 dBm radio behavior, repeated
   collection/USB ownership transitions, `uuid.txt`, recording filenames/bytes,
   and button-driven ship mode with mass storage available as intended.
3. For both: verify MCUboot/DFU layout, reset/filesystem logs, host
   no-medium/read-only behavior during firmware ownership, and that no existing
   client-visible UUID, packet layout, or recording decoder compatibility has
   changed.
