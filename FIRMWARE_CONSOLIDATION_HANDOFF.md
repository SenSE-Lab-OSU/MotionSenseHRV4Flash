# PPGv2 / ECGv0 Firmware Consolidation Handoff

## Mission

Bring the firmware currently maintained on `main` and
`simp-nathan-ecg-yuyi200mAh` into one repository layout without turning them
into one application.

- Rename the application currently at `MSenseDevice` on `main` to `PPGv2`.
- Rename the application currently at `MSenseDevice` on
  `simp-nathan-ecg-yuyi200mAh` to `ECGv0`.
- Keep a separate `main.c`, application configuration, sensor pipeline,
  product identity, and release artifact for each product.
- Replace the modified nRF5340 DK application overlays with real out-of-tree
  board definitions for the two SenseLab hardware platforms.
- Share code only where the behavior and contract are genuinely common.
- Reconcile fixes made independently on both branches, with particular care
  around NAND, NOR, the FAT filesystem, filesystem logging, and USB mass
  storage behavior.
- Keep the two ICM-20948 implementations separate in this first consolidation.
- Rewrite the ECG-only development history into a compact, reviewable series
  before any firmware or repository-layout changes, while preserving the ECG
  tip tree exactly.

The desired result is a single maintenance line containing two independently
buildable and releasable products plus a deliberately designed shared layer.
It is not a configurable universal firmware image.

## Scope boundaries

This work includes structural refactoring and fixes required to preserve the
two products while consolidating them. It does not include opportunistic fixes
to unrelated defects. Record unrelated findings in an issue/notes section with
the affected file, observed behavior, and reproduction evidence, but do not
change the code for them in this effort.

The IMU is an explicit first-round exception. Both boards use an ICM-20948 with
the same wiring, but the implementations have diverged too far:

- PPGv2 keeps `imuSensor.c/.h` and its current behavior.
- ECGv0 keeps `icm20948_accel.c/.h`, `imuFsyncTiming.c/.h`,
  `accelRecorder.c/.h`, `accelTimingEstimator.c/.h`, and its record-format
  files/tests.
- The board descriptions should still describe the common physical ICM-20948
  wiring accurately. Do not force the two C implementations behind one API in
  this round.
- Add a later-work note describing how to converge the IMU drivers after this
  consolidation is stable.

## Evidence snapshot

This plan was prepared on 2026-08-27 from the following remote refs:

| Line | Ref | Tip | Tip date / subject |
| --- | --- | --- | --- |
| PPG import baseline | `origin/main` | `ae58cb6` | millisecond-precision BLE datetime fix; refreshed for Phase 0 |
| ECG | `origin/simp-nathan-ecg-yuyi200mAh` | `070793f` | 2026-08-26, filesystem log newline fix |
| Rewritten ECG baseline | `codex/ecgv0-history-cleanup` | `7aac128` | 11-commit, tree-identical ECG history |
| Rewrite documentation | `codex/ecgv0-history-docs` | `f5a8743` | verification, mapping, and range-diff records only |
| Consolidation line | `codex/ppgv2-ecgv0-consolidation` | `e027883` | Phase 0 complete and audited |
| Common ancestor | both | `68967d2` | 2026-06-26, MCUboot VREGH change |

At Phase 0 import there were 24 main-side commits and 41 original ECG-side
commits after the common ancestor. Phase -1 compacted the ECG range into 11
commits without changing its tree. Phase 0 then created the consolidation line
from `codex/ecgv0-history-cleanup` and imported `origin/main` at `ae58cb6` with
explicit ancestry. Later work continues from
`codex/ppgv2-ecgv0-consolidation`, not from either source branch.

The two histories are not suitable for a blind merge. Work after the common
ancestor is interleaved across storage, BLE, power, identity, logging, USB,
sensor pipelines, and build configuration. One storage patch is already
patch-equivalent on the two branches (`295f80a` on main and `43d07fd` on ECG),
which is another reason to reconcile by subsystem rather than by commit count.

The project is documented for nRF Connect SDK v2.9.3 and uses sysbuild with
MCUboot plus an nRF5340 network-core HCI IPC image. Preserve that SDK version.
Do not silently move to another NCS release during consolidation.

The owner has confirmed that `PPGv2` and `ECGv0` are internal repository and
application names only. Existing BLE advertising names, model strings, UUIDs,
and other externally visible identities must not change as a consequence of
the rename.

## Target repository layout

Use this as the intended shape. Minor naming adjustments are acceptable if the
owner confirms them before code is moved.

```text
MotionSenseHRV4Flash/
├── PPGv2/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── prj.conf
│   ├── sysbuild.conf
│   ├── sysbuild/
│   ├── src/
│   │   ├── main.c
│   │   ├── ppgSensor.c
│   │   ├── imuSensor.c
│   │   └── ...PPGv2-specific modules...
│   ├── tests/
│   └── README.md
├── ECGv0/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── prj.conf
│   ├── sysbuild.conf
│   ├── sysbuild/
│   ├── src/
│   │   ├── main.c
│   │   ├── drivers/ecg/max30001.c
│   │   ├── icm20948_accel.c
│   │   ├── imuFsyncTiming.c
│   │   └── ...ECGv0-specific modules...
│   ├── tests/
│   └── README.md
├── boards/
│   └── senselab/
│       ├── ppgv2/
│       └── ecgv0/
├── shared/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── include/msense/
│   ├── src/
│   ├── drivers/
│   │   ├── nand/
│   │   ├── nor/
│   │   └── dhara/
│   ├── dts/bindings/
│   └── zephyr/module.yml
├── tools/
├── docs/
└── README.md
```

Do not retain two private copies of a file merely because both compile. A file
belongs in `shared` only after its product-specific inputs have been made
explicit through a C API, devicetree, or Kconfig. Conversely, do not create a
shared file full of `#if PPGV2` / `#if ECGV0`; that is duplicated product logic
hidden in one file.

The application CMake files should enumerate sources explicitly. The current
`FILE(GLOB ... src/*.c)` arrangement makes unused or stale product modules part
of a build merely because they exist in the directory. Explicit lists are
important once both applications and a shared module coexist.

## Architectural ownership rules

### Board definitions own physical facts

The board layer owns:

- nRF5340 SoC/core selection and board qualifiers;
- pinctrl groups and peripheral instances;
- GPIOs, chip selects, interrupt pins, power-enable pins, and buttons;
- PPG versus MAX30001 population;
- ICM-20948 physical wiring;
- NOR and NAND population and geometry;
- battery gauge electrical parameters that differ with the battery/charger;
- regulators, HFXO/LFXO wiring, and board clock properties;
- aliases and chosen nodes used as stable application-facing hardware names.

No board pin or NAND chip-select list should remain hard-coded in C. In
particular, `spi_nand.c` currently has a `cs_pins[]` array that differs between
the two branches (four entries on main and two on ECG), while the same facts
also live in the overlay. Removing that duplication is part of this
consolidation, not optional cleanup.

### Shared code owns product-independent mechanisms

Strong first-round shared candidates are:

- custom NAND transport and disk driver;
- custom NOR code;
- Dhara sources (whether or not enabled yet);
- bad-page handling;
- common devicetree bindings for the storage devices;
- generic disk mount/unmount, read-only coordination, and capacity queries;
- generic buffered/chunked recording mechanics after stream policy is removed;
- filesystem log backend mechanics;
- device-ID and Git-build-metadata generation where the product name/model is
  passed in rather than embedded;
- battery monitor logic if both applications can use the same algorithm with
  board-specific gauge properties;
- truly identical BLE utility code, but not the product GATT contract or
  product-specific control policy.

### Application code owns product behavior

Keep these application-specific unless a later review demonstrates a clean
interface:

- each `main.c` and initialization sequence;
- MAX86141 PPG acquisition and format;
- MAX30001 acquisition and ECG format;
- both first-round IMU implementations;
- product-specific BLE services/characteristics and control semantics;
- product name, model, advertising identity, and release version;
- sensor-specific file naming, record formats, chunk sizing, and flush policy;
- ship-mode behavior if the required peripherals or user-visible behavior
  differs;
- application log-level policy and feature selection.

## Phase -1: rewrite and verify ECG history — complete

This phase was completed before directory renames, custom-board work,
shared-module work, or firmware changes. Its accepted output is
`codex/ecgv0-history-cleanup` at `7aac128a5b2c2e028aebd6ea5b076680cc1cbc4b`.
The rewritten tree is exactly the same as original ECG tip `070793f`:
`fd0b7d850237693ea42918314d61fe3598a09a1b`. The original 41-commit ECG range
is represented by 11 compact commits.

The safety archive remains
`refs/archive/ecgv0-original-2026-08-27` -> `070793f`. The original
`simp-nathan-ecg-yuyi200mAh` branch remains unchanged. Verification artifacts
are committed separately on `codex/ecgv0-history-docs` at `f5a8743`; that
documentation branch is not the firmware baseline.

The remainder of this section is retained as the audit record for how Phase -1
was performed and verified. Do not repeat the rewrite unless the owner
explicitly rejects or replaces the accepted baseline.

### Safety and branch policy

1. Fetch and record the current ECG tip and merge base with `origin/main`.
2. Create an immutable local safety ref (and, if repository policy permits, a
   remote archive ref) pointing to the original ECG tip. Record its full SHA in
   the rewrite notes.
3. Create a new branch such as `codex/ecgv0-history-cleanup`. Do not force-push,
   reset, rename, or delete `simp-nathan-ecg-yuyi200mAh`.
4. Rewrite only the ECG-side range after the recorded merge base. Do not mix in
   current main changes, directory renames, custom-board definitions, spelling
   fixes, formatting, or generated files.
5. Flatten incidental development merges where useful. Preserve their intent
   in the old-to-new mapping rather than retaining a merge topology that no
   longer helps reviewers.
6. Stop and investigate if an original change cannot be assigned confidently
   to one clean commit. Do not “clean up” its code while cleaning history.

An interactive rebase with merge awareness may be used, but a controlled
reconstruction of the ECG-only range is also acceptable. The technique matters
less than exact tree identity, a complete mapping, and readable intermediate
commits. Reordering must respect dependencies so each clean commit is coherent;
prefer every intermediate commit to compile, but final tree identity is the
non-negotiable gate.

### Implemented compact ECG commit series

The implemented series uses the following grouping. The mapping on
`codex/ecgv0-history-docs` is authoritative for original-to-rewritten SHAs:

1. `refactor(ecgv0): remove unused legacy processing and simplify sensor orchestration`
   - Combine `b42b2b0`, `09cb3f8`, `d277965`, and `4465f61`.
   - Explain removal of TensorFlow Lite Micro, magnetometer/orientation paths,
     PPG/IMU decoupling, warning cleanup, and test-file expansion adjustments.
2. `feat(ecgv0): add MAX30001 ECG hardware support`
   - Combine the MAX30001 scratch/DT/probe/filter/acquisition work from
     `13a5980`, `2f934a5`, `d068c5c`, `0805ef8`, `a957c4e`, and `d160fb7`.
   - The message must state that MAX30001 is the ECG AFE. MAX86141 is the PPG
     product's optical sensor and is not part of this ECG hardware commit.
3. `feat(ecgv0): integrate ECG recording, USB diagnostics, and BLE transport`
   - Combine the coherent recording/transport parts of `2df5c01`, `53f4f7e`,
     `eaccc6b`, `81680ee`, `a3af46e`, and `06fb120`.
   - Document record flow, mass-storage restoration, BLE counter/stream
     behavior, and the associated format documentation.
4. `build(ecgv0): standardize NCS 2.9.3 clock, sysbuild, and developer tooling`
   - Combine `0fbcf53`, `0203ca6`, `012940a`, and `ce2b543`.
   - Keep Bluetooth preference changes from `721faa4` with the BLE commit if
     that produces a clearer dependency; otherwise document them here.
5. `feat(ecgv0): add the ECG-specific ICM-20948 implementation`
   - Preserve `e7b543e` as the foundation of the intentionally separate ECG
     IMU driver.
6. `feat(ecgv0): anchor accelerometer and ECG records to RTC timing`
   - Combine `43455e9`, `d24fe1d`, and `a8f6c23` in dependency order.
   - Document the RTC0 vector fix, FSYNC timing markers, FIFO watermark, and
     record timestamps.
7. `feat(ecgv0): add stable device identity and Git build provenance`
   - Reword/retain `8e63285` with the existing BLE identity unchanged.
8. `feat(ecgv0): configure the 200 mAh battery and restore battery monitoring`
   - Combine `a1f0c91` and the desired `f56d440` behavior.
   - Expand the message to explain gauge configuration, power-flow reporting,
     and why the 200 mAh/taper parameters are correct. Remove the uncertainty
     from the message, not from the code.
9. `feat(ecgv0): add button-driven ship mode while preserving USB storage access`
   - Combine `702fd46`, `5542eb8`, and `912f812`.
   - State that the physical user button and ship mode are ECGv0-only.
10. `fix(storage): preserve NAND linkage and validate ECC and 8-Gbit geometry`
    - Combine `43d07fd`, `dc7f29b`, and `5718d2e`.
    - Explain LTO linkage, accepted/rejected ECC states, two-device geometry,
      and exact capacity assumptions.
11. `feat(logging): persist reset/filesystem logs without disrupting collection`
    - Combine `9de2f2d`, `90c6e28`, `24ea8c7`, `6b30943`, `5acb491`, and
      `070793f`.
    - Explain the NAND log backend, reset cause, USB CDC suppression during
      collection, and newline/message cleanup.

If a listed original commit spans two proposed subjects, split its patch into
the appropriate clean commits and list the original SHA under both mapping
entries. If two groups cannot be reordered without changing their resulting
patch, keep their dependency order and explain it. Do not manufacture a clean
story that contradicts the code.

### Commit-message standard

Every rewritten commit message should contain:

- a concise conventional subject naming ECGv0 or the shared subsystem;
- a body explaining the problem/context, the behavior introduced, important
  hardware assumptions, and interactions with storage/BLE/USB/timing;
- validation that existed at the time, when discoverable from history or docs;
- an `Original-Commits:` block listing every old SHA represented by the commit;
- any known follow-up boundary, especially the deferred IMU merge.

Do not claim tests that were not run or infer intent not supported by the
patches, nearby documentation, or owner confirmation.

### Rewrite verification gate

The rewritten branch passed the following gate before being accepted for later
phases:

1. Compare the full tree object IDs. The original ECG tip and rewritten ECG tip
   must resolve to the same tree (`<old-tip>^{tree}` equals
   `<rewritten-tip>^{tree}`).
2. Run a no-difference tree comparison between the two tips, including file
   modes, binary files, deletions, and repository-root files. The diff must be
   empty.
3. Produce an old-to-new mapping covering every ECG-only commit, including the
   historical merge and any commit split across more than one replacement.
4. Confirm the rewritten range contains no main-only patches, new formatting,
   generated files, or directory renames.
5. Review the ordered patch series with range-diff or an equivalent comparison.
   Explain expected structural differences caused by squash/split/reorder.
6. If builds are authorized, build the original and rewritten tips with
   separate output directories and compare the meaningful artifacts/configs.
   A build is supplementary; it does not replace tree equality.
7. Present the new branch, mapping, tree IDs, and empty-diff evidence to the
   owner for approval. This review passed; Phase 0 may proceed from the accepted
   cleanup branch.

Suggested rewrite artifacts, stored under `docs/history/`, are:

- `ecgv0-history-rewrite-map.md`;
- `ecgv0-history-range-diff.txt` or a reproducible command/transcript;
- `ecgv0-history-verification.md` containing old/new SHAs and tree IDs.

Phase 0 used `codex/ecgv0-history-cleanup` as its starting branch, as required.
The original ECG branch remains available as the archival source of truth,
while `codex/ecgv0-history-docs` remains the durable rewrite audit record.

## Phase 0: preserve evidence and establish baselines — complete

Phase 0 completed on `codex/ppgv2-ecgv0-consolidation` at
`e027883d6005f8a7e582aedc020c53acf7cbd044`. Its detailed provenance and import
matrix are in `docs/consolidation-phase0.md`. Do not repeat or squash this
history; continue Phase 1 from this consolidation branch.

### Implemented checkpoints

1. `8b27bd3` — renamed the accepted cleaned ECG application from
   `MSenseDevice` to `ECGv0` without content changes.
2. `0616e62` — renamed refreshed main application `ae58cb6:MSenseDevice` to
   `PPGv2` on the temporary main import branch without content changes.
3. `e3e91a1` — merged the PPGv2 import while preserving both source ancestries.
4. `0dc6b5c` — merged Phase -1 documentation commit `f5a8743` as a
   documentation-only change.
5. `e027883` — recorded the source refs, root-file decisions, import matrix, and
   Phase 0 verification results.

The consolidation branch first-parent lineage begins with the cleaned ECG
history. The PPGv2 merge has parents `8b27bd3` and `0616e62`; the documentation
merge has parents `e3e91a1` and `f5a8743`.

### Independent lightweight audit

The read-only Phase 0 audit passed:

- `ECGv0` has tree ID `c49f9039f5bc5e6e290b242a8e64dbb122ac0e9a`,
  exactly matching `7aac128:MSenseDevice`; its full binary/file-mode diff is
  empty.
- `PPGv2` has tree ID `64853c883caee95fc8ec18234ff9b6d899265a03`,
  exactly matching `ae58cb6:MSenseDevice`; its full binary/file-mode diff is
  empty.
- `7aac128`, `ae58cb6`, and `f5a8743` are all ancestors of the consolidation
  tip.
- Both application checkpoint commits consist entirely of `R100` renames.
- No tracked file remains under the old root `MSenseDevice` path.
- `BLEService.c/.h`, `device_identity.c/.h`, and `prj.conf` for each product
  have blob IDs identical to their respective source application.
- The Phase -1 documentation merge adds only its three audit documents, and
  the final Phase 0 commit adds only `docs/consolidation-phase0.md`.
- The selected root `.gitignore` exactly matches `ae58cb6:.gitignore` and is
  documented as product-neutral.
- The tracked index and working tree were clean at audit time. No build or
  functional firmware change was performed.

### Deferred Phase 0 follow-ups

- Main's root `PPG_PACKED_16_BYTE_FORMAT.md` was deliberately not selected
  during the import. It remains available through main ancestry and must be
  placed with PPGv2 product documentation in a later documentation phase.
- The owner subsequently authorized deletion of the ignored legacy
  `MSenseDevice` directory. It was verified to contain no tracked files and was
  removed before Phase 1. Its build/editor/test outputs were disposable and
  can be regenerated; there is no longer a third application-like path.
- The handoff document was committed by the owner. Temporary analysis material
  remains outside the tracked application trees.

## Phase 1: create genuine custom nRF5340 boards

This is the first functional change. Do it before sharing drivers or changing
application behavior so hardware-description regressions can be isolated.

### Required board structure

Create two Hardware Model v2 out-of-tree board definitions based on the NCS
v2.9.3 nRF5340 DK board structure, not application overlays named after the DK.
The intended targets are:

- `ppgv2/nrf5340/cpuapp`
- `ppgv2/nrf5340/cpunet`
- `ecgv0/nrf5340/cpuapp`
- `ecgv0/nrf5340/cpunet`

Include the v2.9.3-equivalent board metadata and configuration files such as
`board.yml`, the board DTS/DTSI and pinctrl files, minimal board defconfigs,
`Kconfig.<board>`, `Kconfig.defconfig`, and `board.cmake`. The application core
target is the user firmware target; the network core target must also remain
valid because sysbuild creates the HCI IPC image.

Expose the repository board root in a way inherited by sysbuild and all child
images. A plain `BOARD_ROOT` assignment after `find_package(Zephyr ...)` is too
late, and application-only setup may not propagate through sysbuild. Prefer a
repository/module-level board-root declaration or the appropriate sysbuild
setup for NCS v2.9.3, then verify the generated board used by the application,
MCUboot, and HCI IPC images.

Nordic's authoritative guidance for this version family is to keep custom
boards out of the SDK tree, mirror the normal `boards/<vendor>/<board>`
structure, define `BOARD_ROOT` before Zephyr discovery, and copy/adapt the
nRF5340 DK definition for an nRF53 multi-core custom board. References:

- <https://nrfconnectdocs.nordicsemi.com/ncs/latest/zephyr/develop/application/index.html#custom-board-devicetree-and-soc-definitions>
- <https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-3-adding-custom-board-support/topic/exercise-2-5/>
- <https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-3-adding-custom-board-support/topic/board-files-for-multi-core-hardware-tf-m/>

### Translate the existing overlays deliberately

Build a hardware inventory table from both schematics and the two current
overlays. Do not infer production hardware solely from DK labels. At minimum,
verify and encode:

| Hardware fact | PPGv2 at snapshot | ECGv0 at snapshot |
| --- | --- | --- |
| Primary sensor on SPI3 | MAX86141 PPG, existing `ppgSensor` path | MAX30001 ECG AFE with INTB/INTB2 |
| ICM-20948 bus/pins | SPI2 with the same INT and FSYNC wiring as ECGv0; older driver does not model a DT child | SPI2 with identical physical INT/FSYNC wiring; newer driver uses a DT child |
| NAND population | four chip selects | two chip selects |
| NAND nominal capacity | four 8-Gbit devices | two 8-Gbit devices |
| Battery design capacity | 300 mAh in main overlay | 200 mAh in ECG overlay |
| Charger comments/settings | BQ25060 / 30 mA taper context | BQ21040 / 9 mA taper context |
| User button | not physically populated; no ship mode | P0.01 active-low; used for ECGv0 ship mode |
| Product radio TX config | current main HCI config requests +3 dBm | current ECG HCI config requests 0 dBm |
| 32.768 kHz source | physical crystal; retain the current PPG capacitor settings | externally driven LFXO input in bypass mode |
| HFXO | same crystal and current capacitor settings as ECGv0 | same crystal and current capacitor settings as PPGv2 |
| Regulators | `VREGMAIN` and `VREGRADIO` in LDO mode; `VREGH` disabled | `VREGMAIN` and `VREGRADIO` in LDO mode; `VREGH` disabled |

The owner confirmed this hardware table on 2026-08-27, including the sensor
part numbers, identical physical ICM-20948 INT/FSYNC wiring, NAND populations,
battery/charger distinctions, absence of a PPGv2 button, clock sources, and
regulator modes. The platform revision names are `PPGv2` and `ECGv0`. The
implementing agent will not have direct schematic or BOM access; treat the
owner-confirmed facts plus the current per-product overlays as the authoritative
Phase 1 inputs. Do not invent or “correct” pin routing or capacitor values that
cannot be supported by those inputs. The `f56d440` ECG battery-monitor behavior
is desired; Phase -1 already replaced its uncertain message with documented
intent. Preserve and hardware-validate that behavior during consolidation.

Describe the ICM-20948 child and its INT/FSYNC GPIOs on both custom boards even
though the retained PPGv2 driver initially continues to access the SPI
controller through its older implementation. The difference is a driver-age
artifact, not a platform wiring difference. Ensure that adding the PPGv2 child
does not enable a second competing driver instance.

Move shared physical definitions into common DTSI fragments only when they are
truly identical (the common ICM-20948 wiring is confirmed). Each
board's top-level DTS should remain readable and show which fragments it
includes. Avoid an inheritance scheme where understanding ECGv0 requires
mentally undoing PPGv2 settings.

Use real sensor child nodes and DT specs. Give applications stable aliases or
chosen entries rather than reaching into DK-oriented node labels. Validate
every `DEVICE_DT_GET`, `SPI_DT_SPEC_GET`, and `GPIO_DT_SPEC_GET` consumer
against the new names.

The clock configurations intentionally differ. ECGv0 uses an externally driven
32.768 kHz input through the nRF5340 LFXO bypass path and retains the documented
NCS v2.9.3 bypass patch. PPGv2 uses a physical 32.768 kHz crystal and must retain
its current capacitor settings without enabling bypass. Both boards use the
same HFXO crystal and current HFXO capacitor settings. Encode these physical
facts in the board definitions and verify the intended configuration reaches
both the application and network-core images. Do not quietly delete the ECGv0
SDK patch or apply its LFXO bypass behavior to PPGv2.

Once custom boards are active:

- remove the DK-named application overlays;
- remove the duplicate repository-root DK overlay if it is truly unused;
- remove regulator duplication from `sysbuild/mcuboot.overlay` only after the
  generated MCUboot DTS proves the custom board already supplies it;
- stop passing `-DDTC_OVERLAY_FILE=nrf5340dk_nrf5340_cpuapp.overlay` in build
  wrappers;
- update docs, test metadata, and VS Code/build instructions to the custom
  target names.

### Phase 1 gate

Before sharing any implementation files, compare the old and new generated
devicetrees for each product. Differences should be exactly the intended
removal of DK hardware and the introduction of correctly named custom-board
nodes. Review at least:

- pinctrl selections and GPIO polarity;
- enabled peripheral instances and shared-instance conflicts;
- application and network-core clock sources;
- regulator modes in application and MCUboot images;
- USB CDC and mass-storage chosen nodes;
- NOR/NAND chip selects, counts, sizes, page/erase geometry, and aliases;
- sensor SPI frequency and interrupt/FSYNC pins;
- flash partitions and MCUboot secondary-image assumptions.

Full NCS v2.9.3 builds are authorized and required for both custom-board targets
before Phase 1 passes. Coordinate representative devices and explicit flashing
approval for the PPGv2 and ECGv0 hardware smoke tests; complete those smoke
tests before declaring the board migration accepted. Keep one checkpoint commit
per board so a board port can be reverted independently.

### Phase 1 completion record

Phase 1 is accepted as of 2026-08-28. Both custom boards, including their
application, MCUboot, and HCI IPC images, built successfully at firmware commit
`e239e57` against the managed, pre-patched NCS v2.9.3 workspace at
`C:\ncs\SenSEv2.9.3`. The prerequisite SDK patches were used as supplied and
were not modified by the board migration. Generated configuration confirmed
PPGv2 controller output at +3 dBm and ECGv0 at 0 dBm after the focused TX-power
correction.

The owner reports that representative ECGv0 and PPGv2 hardware smoke tests
both passed. This closes the Phase 1 build and physical-acceptance gate and
authorizes the consolidation to proceed to Phase 2. The exact smoke-test
procedure was not supplied in this handoff, so do not infer that destructive
storage endurance, production DFU compatibility, or exhaustive recording-file
compatibility was covered by these smoke tests.

The equal 264 KiB MCUboot slot layout introduced during the custom-board port
remains subject to the later Partition Manager/production-DFU review. The
known NAND geometry arithmetic and compiler warnings remain tracked technical
debt. The unregistered `senselab` devicetree vendor-prefix warning can be
resolved when Phase 2 establishes the shared devicetree/module root; none of
these deferred items reopens the Phase 1 acceptance decision.

## Phase 2: establish the shared module without changing behavior

1. Create `shared` as a proper Zephyr module with its own `zephyr/module.yml`,
   `CMakeLists.txt`, and `Kconfig` entry points.
2. Move the identical custom devicetree bindings into the shared DTS root.
3. Move files that are byte-identical at the two recorded tips first: the
   common NOR/Dhara/bad-page sources and other proven-identical support code.
4. Have both applications add the shared module before `find_package(Zephyr
   ...)` and select the same shared source through Kconfig/CMake.
5. Replace CMake source globs in both applications with explicit source lists.
6. Do not alter public behavior in this phase. This is a path/build-system
   move, and should be reviewable as such.

The MAX30001 driver and its binding are ECGv0-specific for now. The ICM-20948
binding may live in a common DTS-binding location because it describes the
hardware contract, but the two C implementations remain in their applications.

### Phase 2 gate

- Each application resolves exactly one copy of every linked symbol.
- Changing a shared file causes both applications to rebuild it.
- No application compiles the other product's sensor modules.
- The generated Kconfig contains the shared module once.
- Output identities, record formats, and enabled features are unchanged.

### Phase 2 completion record

Phase 2 is accepted at commit `05b796f`. The `msense-shared` Zephyr module now
owns the common Dhara, NOR, bad-page support, and generic storage bindings;
both applications use explicit source lists and build the same shared sources.
The product-local NAND transports remain separate for Phase 3, PPGv2 alone
compiles `ppgSensor.c`, and ECGv0 alone compiles the MAX30001 implementation.
Both NCS v2.9.3 builds and both `msense_git_metadata` tests passed. The
lightweight post-implementation audit found no Phase 2 blocker.

ECGv0 temporarily retained `src/ble_ppg_compat.c` after Phase 2 because its BLE
service still referenced legacy PPG configuration globals. Subsequent owner
testing confirmed that this surface was vestigial. Commit `03949b5` removes the
shim and the inactive ECG PPG acquisition, power, configuration, and BLE paths
while retaining the ECG manual test-file commands and storage-ownership
behavior. Phase 4 must preserve that removal rather than recreating a generic
PPG compatibility layer in ECGv0.

## Phase 3: reconcile NAND, NOR, disk, and filesystem work

Treat storage as a mini-project with a written reconciliation ledger. Do not
pick one branch's directory as a winner.

### Storage baseline and required ports

Use main's storage implementation as the initial mechanical baseline because
its later work includes the larger structural repair in `9d76b98` and the
erase-block unit fix in `2b4077f`. Preserve and verify, rather than accidentally
dropping, the following main-side behavior:

- serialized/re-entrant disk access around the shared NOR/NAND bus;
- propagation of read/write errors instead of unconditional success;
- file-table NOR error diagnostics and in-session CRC checking;
- corrected `DISK_IOCTL_GET_ERASE_BLOCK_SZ` units (sectors, not bytes);
- corrected sector-offset and duplicate-write handling;
- storage status counters/diagnostics introduced by the main repair;
- logging and filesystem changes coupled to the storage repair.

Then port the intent of the ECG-only storage changes, not necessarily their
exact old diff:

- `dc7f29b`: diagnose and reject unexpected NAND ECC status;
- `5718d2e`: correct 8-Gbit device geometry;
- `070793f`, `6b30943`, and relevant parts of `5acb491`: remove embedded or
  duplicate newlines and clean filesystem log messages;
- `9de2f2d` and `90c6e28`: NAND log-backend enablement and reset-cause logging,
  separated into shared mechanism versus application policy;
- `24ea8c7`: USB CDC logging behavior during collection, only after comparing
  it with main's `ac23549` USB-error logging and each product's required host
  behavior.

The LTO helper fix is already patch-equivalent between main `295f80a` and ECG
`43d07fd`; include it once and mark both commits reconciled.

### Make the shared storage driver platform-driven

Before declaring the storage driver shared:

1. Remove `cs_pins[]` and retrieve all NAND chip selects from devicetree.
2. Derive or validate flash count, individual capacity, total logical capacity,
   pages per erase block, page size, dies per package, and address mapping from
   a coherent device description. Add build-time assertions for inconsistent
   combinations.
3. Use exact integer geometry. Do not retain approximate values such as
   `4294901760` or `1073725440` simply because they were in an overlay.
4. Audit all arrays currently sized for four devices and all constants such as
   64 pages/block or two dies/device. Either make them device-data-driven or
   assert that the supported MT29 part has those fixed properties.
5. Ensure `DISK_IOCTL_GET_SECTOR_COUNT`, sector size, erase-block size, FAT
   volume size, file-table reservation, and bad-block scanning all use the same
   geometry model.
6. Keep product-specific NAND population only in each board DTS. The C driver
   must be identical for two-chip and four-chip boards.
7. Preserve a single lock owner for the shared SPI/storage path and document
   which APIs may re-enter it.
8. Define explicit return semantics for USB writes rejected while the firmware
   owns the disk. Do not rely on a fake-success convention without documenting
   and testing why the USB stack needs it.

### Split filesystem mechanism from stream policy

The current `zephyrfilesystem.c` is not yet a clean shared candidate. At the
tips it embeds PPG versus ECG enum values, file names, record descriptions,
write sizes, chunk rules, collection IDs, and work items. Refactor it along
these boundaries:

- shared volume layer: mount, format policy, free-space query, read-only state,
  sync/close, disk ownership, and common error reporting;
- shared buffered writer: bounded buffers, queued writes, flush, close, chunk
  rotation, and error propagation;
- shared filesystem-log backend: generic callback and lifecycle;
- application stream descriptors: stream ID, prefix, record-description text,
  write/chunk size, maximum writes, collection ID rules, and rollover behavior;
- application orchestration: when collection begins/ends and which streams are
  flushed.

Prefer a typed descriptor/API over a shared enum that assumes every product
has PPG, accelerometer, and ECG streams. Keep file-format documentation and
format tests next to the application that owns the format.

### Storage validation gate

Perform these tests separately on both board populations:

- geometry assertions and sector-count/erase-block ioctl checks;
- NOR file-table erase/read/write and power-cycle persistence;
- NAND read/write across page, erase-block, die, and chip boundaries;
- ECC clean/corrected/uncorrectable status handling, including unexpected
  status values;
- concurrent filesystem/log/USB access and lock behavior;
- FAT format, mount, preallocation, file rollover, close, remount, and host
  readback;
- full/near-full disk behavior and recovery after interrupted writes;
- correct capacity reported over USB for two-chip and four-chip hardware;
- logs with no unintended blank lines or embedded terminal newlines;
- existing PPGv2 and ECGv0 recordings remain compatible in both binary record
  format and filename convention. This is a required compatibility contract,
  not an optional migration decision.

Formatting or destructive media tests require explicit owner approval and
known disposable/backed-up devices.

### Phase 3 completion record

Phase 3 is owner-accepted as of 2026-09-01. The shared NAND and filesystem-log
implementation was completed through `7442059`, with the final build evidence
recorded by `f555c0f`. The owner subsequently tested the firmware on hardware
and approved continuing to Phase 4. Detailed case-by-case hardware evidence was
not supplied to this handoff, so do not infer exhaustive fault-injection,
endurance, power-loss, full-disk, or factory-bad-block coverage.

Hardware testing and follow-up analysis produced additional storage-lifecycle
work. The current local history compacts that work into:

- `1084876` — exclusive firmware/host filesystem ownership, safe legacy MSC
  no-medium/read-only publication, storage-log discard handling, boot-session
  `uuid.txt` creation, and the storage audit documents;
- `03949b5` — removal of inactive ECG PPG compatibility paths plus larger ECG
  and PPG storage-transition stacks.

The owner rewrote this history by squashing commits but confirmed that the code
did not change. Continue Phase 4 from local branch
`codex/ppgv2-ecgv0-consolidation` at `03949b5`. The unresolved production risks
in `STORAGE_AUDIT.md` and `docs/ecg-filesystem-corruption-review.md` remain
tracked findings; Phase 3 acceptance does not silently declare them fixed.

## Phase 4: reconcile other shared subsystems

Work one subsystem at a time, with a commit and validation gate for each.

### Build metadata and device identity

Main commits `9a0a012` and `ab3b816` establish Git provenance and stable nRF5340
identity; ECG commit `8e63285` adapts that work for ECG. Share the mechanism,
but pass the product identity/model/name from each application. Verify that the
rename to PPGv2/ECGv0 does not accidentally change an externally consumed BLE
name, UUID, filesystem metadata string, or device ID unless the owner approves
that product change.

During late Phase 3 testing, generated Git metadata temporarily became
`unknown` because of a build-wrapper defect. The owner reports that the wrapper
is fixed; do not work around it in firmware. Make the first Phase 4 build gate
confirm that both products embed the actual commit and tree state. Commit
`1084876` also moves `uuid.txt` creation into the boot filesystem session;
preserve and validate that timing and file-content contract during identity
reconciliation.

### Battery monitoring

Keep capacity/taper/charger facts in each board definition. Reconcile main
`940149d`, ECG `a1f0c91`, and ECG `f56d440` at the function level. The large
`f56d440` rewrite is confirmed as desired ECG behavior, but must still be
reviewed and hardware-tested while being adapted to shared code. A shared
battery-monitor module is appropriate only if the algorithm is common and all
electrical/product differences are parameters.

### BLE and USB

`BLEService.c` differs by roughly 755 changed lines between the tips and is
mixed with product behavior. First produce a table of services,
characteristics, commands, notifications, connection parameters, device names,
and storage/collection interactions for both products. Extract only common
utilities initially. Keep separate product service definitions if their GATT
contracts differ.

Reconcile, rather than overwrite:

- main's BLE power optimizations and current connection/status behavior;
- ECG's stable identity/build metadata;
- each product's preferred connection parameters and controller TX power;
- ECG-only button-driven ship mode and the fix that keeps mass storage
  available in ship mode. Do not add ship mode to PPGv2, whose button is not
  physically populated;
- USB ownership and logging during active collection.

The planned compatibility cleanup is already implemented in `03949b5` after
owner testing established that the surface was vestigial: ECGv0 no longer contains
`ble_ppg_compat.c`, `ppgSensor.c/.h`, or the inactive PPG BLE/configuration
surface. Treat that commit as the accepted baseline and verify its external
BLE contract rather than repeating the removal. Likewise, `1084876` already
implements the shared MSC no-medium/read-only ownership mechanism and the
product lifecycle integration. Phase 4 should reconcile and test the remaining
BLE/USB contracts around that mechanism, not replace it wholesale.

### Sysbuild and child images

Use a common sysbuild pattern, but allow per-product fragments where settings
really differ. ECG added `sysbuild.cmake` to ensure its HCI IPC fragment is
applied; main does not have the same file at its tip. Verify configuration in
the generated HCI image rather than assuming either route wins. Confirm
MCUboot partition sizing, external-flash checks, HCI IPC activation, LF clock,
PHY/data-length features, and TX power for both products.

### Common utilities

Review `common.h`, Git metadata generation, timer helpers, and logging helpers
for extraction only after their APIs are cleaned up. Do not broaden this phase
into a general style rewrite. Preserve timing and concurrency semantics.

### Phase 4 implementation record — awaiting owner acceptance

Phase 4 implementation is complete at functional commit
`ee2173ef5d9877dce284d121a4cdd58e92b4c9f8` on
`codex/ppgv2-ecgv0-consolidation`; it is not owner-accepted yet. The full
reconciliation ledger and build evidence are in `docs/consolidation-phase4.md`.

- `bbbaf74c` shares Git-provenance generation/CTest and stable nRF5340 identity
  derivation while each product explicitly retains its BLE prefix and DIS model.
- `b1e7de0` adds the shared static compatibility gate that protects accepted
  BLE/USB, storage, sensor, IMU, ship-mode, board, and child-image boundaries.
- `ee2173e` shares only the conditional HCI IPC-overlay mechanism; each product
  continues to own its controller fragment, clocks, radio power, and board
  settings. Battery monitors, BLE service definitions, product sensor paths,
  IMU drivers, timer helpers, and logging policy remain product-local by
  evidence-based decision.
- Fresh detached-worktree wrapper builds for both products passed at `ee2173e`,
  with all three application CTests passing. Both generated provenance headers
  contain the exact commit and `MSENSE_GIT_TREE_STATE "clean"`.
- No external BLE name, UUID, DIS model, `uuid.txt` filename/timing/content
  contract, recording filename/layout, Phase 3 storage ownership behavior, or
  board regulator/oscillator setting was changed. No PPG path was reintroduced
  in ECGv0.

The remaining acceptance gate is independent review plus owner hardware smoke
tests on one PPGv2 and one ECGv0 unit: boot MCUboot/application/network images;
verify product BLE identity, connection/PHY, streams, battery and charger
behavior, repeated collection/USB ownership transitions, `uuid.txt` and record
compatibility; then additionally verify ECGv0 button-driven ship mode retains
mass storage and PPGv2 has no ship-mode/button path. The production storage
risks in `STORAGE_AUDIT.md` and `docs/ecg-filesystem-corruption-review.md`
remain deferred. No hardware was flashed, no media was accessed, and nothing
was pushed during Phase 4.

Phase 5 is authorized to proceed speculatively while those Phase 4 smoke tests
run concurrently. The smoke-test firmware is the Phase 4 functional commit
`ee2173ef5d9877dce284d121a4cdd58e92b4c9f8`; do not attribute its results to a
later Phase 5 commit. Before Phase 4 is marked accepted—and before Phase 5 is
accepted or used as a release baseline—re-synchronize this branch with the
Phase 4 test record:

- record the tested unit/board revision, firmware SHA, procedure, and result;
- port any required Phase 4 correction onto the current Phase 5 lineage rather
  than discarding Phase 5 work or editing the historical test result;
- rerun every affected focused/build/hardware check after such a correction;
- confirm that Phase 5 naming and cleanup did not change any external contract
  covered by the Phase 4 tests.

A passing Phase 4 result closes the Phase 4 hardware gate only for `ee2173e`.
Phase 5 still requires its own final build and appropriately scoped regression
testing after cleanup.

## Phase 5: application cleanup and naming

After common infrastructure is stable:

1. Set the CMake project names to `PPGv2` and `ECGv0`.
2. Update documentation, test manifests, wrapper defaults, artifact paths, and
   developer instructions.
3. Remove stale product sources from each application once explicit source
   lists prove they are unused. For example, investigate why ECGv0 still has
   `ppgSensor.c`; do not delete it solely based on its name.
4. Keep each `main.c` thin enough to show product orchestration, but do not make
   a broad architecture rewrite a prerequisite for consolidation.
5. Preserve product-specific record format documents and tests. PPG's packed
   format work on main and ECG's record/timestamp tests on the ECG branch must
   both remain represented.
6. Add a root README explaining the two products, their board targets, their
   build outputs, shared-code policy, and how a future third platform should be
   added.

## Build and test tooling

The shared wrapper at `D:\senselab-tools\vscode-wrapper` has been updated and
no longer forces the DK target or a DK-named overlay. Callers explicitly select:

- application root (`PPGv2` or `ECGv0`);
- custom board target;
- separate build directory;
- optional pristine mode;
- the fixed NCS v2.9.3 and compatible toolchain roots.

It also now supplies the temporary Git trust/configuration needed for accurate
application provenance. The earlier `MSENSE_GIT_COMMIT` /
`MSENSE_GIT_TREE_STATE` issue was a wrapper defect and is resolved; do not add a
firmware workaround. Builds on this machine must run serially because concurrent
wrapper invocations can race while resolving the shared SDK/toolchain
environment.

Never let the two products share a build directory. Recommended defaults are
`PPGv2/build` and `ECGv0/build`. Preserve timestamped logs in
`D:\senselab-tools\logs` and report the log and artifact paths.

The owner has authorized full NCS v2.9.3 builds for this consolidation,
including the Phase 1 custom-board gate. Read
`D:\senselab-tools\vscode-wrapper\README.md` and use the shared wrapper rather
than assuming `west`, CMake, or the Nordic toolchain are on `PATH`. Report each
build's result, log path, and artifact paths. Do not add `-Pristine` unless a
clean build is appropriate or requested.

Add a build matrix to CI or documented local checks:

| Application | Board | Images |
| --- | --- | --- |
| PPGv2 | `ppgv2/nrf5340/cpuapp` | app + MCUboot + HCI IPC network core |
| ECGv0 | `ecgv0/nrf5340/cpuapp` | app + MCUboot + HCI IPC network core |

Run the existing focused tests and keep them assigned to the correct product:

- Git metadata CMake test for both applications/shared helper;
- ECG record-format test;
- ECG accelerometer record-format and recorder tests;
- PPG packed-record tests/documented decoder checks;
- new host/unit tests for shared storage geometry and stream descriptors where
  feasible.

## Hardware acceptance matrix

A successful compile is not completion. Test one representative PPGv2 unit and
one representative ECGv0 unit.

For each product verify:

1. MCUboot, application core, and network core boot normally.
2. The expected BLE name/model/identity is advertised and remains stable.
3. Connection, PHY/data length, notifications, and disconnect/reconnect work.
4. Battery percentage, charging indication, and low-battery behavior are
   plausible for the correct battery/charger.
5. USB CDC and mass storage enumerate in idle and collection states as required;
   ECGv0 additionally passes its button-driven ship-mode checks. PPGv2 has no
   ship-mode acceptance case.
6. The correct primary sensor streams at the expected rate and format.
7. The product's retained IMU implementation behaves exactly as before.
8. Data collection starts/stops repeatedly without leaked work items or stale
   file state.
9. Files survive close, reset, remount, and host copy; binary decoders accept
   the resulting records.
10. Reported disk capacity and writes across NAND chip boundaries match the
    physical population.
11. DFU works with the retained MCUboot/partition layout.
12. Reset-cause and filesystem logging are readable and do not corrupt the data
    path under load.

Capture firmware SHA, board revision, test procedure, and result for every
hardware run.

## Commit and review strategy

Keep commits small enough to answer one question. A good sequence is:

1. completed ECG history rewrite and tree-identity verification on
   `codex/ecgv0-history-cleanup`;
2. completed Phase 0 on `codex/ppgv2-ecgv0-consolidation` at `e027883`,
   including the provenance snapshot, ECGv0 rename, PPGv2 import from
   `origin/main` at `ae58cb6`, and independent lightweight audit;
3. PPGv2 custom board;
4. ECGv0 custom board;
5. shared module scaffold and identical-file moves;
6. shared NAND geometry/DT conversion;
7. main storage repairs preserved in shared baseline;
8. ECG ECC/geometry/log fixes ported;
9. generic filesystem layers extracted;
10. per-product stream policies wired back in;
11. identity/build metadata reconciliation;
12. battery reconciliation;
13. BLE/USB/sysbuild reconciliation;
14. docs, wrappers, CI, and final cleanup.

For every subsystem maintain a ledger with these columns:

| Source commit | Product/branch | Files/functions | Classification | Destination | Validation |
| --- | --- | --- | --- | --- | --- |
| SHA | main or ECG | exact scope | shared / PPG-only / ECG-only / superseded / deferred | final module/path | test or evidence |

No unique post-merge-base commit should be left unclassified, even if the
decision is “superseded by later code” or “unrelated bug deferred.” This is the
primary defense against silently losing simultaneous development.

After the separately reviewed ECG history rewrite, avoid a giant squash of the
consolidation work. The board ports, imports, and storage reconciliation need
independent rollback points. Do not delete the old ECG branch after merge; mark
it retired/read-only only after both products pass acceptance and the owner
agrees.

## Known risks and investigation notes

These are consolidation-related findings, not authorization to fix unrelated
behavior:

- The duplicated NAND chip-select definition in DTS and C is a direct blocker
  to one shared driver.
- Main has materially stronger disk serialization/error handling than the ECG
  tip, while ECG has newer ECC-status and geometry corrections. Choosing either
  directory wholesale loses important work.
- The filesystem layer mixes generic I/O with product-specific record policy;
  sharing the file unchanged would create conditionals and fragile enums.
- `FILE(GLOB ... src/*.c)` can silently compile stale product code.
- The ECG battery rewrite `f56d440` is large and was self-labeled uncertain;
  the owner has since confirmed its behavior is desired, so the risk is in
  preserving and validating it rather than deciding whether to discard it.
- Sysbuild child configuration is applied differently at the two tips; inspect
  generated child-image configs.
- Historical build wrappers forced the DK board and overlay. This is resolved:
  the current wrapper requires the application/custom-board selection and the
  Phase 4 builds used the genuine PPGv2 and ECGv0 targets.
- Any remaining DK-named overlay is legacy cleanup scope; neither current
  product build may depend on it.
- `C:\ncs\SenSEv2.9.3` is the managed, pre-patched NCS workspace for this
  project, paired with toolchain `C:\ncs\toolchains\b620d30767`. The patches are
  maintained as part of that managed workspace rather than as undocumented
  manual edits. Builds must use that documented workspace (or a reproducible
  equivalent carrying the same managed patches), not silently substitute a
  vanilla or newer SDK checkout.

If a genuinely unrelated bug is discovered, add an entry containing:

```text
Observed behavior:
Affected product(s):
File/function:
Evidence or reproduction:
Risk:
Suggested later issue title:
Changed in this consolidation: no
```

## Confirmed owner decisions and remaining inputs

Confirmed on 2026-08-27:

- final application directories are `PPGv2` and `ECGv0`;
- final board IDs are `ppgv2` and `ecgv0`;
- these are internal names only, and BLE names/model strings/UUIDs do not
  change;
- PPGv2 uses MAX86141 and ECGv0 uses MAX30001;
- both products have identical physical ICM-20948 SPI, INT, and FSYNC wiring;
- PPGv2 has four MT29 devices and ECGv0 has two, with the battery/charger
  differences recorded in the hardware table;
- PPGv2 has no physical user button and does not receive ship mode;
- ECGv0 retains button-driven ship mode;
- the ECG battery-monitor behavior from `f56d440` is desired;
- PPGv2 uses a 32.768 kHz crystal with its current capacitor settings, while
  ECGv0 uses an externally driven 32.768 kHz source through LFXO bypass;
- both platforms use the same HFXO crystal and current HFXO capacitor settings;
- both platforms keep `VREGMAIN` and `VREGRADIO` in LDO mode with `VREGH`
  disabled, with no other oscillator/capacitor/DCDC/regulator differences;
- full NCS v2.9.3 builds are authorized, and representative devices are or can
  be made available for hardware testing;
- existing recordings must remain compatible in binary format and filename;
- consolidation and history branches remain local for now and must not be
  pushed without later authorization;
- the legacy ignored `MSenseDevice` directory was authorized for deletion and
  has been removed after confirming it held no tracked files;
- ECG history is rewritten, verified, and accepted on
  `codex/ecgv0-history-cleanup`; Phase 0 used it as the starting branch, and
  further work continues on `codex/ppgv2-ecgv0-consolidation`.

Phases 1, 2, and 3 are owner-accepted. Phase 4 implementation and independent
review passed at functional commit `ee2173e`, with documentation commit
`464658c`; owner smoke testing of `ee2173e` is still in progress. Phase 5 may
proceed speculatively on the current local lineage, but its work must be
re-synchronized with those test results before either phase is accepted. The
Phase 3 follow-up history was squash-rewritten without changing code. ECGv0's
temporary PPG compatibility layer has been removed, and the exclusive
MSC/filesystem ownership work is already present. The production-storage risks
recorded in the audit documents remain open. Destructive storage-media tests
still require explicit confirmation that the selected devices are disposable
or backed up. Do not push the local branches without later owner authorization.

## Definition of done

The consolidation is complete only when all of the following are true:

- the ECG rewrite branch has the exact same final tree as the archived source
  tip, every original ECG-only commit is mapped, and the owner has approved the
  rewritten series;
- `PPGv2` and `ECGv0` are separate root applications with separate `main.c`
  files, configs, artifacts, and release identities.
- Both build against genuine custom nRF5340 board targets; no DK-named overlay
  is required by either application or wrapper.
- All 24 main-side and 41 original ECG-side commits recorded by Phase 0 are
  classified in the reconciliation ledger.
- There is one maintained NAND/NOR/storage-driver implementation parameterized
  by board description, including correct two-chip/four-chip behavior.
- Generic filesystem code is shared and product record policy remains explicit.
- The two IMU implementations remain separate and a later merge plan is noted.
- Both sysbuild outputs include the intended MCUboot and HCI IPC settings.
- Focused tests pass, both full builds pass when authorized, and both physical
  products pass the hardware acceptance matrix.
- Documentation explains how to build, test, release, and add a future third
  product without copying the entire codebase.
- Unrelated bugs found during the work are documented but not folded into this
  change set.
