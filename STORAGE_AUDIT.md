# NAND, NOR, and Filesystem Storage Audit

**Audit date:** 2026-08-29  
**Repository revision:** `f664b03fd121c2fd570c286096ccb8443dc41f8d` (`codex/ppgv2-ecgv0-consolidation`)  
**Applications:** `PPGv2` and `ECGv0`  
**SDK/toolchain:** nRF Connect SDK `C:\ncs\SenSEv2.9.3`; Nordic toolchain `C:\ncs\toolchains\b620d30767`  
**Scope:** Active NAND, NOR, FAT filesystem, application storage, and USB mass-storage paths. No source code was changed.

## Executive summary

The active storage stack builds successfully for both products, and several low-level fundamentals are correct: the devicetree geometry matches the fitted devices, NAND page/die/package address decomposition is correct, command encodings and read/program/erase ordering mostly match the Micron datasheet, the NOR is correctly described and is handled by the upstream Zephyr SPI-NOR driver, and physical disk I/O is serialized.

The design is not yet safe for production data retention. Four issues dominate the risk:

1. The NAND is exposed as a linear, write-once disk without factory or runtime bad-block management. A full erase erases every block, including factory bad-block markers, contrary to the Micron requirements.
2. FatFS is configured to format automatically on mount failure. On this medium, formatting metadata does not erase the NAND data area, so later allocation can program already-programmed pages and silently corrupt data.
3. NAND busy polling is effectively unbounded and converts an SPI read error into a busy status. Program and erase paths then discard several transport and timeout errors and misinterpret the complete status byte.
4. FAT metadata in NOR is updated by erase-then-program of a single 4 KiB copy. Power loss can destroy a sector, and the in-RAM CRC cannot detect pre-boot corruption. The single-FAT volume and automatic reformat compound this failure mode.

There are also application-level data-loss and concurrency defects: legacy double-buffer work items do not establish buffer ownership before queuing; PPG shutdown sleeps for a fixed interval instead of flushing and draining; positive short writes are treated as success; FatFS is non-reentrant while access can originate from more than one context; and both builds keep USB MSC active during firmware filesystem writes.

This audit identifies **4 Critical, 7 High, 7 Medium, 2 Low, and 1 Informational** findings. Severity reflects plausible impact in this implementation, not merely defensive coding preference.

## Sources and method

The audit traced application calls through Zephyr's filesystem API, FatFS, the custom disk driver, the custom Micron NAND driver, and the active upstream Zephyr SPI-NOR driver. Effective `.config`, generated build outputs, SDK source/API contracts, and the following local datasheets were checked:

- `datasheets/m70a_4_8gb_nand_spi_auto.pdf` — Micron M70A 4Gb/8Gb SPI NAND, Rev. J, May 2023.
- `datasheets/MX25U8035F.pdf` — Macronix MX25U8035F 8M-bit SPI NOR, Rev. 1.1, 2018-07-06.

NCS guidance and filesystem semantics were cross-checked using Nordic's authoritative documentation source. Upstream SDK code was read only to establish contracts and active behavior; it was not audited for defects.

Dormant paths were excluded after confirming they are inactive: Dhara is disabled, LittleFS is disabled, and `shared/drivers/nor/spi_nor.c` is not compiled because the effective configuration uses Zephyr's `jedec,spi-nor` driver and does not enable `CONFIG_SPI_NOR_CUSTOM_DRIVER`.

## Active architecture and data flow

```text
PPG/IMU/ECG/log producer or ECG accelerometer recorder
  -> store_data()/recorder work item
  -> sensor_write_to_file()/fs_write(), fs_sync(), fs_close()
  -> Zephyr VFS
  -> FatFS (disk name "SD", 4096-byte logical sector)
  -> shared/drivers/nand/nand_disk.c
       sectors 0..179   -> Zephyr flash API -> MX25U8035F NOR
                           each write: erase 4 KiB, program 4 KiB
       sectors 180..end -> custom SPI NAND -> MT29F8G01AD package/die/page
  <-> legacy USB MSC at the same disk_access layer
```

### Geometry

| Property | PPGv2 | ECGv0 | Assessment |
| --- | ---: | ---: | --- |
| NAND packages | 4 | 2 | Matches chip-select arrays |
| Dies/package | 2 | 2 | Matches MT29F8G01AD stacked-die device |
| Bytes/package | 1 GiB | 1 GiB | Matches 8 Gbit device |
| Page / logical sector | 4096 B | 4096 B | Matches data area of NAND page |
| Pages/erase block | 64 | 64 | Matches 256 KiB block |
| Total logical capacity | 4 GiB | 2 GiB | Correctly derived with a 64-bit intermediate |
| NOR | 1 MiB MX25U8035F | 1 MiB MX25U8035F | JEDEC ID `c2 25 34`, 8 MHz |
| NOR-routed sectors | 180 (720 KiB) | 180 (720 KiB) | Within NOR capacity |

With the effective FatFS format settings, both volumes become FAT16. The PPG volume uses approximately 128 KiB clusters and the ECG volume approximately 64 KiB clusters. Reserved sector, one FAT, and 4096-entry root directory occupy about 50 sectors; therefore the 180-sector NOR window covers all FAT metadata plus roughly 130 initial data sectors. This boundary is intentional-looking and within the 1 MiB NOR, but it means some file data is also stored in NOR.

## Findings

### F-01 — No usable NAND bad-block management

- **Severity:** Critical
- **Confidence:** Confirmed defect
- **Location:** `shared/drivers/nand/spi_nand.c:591-665`, `:886-973`; `shared/drivers/nand/bad_page.c:140-153`; `PPGv2/prj.conf:253`; `ECGv0/prj.conf:256`
- **Description:** The active mapping exposes every physical NAND page as a logical sector. `CONFIG_RAW_NAND_BAD_SECTOR_SAVING=n`; manufacturer bad-block detection is not called; no persistent bad-block table, block retirement, or relocation is active. The full erase loop erases every block on every die and only then erases NOR metadata.
- **Why it matters:** MT29F8G01AD devices may ship with invalid blocks and may develop additional failures. An invalid block can therefore hold live FAT file data. Erasing a factory-marked block can destroy its identifying mark, making later reconstruction impossible. A program/erase failure has no replacement destination.
- **Evidence:** Micron M70A, “Error Management,” PDF page 51, requires checking the first spare-area byte of the first page in every block **before any PROGRAM or ERASE** and building a bad-block map. For MT29F8G01, only 4016 of 4096 blocks per die are guaranteed valid (up to 80 invalid blocks/die). The current erase starts at block zero and has no exclusion list.
- **Recommendation:** Implement block-level translation with a persistent, redundant, checksummed bad-block table. Scan factory markers before any destructive command, reserve replacement blocks, retire blocks on erase/program failure and on severe ECC degradation, and preserve the factory marks. Do not simply enable the dormant routine: it leaks its device lock on `continue`, does not scan/map all package/die combinations correctly, and does not provide a transactional persistent mapping.

### F-02 — Automatic format is unsafe on non-erasing raw NAND

- **Severity:** Critical
- **Confidence:** Confirmed design defect
- **Location:** effective build configs `CONFIG_FS_FATFS_MKFS=y` and `CONFIG_FS_FATFS_MOUNT_MKFS=y`; `PPGv2/src/zephyrfilesystem.c:768-792`; `ECGv0/src/zephyrfilesystem.c:864-888`; SDK `zephyr/subsys/fs/fat_fs.c:447-482`
- **Description:** `fs_mount()` is called without `FS_MOUNT_FLAG_NO_FORMAT`. When FatFS returns `FR_NO_FILESYSTEM`, Zephyr automatically calls `f_mkfs()` and mounts the newly formatted volume. Formatting rewrites the NOR-resident metadata but does not erase NAND data pages.
- **Why it matters:** FatFS will regard old NAND pages as free and later attempt to reuse them. NAND programming cannot restore zero bits to one; programming a formerly used page without block erase is illegal and can corrupt the new file. A transient/torn metadata read can also trigger automatic destruction of otherwise recoverable metadata.
- **Evidence:** The effective `.config` enables mount-time mkfs. Zephyr's adapter formats specifically on `FR_NO_FILESYSTEM`. The active disk write path directly programs NAND sectors and never erases a block on allocation or sector reuse.
- **Recommendation:** Disable mount-time formatting. Treat mount failure as a recoverable fault requiring an explicit, verified erase/reinitialize workflow. That workflow must first invalidate old metadata transactionally, erase valid NAND blocks while preserving bad-block information, verify completion, and only then create the filesystem.

### F-03 — Full-volume erase destroys bad-block evidence and is not interruption-safe

- **Severity:** Critical
- **Confidence:** Confirmed defect
- **Location:** `shared/drivers/nand/spi_nand.c:886-973`; `PPGv2/src/BLEService.c:378-415`; `ECGv0/src/BLEService.c:570-607`
- **Description:** The user-triggered reset erases all NAND blocks package by package, waits 500 ms between packages, and erases NOR metadata last. BLE handlers ignore the returned status and reset the device.
- **Why it matters:** Besides F-01's destruction of factory markers, loss of power during the long erase leaves old FAT metadata pointing into a partly erased NAND array. A failed erase is reported as complete by the BLE path and followed by reboot; subsequent mount/autoformat behavior can create a mixed old/new volume.
- **Evidence:** NAND erase order is all data first, metadata last; there is no durable erase-in-progress marker or completion record. The application does not branch on `spi_nand_multi_chip_erase()` failure.
- **Recommendation:** Replace with an explicit two-phase media-reset state machine stored redundantly in reliable metadata. Invalidate the old filesystem before data erasure, skip known bad blocks, verify each erase, record progress/completion, and propagate failure to the user. Never advertise completion or reboot as success after a failed operation.

### F-04 — NOR metadata update has a single-copy power-loss window

- **Severity:** Critical
- **Confidence:** Confirmed design defect
- **Location:** `shared/drivers/nand/nand_disk.c:44-51`, `:150-200`
- **Description:** Every metadata-sector update erases a 4 KiB NOR sector and programs its replacement in place. There is no journal, copy-on-write slot, generation counter, or second FAT. The CRC exists only in RAM; the first read after every boot treats current contents as correct, and a later CRC mismatch is logged but returned to FatFS as success.
- **Why it matters:** Power loss after erase or during program can destroy any FAT, directory, boot, or initial data sector. The next boot cannot distinguish torn data and can autoformat. A single FAT provides no repair copy.
- **Evidence:** `file_table_access()` records CRC before erase, then performs one erase and one write. `file_table_crc_valid[]` starts false on boot. Effective formatting uses one FAT. The FatFS documentation explicitly does not provide power-fail atomicity.
- **Recommendation:** Add a transactional metadata layer: redundant sectors/regions, generation and commit records, CRC validated from durable metadata, and recovery selection at boot. Return `-EIO` on validation failure. Consider a filesystem/FTL designed for raw NAND and power failure rather than emulating a rewritable FAT disk.

### F-05 — NAND ready polling can hang for an extreme duration and masks SPI errors

- **Severity:** High
- **Confidence:** Confirmed defect
- **Location:** `shared/drivers/nand/spi_nand.c:418-438`, `:485-500`
- **Description:** `get_features()` returns the byte value `253` on transport error. Its low bit is set, so the wait loop treats the error as OIP/busy. The local `ret` is never changed, the loop has no sleep/yield, and its limit is two billion SPI polls; even the timeout check starts at 1.9 billion.
- **Why it matters:** A transient SPI failure can monopolize a workqueue/thread and storage locks for an operationally unbounded time, causing data loss, watchdog reset, or system unresponsiveness.
- **Evidence:** Micron maximum operation times are bounded (for example block erase up to 10 ms and program up to 600 µs in the applicable timing tables), making the implemented bound many orders of magnitude too large.
- **Recommendation:** Return transport status separately from the register byte. Poll with a monotonic deadline selected per operation plus a conservative margin, yield/sleep between polls, and return `-ETIMEDOUT` or the SPI errno. Add fault-injection tests for read-status failures and stuck OIP.

### F-06 — Program/erase/reset discard errors and misuse the complete NAND status byte

- **Severity:** High
- **Confidence:** Confirmed defect
- **Location:** `shared/drivers/nand/spi_nand.c:372-378`, `:790-881`, `:992-1011`, `:1077`
- **Description:** Reset always returns success. Page program overwrites the WREN result, ignores ready-wait and WRDI results, then returns the complete status byte. Block erase ignores WREN, command-transfer, wait, and WRDI results. Initialization calls `flash_reset_and_unlock()` without acting on its return. The complete status contains OIP, WEL, E_FAIL, P_FAIL, and ECC history.
- **Why it matters:** Transport or timeout failures can be reported as success. Conversely, corrected-ECC bits from a previous read persist until another read and can make a successful program/erase return a positive nonzero “failure.” Error codes are neither stable errno values nor correctly scoped device results.
- **Evidence:** Micron status register C0h defines E_FAIL at bit 2, P_FAIL at bit 3, and ECC in bits 4–6. P_FAIL and E_FAIL are cleared by the next corresponding operation/reset; ECC is updated by page reads. Only the relevant failure bit should determine program/erase success after successful transport/polling.
- **Recommendation:** Check and propagate every command result. Verify WEL after WREN if required by the reliability policy. After completion, test only P_FAIL for program and E_FAIL for erase; translate failures to errno and preserve diagnostic status separately. Make initialization fail if reset, unlock, ID, or configuration cannot be verified.

### F-07 — Disk initialization registers a failed NAND device

- **Severity:** High
- **Confidence:** Confirmed defect
- **Location:** `shared/drivers/nand/nand_disk.c:220-230`, `:564-575`
- **Description:** `disk_nand_access_init()` logs `spi_init()` failure but always returns zero. `disk_sdmmc_init()` ignores even that local `status` and registers the disk.
- **Why it matters:** FatFS and USB MSC can access a device whose JEDEC/configuration/unlock sequence failed. Mount failure may then invoke the unsafe automatic-format path.
- **Evidence:** Both functions contain unconditional success/registration behavior; the build also reports `status` as unused.
- **Recommendation:** Fail device initialization and disk registration on any NAND setup failure. Gate filesystem setup and USB exposure on `device_is_ready()`, `disk_access_init()`, verified JEDEC IDs for every package, and successful die configuration.

### F-08 — Legacy async buffers can be reused before the queued write owns them

- **Severity:** High
- **Confidence:** Confirmed defect
- **Location:** `PPGv2/src/zephyrfilesystem.c:490-607`; `ECGv0/src/zephyrfilesystem.c:581-702`
- **Description:** A buffer/work container is marked `in_use=true` only when its handler begins, not atomically before submission. The submit path obtains `k_work_busy_get()` but makes its reuse decision from `in_use`. `store_data()` resets/toggles the producer buffer immediately after queuing. The same work item and backing buffer can therefore be filled or resubmitted while queued or executing.
- **Why it matters:** Under storage latency or burst load, data can be overwritten before `fs_write()`, work submission can be rejected/dropped, and samples can be duplicated or lost. This is plausible during NOR metadata erase/program, NAND delays, or concurrent MSC reads.
- **Evidence:** Queue state and buffer ownership are separate and non-atomic. ECG's dedicated accelerometer recorder demonstrates the safer pattern: a pool of owned immutable blocks and one filesystem queue.
- **Recommendation:** Replace the legacy two-buffer scheme with a bounded pool/queue whose ownership transfers atomically before submission and returns only after completion. Check every `k_work_submit_to_queue()` result, define backpressure/overrun policy, and track dropped blocks explicitly.

### F-09 — PPG stop does not flush or drain filesystem work before closing

- **Severity:** High
- **Confidence:** Confirmed defect
- **Location:** `PPGv2/src/BLEService.c:447-465`; `PPGv2/src/zephyrfilesystem.c:610-657`, `:729-737`; compare `ECGv0/src/main.c:842-848`
- **Description:** PPG collection stop deinitializes the RTC, sleeps 500 ms, then closes files. It does not flush partial sensor buffers or drain the filesystem workqueue. A queued handler may race with close, reopen a file, or use a buffer reset by shutdown. The final partial buffers are always lost. ECG explicitly flushes ECG data and drains the queue before close, although its runtime legacy-buffer reuse risk remains.
- **Why it matters:** Tail loss occurs on every recording that ends with a non-full buffer; slow storage can also produce close/write races and filesystem corruption.
- **Evidence:** Fixed delay is not a completion primitive. FatFS is configured with BSS LFN storage and is not reentrant, so concurrent close/write is outside the safe contract.
- **Recommendation:** Stop producers first, enqueue all partial buffers, drain the single filesystem owner queue, call `fs_sync()` where needed, then close and verify results. Reuse the ECG shutdown ordering in a common storage lifecycle implementation.

### F-10 — Active FatFS configuration is non-reentrant while ownership is not exclusive

- **Severity:** High
- **Confidence:** Confirmed configuration/design defect
- **Location:** effective configs `CONFIG_FS_FATFS_LFN_MODE_BSS=y`, `CONFIG_FS_FATFS_FF_USE_LFN=1`, no `CONFIG_FS_FATFS_REENTRANT`; PPG filesystem workqueue and BLE close path; both `prj.conf` files `CONFIG_USB_ALWAYS_ON=y`
- **Description:** Application filesystem operations are not all serialized at the filesystem abstraction. The disk mutex serializes individual physical calls, but it cannot protect FatFS internal state or make multi-call FAT transactions atomic. Both builds also leave USB MSC active while firmware files are open and changing. USB host writes are rejected, but reads may interleave between metadata/data updates and the host caches the volume.
- **Why it matters:** Concurrent application contexts can corrupt FatFS state. Even read-only host access can observe inconsistent directory/FAT/data snapshots while firmware writes; a host may cache stale filesystem structures.
- **Evidence:** FatFS with `_USE_LFN=1` (static/BSS LFN buffer) is not thread-safe without reentrancy. Zephyr's legacy MSC thread accesses the disk layer directly; the disk mutex protects calls, not a coherent volume snapshot. `CONFIG_USB_ALWAYS_ON=y` makes `#ifndef CONFIG_USB_ALWAYS_ON` disconnect code inactive in both products.
- **Recommendation:** Give one thread/workqueue exclusive ownership of all FatFS calls, including open/close/stat/logging, or configure and validate filesystem reentrancy with one shared lock. Do not expose the mounted, changing volume to MSC; disconnect/unpublish it while recording, then sync/close/unmount before re-exposure.

### F-11 — File creation can reopen an existing PPG file at offset zero

- **Severity:** High
- **Confidence:** Likely defect
- **Location:** `PPGv2/src/zephyrfilesystem.c:331-388`, `:433-485`; SDK `zephyr/subsys/fs/fat_fs.c:100-114`
- **Description:** PPG-generated names are opened with `FS_O_CREATE | FS_O_WRITE` without first rejecting an existing path and without append or truncation. Zephyr maps `FS_O_CREATE` to FatFS `FA_OPEN_ALWAYS`: an existing file is opened, not replaced, and the initial offset is zero.
- **Why it matters:** A name collision after reboot, clock reuse, random collision, or counter reuse overwrites the start of an existing file. At the physical layer that may reprogram already-programmed NAND pages without erase, which is illegal. `FS_O_TRUNC` alone would not solve safe physical reuse.
- **Evidence:** The SDK contract states CREATE creates the file if absent; it does not imply exclusive create. ECG recording paths use `fs_stat()` and refuse duplicates, which is the appropriate current behavior.
- **Recommendation:** Generate globally unique, reboot-stable recording IDs and explicitly fail if a path exists. Do not truncate/reuse a NAND-backed file until a real erase-aware translation layer exists.

### F-12 — File writes mishandle short writes and logging reports false success

- **Severity:** Medium
- **Confidence:** Confirmed defect
- **Location:** `PPGv2/src/zephyrfilesystem.c:400-412`; `ECGv0/src/zephyrfilesystem.c:489-501`; both `src/filesystemlogging_policy.c:10-14`
- **Description:** `current_writes` advances before success is known. A negative `fs_write()` is logged, but a positive result smaller than the requested length is not treated as an error or retried. The logging backend calls the void `store_data()` and always reports the original length as consumed.
- **Why it matters:** Disk-full, I/O, read-only, queue-full, or partial-write conditions can silently lose bytes while file rotation/accounting proceeds as if data were durable. Logs are silently dropped under the same conditions.
- **Evidence:** Zephyr `fs_write()` returns the bytes written or a negative errno; equality with the requested length is required for this fixed-record stream.
- **Recommendation:** Return status through every application storage layer. Advance counters only by confirmed bytes, handle/retry a defined class of short writes or fail the recording, and make the logging backend report actual acceptance/durability semantics.

### F-13 — Main sensor files lack periodic sync and FAT is not power-safe

- **Severity:** Medium
- **Confidence:** Confirmed risk
- **Location:** `PPGv2/src/zephyrfilesystem.c:304-430`; `ECGv0/src/zephyrfilesystem.c:370-522`; `ECGv0/src/accelRecorder.c:243-349`
- **Description:** Main sensor streams write/preallocate files up to about 4 MiB and rely on close for flush; there is no periodic `fs_sync()`. ECG's dedicated accelerometer recorder correctly syncs its header, periodically after blocks, trailer, and close.
- **Why it matters:** Power loss can lose the current file's directory size and cached FAT/directory changes and may leave an inconsistent volume. `fs_close()` does flush the stream when reached, but it does not protect unexpected reset/power loss.
- **Evidence:** Zephyr's `fs_close()` contract flushes the associated stream; FatFS itself is not power-fail safe. The main writer lacks intermediate sync calls, while the accelerometer recorder demonstrates an implemented cadence.
- **Recommendation:** Define a bounded durability interval, call and check `fs_sync()` accordingly, and measure the latency/sample-buffer capacity impact. This reduces—but does not eliminate—the need for transactional metadata/FTL recovery.

### F-14 — Corrected ECC levels requiring refresh are only logged

- **Severity:** Medium
- **Confidence:** Confirmed risk
- **Location:** `shared/drivers/nand/spi_nand.c:747-780`
- **Description:** ECC codes 1, 3, and 5 are counted as corrected and returned as successful. No block refresh, relocation, retirement, or persistent health tracking occurs.
- **Why it matters:** Code 5 indicates 7–8 corrected bits and Micron says the page **must** be refreshed; code 3 (4–6 bits) says refresh may be required. Continued use can become uncorrectable without warning to higher layers.
- **Evidence:** Micron M70A status/ECC tables, PDF pages 48–49: `010b` is uncorrectable, `011b` is 4–6 corrected, and `101b` is 7–8 corrected with mandatory refresh. The decode itself is correct.
- **Recommendation:** Surface degraded reads to an erase-aware mapping layer, copy valid block contents to a replacement block, retire the source, and persist health/retirement data. Escalate code 5 if refresh cannot be completed.

### F-15 — Reset sequence violates stacked-die tRST command restriction

- **Severity:** Medium
- **Confidence:** Confirmed datasheet mismatch
- **Location:** `shared/drivers/nand/spi_nand.c:992-1011`
- **Description:** Initialization sends RESET and immediately starts GET FEATURE polling through `spi_flash_wait_until_ready()`.
- **Why it matters:** For the stacked dual-die device, Micron states that no command may be issued until tRST after RESET; GET FEATURE cannot be used to poll during reset. Behavior may be intermittent across temperature, voltage, or prior NAND state.
- **Evidence:** Micron M70A RESET section, PDF page 16; AC timing tables give tRST maxima up to approximately 615–635 µs depending on conditions.
- **Recommendation:** Check RESET transport status, delay for the worst-case datasheet tRST before any further command, then read/validate features. Also observe power-on tPOR before initialization if the board/startup sequence does not already guarantee it.

### F-16 — Runtime read-only API is a silent no-op

- **Severity:** Medium
- **Confidence:** Confirmed defect
- **Location:** `shared/drivers/nand/nand_disk.c:203-214`, `:260-267`; PPG/ECG collection lifecycle calls; effective configuration lacks `CONFIG_RAW_NAND_ALLOW_RUNTIME_READONLY_FS`
- **Description:** `set_read_only()` changes state only when a disabled Kconfig option is enabled; callers receive no indication that the request did nothing. Disk status also reports OK rather than write-protected. USB writes are separately rejected by exact legacy MSC thread name when `CONFIG_USB_WRITABLE` is off.
- **Why it matters:** Application state and comments imply ownership protection that is not active. Future firmware writes from other contexts remain allowed, and callers cannot detect the failure to transition.
- **Evidence:** `IS_ENABLED()` compiles to false in both effective builds. The active SDK legacy MSC thread is named `usb_mass`, so that narrower host-write rejection does work.
- **Recommendation:** Make ownership an explicit state machine with an error-returning API, enforce it at the filesystem owner/volume level, and report write protection correctly. Avoid relying on thread names as the primary policy mechanism.

### F-17 — Filesystem readiness can be asserted after root-directory failure

- **Severity:** Medium
- **Confidence:** Confirmed defect
- **Location:** `PPGv2/src/zephyrfilesystem.c:795-872`; `ECGv0/src/zephyrfilesystem.c:891-968`
- **Description:** After a successful mount/stat, failure of `fs_opendir()` is logged, but control reaches `file_system_ready = true`; `fs_closedir()` is then called regardless. More broadly, initialization errors do not transition the application into an explicit storage-failed state that blocks USB/application use.
- **Why it matters:** Producers may begin recording against a filesystem that could not open its root, converting initialization failure into later silent drops or reopen attempts.
- **Evidence:** The assignment occurs after the directory enumeration loop and is not conditioned on `rc >= 0`.
- **Recommendation:** Use one fail-fast initialization path with cleanup. Set ready only after every required step succeeds; propagate a durable fault state to recording control and USB exposure.

### F-18 — Disk API lacks defensive range and overflow validation

- **Severity:** Low
- **Confidence:** Improvement
- **Location:** `shared/drivers/nand/nand_disk.c:271-395`
- **Description:** Read/write callbacks do not validate null buffers, zero/oversized counts, `sector + count` overflow, or end-of-device bounds before looping. Active FatFS/MSC callers normally provide valid requests, and `convert_page_to_address()` catches individual out-of-range NAND pages, but wrapped arithmetic could redirect to NOR sectors.
- **Why it matters:** A malformed or corrupted request can access the wrong physical region or dereference invalid memory. This is not currently proven reachable through normal SDK callers.
- **Evidence:** Loop expressions use 32-bit `sector + x`; validation occurs only later for NAND mapping and not before NOR routing.
- **Recommendation:** Validate the complete request with overflow-safe arithmetic before acquiring/using the device and return `-EINVAL`/`-ERANGE` consistently. Add boundary tests at sector 0, 179/180, last sector, and wraparound.

### F-19 — Shutdown/close errors are discarded and unmount is not performed

- **Severity:** Low
- **Confidence:** Improvement
- **Location:** both `src/zephyrfilesystem.c` `shutdown_filesystem()` and `close_all_files()` implementations
- **Description:** Close results are generally ignored, and `shutdown_filesystem()` does not unmount. The destructive-reset path reboots, so unmount is not strictly required after a verified close, but error reporting and a clean ownership transition are absent.
- **Why it matters:** A failed final flush is invisible; USB can be exposed without proof that filesystem state is quiescent.
- **Evidence:** Zephyr documents that `fs_close()` flushes the stream, so its result is the final durability signal and must be checked.
- **Recommendation:** Return/aggregate close errors, call `fs_sync()` before close when durability requires it, unmount before handing the volume to USB, and expose failure to the lifecycle controller.

### F-20 — Storage-related compiler warnings reduce diagnostic reliability

- **Severity:** Informational
- **Confidence:** Confirmed build evidence
- **Location:** both build logs; `shared/drivers/nand/nand_disk.c`, `spi_nand.c`, `bad_page.c`
- **Description:** Both builds warn about duplicate file-table macros, discarded `const`, incorrect format specifiers, unused storage variables, and an internal function declared/used but never defined in `spi_nand.c`. The link succeeds because the affected static path is not retained/used.
- **Why it matters:** Warning noise can conceal new defects; wrong format specifiers are undefined behavior in variadic logging and can make field diagnostics misleading.
- **Evidence:** See build logs listed below, especially approximately lines 1200–1450.
- **Recommendation:** After functional fixes, make active storage code warning-clean and fail CI on newly introduced warnings. Remove or isolate dormant templates so active audit/build output is unambiguous.

### F-21 — NOR metadata endurance is not budgeted

- **Severity:** Medium
- **Confidence:** Risk; workload-dependent
- **Location:** `shared/drivers/nand/nand_disk.c:150-180`; FAT metadata routing design
- **Description:** Every 4 KiB FAT/directory sector update performs a full NOR sector erase, with no wear leveling. The MX25U8035F specifies 100,000 erase/program cycles per sector. Hot FAT/directory sectors can receive disproportionate updates.
- **Why it matters:** Repeated recordings, syncs, file creation, and host enumeration can exhaust a metadata sector long before aggregate NOR capacity would suggest. Adding more `fs_sync()` without wear management may accelerate this.
- **Evidence:** Macronix MX25U8035F features/endurance specification; the disk layer always erases the addressed 4 KiB sector before rewriting it.
- **Recommendation:** Instrument metadata-sector erase counts and model expected field workload before setting a sync cadence. A transactional metadata layer should rotate copies/wear-level hot sectors and retain redundant recovery records.

## Datasheet conformance summary

### Micron MT29F8G01AD / M70A NAND

Correct or substantially correct:

- RESET `FFh`, GET/SET FEATURE `0Fh`/`1Fh`, READ ID `9Fh`, PAGE READ `13h`, READ CACHE `03h`, WREN `06h`, PROGRAM LOAD `02h`, PROGRAM EXECUTE `10h`, and BLOCK ERASE `D8h` encodings and address lengths match the command table (PDF page 15).
- Read uses PAGE READ, polls OIP, then READ CACHE; program uses WREN, PROGRAM LOAD, PROGRAM EXECUTE, then polling; erase uses WREN, D8h, then polling. These are the required high-level sequences (pages 19–21, 32–38).
- Die selection uses SET FEATURE D0h with `00h`/`40h`, matching page 41.
- ECC is enabled by default and the implemented ECC-code decode matches the table. Full 4096-byte page programming is within the maximum four partial-page programs.
- The 8 MHz SPI clock is comfortably below the device limit.

Nonconforming or incomplete aspects are F-01/F-03 (bad blocks), F-05/F-06 (status/error handling), F-14 (refresh), and F-15 (reset timing).

### Macronix MX25U8035F NOR

The active driver is Zephyr's upstream `jedec,spi-nor`, not the repository's custom NOR copy. Devicetree correctly describes 8 Mbit/1 MiB, JEDEC `c2 25 34`, and 8 MHz. The upstream driver provides page splitting for the 256-byte page-program limit, WREN/WIP sequencing, bounds checks, and 4 KiB erase support. The disk layer's 4 KiB erase followed by a 4 KiB write is aligned and supported. The principal NOR issues are architectural—atomicity (F-04) and wear concentration (F-21)—not command conformance.

## Investigated and determined correct

- **NAND address geometry:** `convert_page_to_address()` correctly selects package, die, and a local row in the range 0–131071. Total sector counts fit `uint32_t`; capacity multiplication uses a 64-bit intermediate.
- **Page/block units:** One 4096-byte logical sector equals one NAND data page; 64 pages equals the datasheet's 256 KiB erase block. `DISK_IOCTL_GET_ERASE_BLOCK_SZ` returning 64 sectors is correct.
- **NAND read-cache address:** Three transmitted zero bytes correctly represent two column-address bytes plus the required dummy byte for READ CACHE `03h`.
- **NAND program granularity:** Active calls write a complete 4096-byte page once; this satisfies page size and partial-program limits when the page is erased.
- **NAND ECC decode:** Codes 0/1/2/3/5 are interpreted correctly; the missing action for codes 3/5 is separately recorded in F-14.
- **NOR identity and limits:** Size, JEDEC ID, erase alignment, and 8 MHz frequency match MX25U8035F. Zephyr handles 256-byte page boundaries and synchronous ready polling.
- **Disk physical serialization:** `disk_access_mutex` serializes disk read/write callbacks. Zephyr mutexes are recursive, so optional verify-read recursion does not deadlock. This does not substitute for FatFS/volume-level ownership (F-10).
- **USB host-write rejection in the active stack:** The legacy Zephyr MSC worker is named `usb_mass`; therefore the exact name check rejects host writes while `CONFIG_USB_WRITABLE` is disabled. Read-side coherence remains unsafe (F-10), and the runtime read-only API remains ineffective (F-16).
- **NOR window size:** 180 × 4096 = 737,280 bytes fits in the 1 MiB NOR. The current FatFS format's metadata fits inside that window.
- **ECG accelerometer recorder:** It uses owned blocks, checks short writes, serializes control/data on one filesystem queue, performs periodic sync, and rejects duplicate paths. It is a strong reference for consolidating the legacy sensor writer.
- **Static geometry regression test:** `cmake -DMSENSE_SOURCE_ROOT=D:/MotionSenseHRV4Flash -P shared/tests/cmake/test_storage_geometry.cmake` passed.

## Build results

The normal wrapper was read and used. The first invocation with wrapper defaults failed before CMake because the wrapper attempted `C:\ncs\v2.9.3`, while its README documents the installed SDK as `C:\ncs\SenSEv2.9.3`. This is a build-environment reproducibility issue, not a firmware build failure. Supplying the documented roots produced two complete pristine sysbuild builds, including MCUboot, application core, and HCI IPC network-core images.

### PPGv2

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash\PPGv2' `
  -Board 'ppgv2/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash\PPGv2\build-audit' `
  -NcsRoot 'C:\ncs\SenSEv2.9.3' `
  -ToolchainRoot 'C:\ncs\toolchains\b620d30767' `
  -Pristine
```

- **Result:** Success, exit code 0.
- **Log:** `D:\senselab-tools\logs\ncs-build-20260829-134853-041.log`
- **Application memory:** FLASH 255,452 / 441,856 bytes (57.81%); RAM 242,312 / 458,752 bytes (52.82%).
- **Artifacts:** `PPGv2/build-audit/PPGv2/zephyr/zephyr.elf`, `.hex`, `.bin` (bin 255,452 bytes).

### ECGv0

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash\ECGv0' `
  -Board 'ecgv0/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash\ECGv0\build-audit' `
  -NcsRoot 'C:\ncs\SenSEv2.9.3' `
  -ToolchainRoot 'C:\ncs\toolchains\b620d30767' `
  -Pristine
```

- **Result:** Success, exit code 0.
- **Log:** `D:\senselab-tools\logs\ncs-build-20260829-134853-030.log`
- **Application memory:** FLASH 262,484 / 441,856 bytes (59.40%); RAM 260,136 / 458,752 bytes (56.71%).
- **Artifacts:** `ECGv0/build-audit/ECGv0/zephyr/zephyr.elf`, `.hex`, `.bin` (bin 262,484 bytes).

Both builds contain the storage warnings summarized in F-20. The final CMake warnings `No SOURCES given to Zephyr library: drivers__disk` and `No disk access settings detected` originate from another child/build configuration; the application images did compile and link the custom NAND disk objects, as proven by the preceding storage-source compilation warnings and effective application configuration. Neither build produced a link overflow or storage-related fatal error.

## Prioritized implementation checklist

1. **Stop destructive behavior first:** disable mount-time autoformat and prevent whole-volume erase from touching factory-bad blocks or reporting false success.
2. **Design the NAND translation/recovery model:** persistent redundant BBT, spare blocks, block retirement, ECC refresh, erase-before-reuse, and power-loss-safe mapping. Decide whether FAT remains appropriate above it.
3. **Make NAND operations trustworthy:** bounded polling, separate status/data returns, correct P/E bit checks, complete errno propagation, reset timing, verified init for every package/die.
4. **Make metadata transactional:** redundant copy-on-write NOR metadata with durable CRC/generation/commit and wear leveling; recovery must never autoformat on ambiguous state.
5. **Enforce one volume owner:** serialize all filesystem calls, disconnect/unmount MSC during recording, and use an explicit lifecycle state machine.
6. **Replace legacy async buffering:** owned immutable block pool, checked queue submissions, backpressure/drop telemetry, flush/drain/sync/close ordering for both products.
7. **Fix file semantics and errors:** exclusive unique names, short-write handling, accurate counters, propagated logging failures, checked sync/close/init errors.
8. **Add recovery and fault tests:** factory-bad media, injected SPI errors, stuck OIP, P_FAIL/E_FAIL, ECC 3/5/2, reset during each NOR/NAND phase, full disk, short writes, queue saturation, MSC access, and boundary sectors.
9. **Clean storage warnings and standardize the wrapper default SDK path** so CI/audit signals remain reliable.

## Unknowns and required validation

- **Hardware bad-block inventory:** No pre-erase dump of factory marks or per-device history was available. Read spare markers on representative untouched devices before any further bulk erase and compare across all packages/dies.
- **Power-fail behavior:** Static analysis proves vulnerable windows but not their observed recovery signatures. Use automated power cutting at each metadata erase/program, FAT update, NAND program/erase, sync, close, and full-reset phase; retain raw NOR/NAND images for analysis.
- **Endurance workload:** Recording/file/sync/host-enumeration rates and expected service life were not supplied. Instrument erase counts by NOR sector and NAND block, then calculate worst-case lifetime using real workloads.
- **Throughput/backpressure margin:** Build success does not establish that filesystem latency stays below producer fill time. Measure worst-case queue depth and write latency during NOR metadata updates, ECC events, concurrent MSC reads, near-full FAT scans, and low-voltage conditions.
- **Board-level timing/power guarantees:** The schematic/power-rail ramp and brownout behavior were not part of the supplied scope. Confirm NAND tPOR/tRST and NOR power-down requirements against measured rails and reset sequencing.
- **On-media compatibility:** Existing deployed media may already have erased factory markers or pages programmed more than once. Before migrating, create a forensic/read-only extraction and a versioned conversion/reformat procedure; do not mount with automatic formatting.
- **Dynamic testing:** No hardware was flashed or exercised in this audit. The builds and CMake geometry test validate compilation/configuration, not electrical behavior or persisted data integrity.

## Conclusion

The current implementation is internally coherent enough to build and often work on nominal flash, but it relies on properties raw NAND and FAT do not provide: all-good blocks, rewritable sectors, atomic metadata replacement, and coherent simultaneous filesystem/MSC access. The recommended work should begin with bad-block/erase safety and automatic-format removal before any further consolidation treats the storage interface as stable.
