# Consolidation Phase 3 — storage reconciliation

## Scope and safety boundary

Phase 3 makes the raw NAND disk driver and the Zephyr filesystem-log backend
shared. The initial consolidation did not change either product's recording
descriptors, filename conventions, or binary record writers; those remain
application-owned compatibility contracts. The initial implementation and
agent validation did not access media. The owner subsequently exercised the
Phase 3 firmware on hardware and, on 2026-09-01, reported that testing was
complete and approved continuing to Phase 4.

The detailed device-by-device test matrix and raw logs were not added to this
record, so that approval must not be interpreted as evidence that every fault-
injection, endurance, power-loss, near-full, or factory-bad-block case was
executed. The known production-storage risks remain documented in
`STORAGE_AUDIT.md` and `docs/ecg-filesystem-corruption-review.md`.

## Reconciliation ledger

| Source commit | Retained intent | Phase 3 implementation |
| --- | --- | --- |
| main `9d76b98` | recursive disk serialization, error/status counters, file-table NOR diagnostics and in-session CRC | shared `drivers/nand/nand_disk.c` retains the mutex, counters, CRC tracking, NOR failures, duplicate-write handling, and now returns the first read/write failure |
| main `2b4077f` | erase-block ioctl is measured in sectors | `DISK_IOCTL_GET_ERASE_BLOCK_SZ` returns the configured pages-per-erase-block; one raw sector is one NAND page |
| main `295f80a`; ECG `43d07fd` | LTO helper repair | already patch-equivalent before Phase 3; no duplicate port required |
| ECG `dc7f29b` | reject unknown NAND ECC states with useful diagnostics | shared `spi_nand.c` accepts only clean/corrected/uncorrectable MT29 status values and returns `-EIO` for reserved status codes |
| ECG `5718d2e` | exact 8-Gbit geometry | each board now declares a 1,073,741,824-byte package, two dies, 4,096-byte pages, and 64 pages per erase block; total capacity is derived in 64-bit arithmetic |
| ECG `070793f`, `6b30943`, `5acb491` | cleaner filesystem logs | the common log backend does not add terminal newlines; application log policy remains narrow and product-owned |
| ECG `9de2f2d`, `90c6e28` | NAND filesystem logging and reset policy | log-backend mechanism is in `shared/storage_log_backend.c`; each product supplies only its collection/panic policy. Existing reset-cause application policy is unchanged. |
| ECG `24ea8c7`; main `ac23549` | USB behavior during collection | the shared disk driver rejects writes with `-EROFS` whenever firmware owns the disk. It never reports rejected USB writes as successful. CDC enable/disable remains application policy pending host behavior validation. |

## Geometry and ownership model

The custom NAND disk node describes one package. Its parent SPI controller's
`cs-gpios` list is the only source of package population. The driver asserts
that its length equals `num-flashchips`, and validates the page/block/die
geometry for the supported MT29 part. PPGv2 builds a four-package (4 GiB)
population; ECGv0 builds a two-package (2 GiB) population. The total is never
stored in a 32-bit byte property.

`disk_access` owns the outer recursive mutex. It is the only entry point that
may re-enter the disk path (duplicate-write checks and verify-readback). The
SPI helper's inner semaphore remains private to an individual transfer.

When runtime read-only is enabled, all writers receive `-EROFS`, including USB
mass storage. This is intentional: a host must not be told that a rejected
write was durable.

The die erase loop uses the half-open range `0 .. block_count - 1`. With the
MT29 geometry, a die has exactly 2,048 erase blocks, so block 2,048 is never
addressed. Package/die selection now validates the selected package before
indexing its state and logs only the selected package/die, rather than reading
four fixed array slots. Multi-die erase and package initialization now return
the first failure instead of overwriting it with a later operation.

The multi-package erase path also propagates a NOR file-table erase failure:
it logs the failure and returns its error immediately. `all erase complete` is
therefore emitted only after every NAND package and the NOR file table succeed.

## Filesystem boundary

The shared log backend invokes three application hooks:

- whether a log write is currently allowed;
- how to append bytes to the product's log stream;
- panic-time flush/close policy.

PPGv2 keeps its collection-gated log policy. ECGv0 keeps its existing policy.
Their `zephyrfilesystem.c` files retain their typed product stream enums and
record-format behavior rather than imposing a false common enum.

## Build validation

Earlier integration logs retained for Phase 3 history (before the final
file-table return-path correction) are:

- PPGv2: `D:\senselab-tools\logs\ncs-build-20260828-213257-776.log`
- ECGv0: `D:\senselab-tools\logs\ncs-build-20260828-213439-698.log`

Final clean-tree firmware evidence is commit `7442059c27b6a94919151cc54fe4d5527037fcd4`:

- PPGv2: `D:\senselab-tools\logs\ncs-build-20260828-214852-911.log`
- ECGv0: `D:\senselab-tools\logs\ncs-build-20260828-215035-785.log`

The existing `msense_git_metadata` and `msense_storage_geometry` CTests passed
in both application build directories. The geometry test statically verifies
both DTS populations, two dies per package, the 2,048-block die geometry, the
exclusive erase bound, absence of fixed package-array reads, and the NOR
file-table error return preceding the all-erase success log. These static/build
checks do not constitute destructive NAND/NOR persistence, boundary, ECC
injection, FAT, USB host, full-disk, or recording-compatibility hardware
validation. At the time these build records were created, those media/hardware
checks remained the outstanding Phase 3 acceptance gate; the later owner
acceptance is recorded below.

## Hardware acceptance and post-validation work

The owner subsequently tested the Phase 3 firmware on representative hardware
and accepted the phase on 2026-09-01. This owner acceptance closes the Phase 3
gate for purposes of continuing the consolidation. It does not close the
production-readiness findings in `STORAGE_AUDIT.md`, including bad-block
management, power-loss-safe metadata, erase/recovery safety, bounded NAND
polling, and fault-injection coverage.

Testing exposed additional storage-lifecycle and application issues. Their
final implementation was compacted into two local commits:

- `1084876` (`feat(storage): enforce exclusive filesystem ownership`) adds the
  storage audit records, consumes intentionally discarded storage-log output,
  enforces exclusive firmware-versus-host MSC ownership, writes `uuid.txt`
  during the boot filesystem session, and tears the filesystem down before
  publishing read-only media to the host.
- `03949b5` (`refactor(ecgv0): remove inactive PPG compatibility paths`)
  removes the vestigial ECG PPG surface and increases the ECG and PPG
  collection-transition stacks needed by the storage lifecycle.

The owner rewrote this local history by squashing the earlier fine-grained
commits into the two commits above. The code was not changed by that rewrite.
The Phase 4 baseline is therefore local branch
`codex/ppgv2-ecgv0-consolidation` at `03949b5`.

Later builds temporarily emitted `MSENSE_GIT_COMMIT` and
`MSENSE_GIT_TREE_STATE` as `unknown`. The owner traced this to the shared build
wrapper and reports that the wrapper is now fixed; it was not a firmware-source
defect. The first Phase 4 validation builds must nevertheless confirm that both
generated metadata headers again contain the actual commit and clean/dirty
tree state.
