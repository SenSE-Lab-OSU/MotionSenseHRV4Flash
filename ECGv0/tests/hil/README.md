# ECG filesystem-cleanup HIL

This production-like ECG test runs two normal collection sessions without a
reboot, then validates only files newly created by each session. It exercises
the logger work item's first initialization and its reuse after stop/start. It
does not instrument page writes or test the NAND alignment invariant directly.

## Prerequisites

- Flash the reviewed ECG production image and retain its exact application ELF.
- Use the proven `capture_uart.py`, `short_button.py`, and
  `validate_new_files.py` together in one explicit `--tools-dir`.
- Connect the ECG USB CDC/MSC interface and an external probe configured for the
  ECG target. This runner drives the ECG directly; no Central is used.
- Identify the probe serial, ELF, CDC port, ECG USB serial, MSC drive, and a new
  evidence directory. The output directory must not already exist; if it does,
  the runner exits without changing it. Do not infer values from historical COM
  or drive letters.
- Start awake and idle, or pass `--wake-first`; the first short release then only
  wakes the device. Never substitute a long press.
- Before the baseline snapshot, perform an explicitly reviewed normal FatFS
  format. Supply its exact argv in `--format-argv-json`, or perform it separately
  and pass the literal `--manual-format-confirmation I_FORMATTED_ECG_FATFS`.
  The Central's generic `reset 68` command is one supported normal BLE format
  path when the supplied command waits through `RESET_REDISCOVERED code=68`.
  Never infer a drive or destructive command. The post-format baseline fails if
  any prior ECG, accelerometer, or log recording remains; `uuid.txt` is allowed.

The run consumes NAND clusters for two ECG, accelerometer, and log recordings.
Under the append-only storage model those clusters are not reclaimed online;
do not delete test files as cleanup. Reformat only as a separately authorized
maintenance action. `uuid.txt` is create-once metadata and is deliberately
excluded from per-session new-file checks.

## Run

First run the offline checks:

```powershell
python ECGv0/tests/hil/run_cleanup_hil.py self-test
python ECGv0/tests/hil/run_cleanup_hil.py run --help
```

Preserved text evidence can be rescanned without hardware:

```powershell
python ECGv0/tests/hil/run_cleanup_hil.py scan-evidence --input <uart.txt> --summary <scan.json>
```

Then run with explicit current values (example placeholders only):

```powershell
python ECGv0/tests/hil/run_cleanup_hil.py run `
  --tools-dir <directory-containing-the-three-proven-tools> `
  --probe <probe-serial> --elf <exact-zephyr.elf> `
  --port <COM-port> --usb-serial <ECG-USB-serial> `
  --drive <MSC-root> --output <new-evidence-directory> `
  --wake-first --format-argv-json <reviewed-format-argv.json> `
  --preflight-log <complete-native-boot-log.txt> `
  --acquisition-seconds 30
```

`--preflight-log` accepts one or more complete native logs captured after the
format/reboot and before acquisition. The runner preserves and scans them for
NAND initialization, package/die setup, mount, timeout/`-ETIMEDOUT`,
`P_FAIL`/`E_FAIL`, and SPIM4/shared-storage-bus errors. It does not require a
success string: current production logging may be silent on successful setup.
If no complete log is available, the summary records `NOT_PROVIDED`; initial
driver setup is then a manual observability gap, not a tested pass.

For each session the runner starts a fixed-duration UART capture, injects one normal start
release, performs no probe/debug access during acquisition, injects one stop
release, captures through the later teardown interval, waits for MSC remount, snapshots the
directory, and copies only new `ecg`, `ac`, and `log` files. The button helper
still resolves RAM symbols from the supplied ELF and restores GPIOTE in its
single batch.

Set `--capture-timeout` longer than `--acquisition-seconds` and long enough for
log finalization and filesystem shutdown (the defaults leave 60 seconds after
the stop release). The full capture must contain the ordered shutdown markers;
it does not stop at the last marker, so later close/unmount errors remain visible.

Each copied logger file must contain the ordered shutdown markers, be nonempty
and shorter than 4 MiB, and contain no exposed erased `0xFF` tail. The runner
scans the complete per-session UART capture for NAND/program/write/erase/EIO,
filesystem-corruption, duplicate-program, and related storage failures. Any
such failure from a storage subsystem fails the run; unrelated sensor-register
and connection messages are not storage failures. Bad-block lines are reported
separately
in `summary.json` because bad-block behavior is outside this campaign.
After each session, the runner also requires every file present after the
previous phase—including `uuid.txt` and all earlier recordings—to remain
present with the same SHA-256 content hash.

An optional post-session reset/remount persistence check accepts a JSON file
containing the exact reset command as an argv array:

```json
["nrfutil", "...explicit supported reset arguments..."]
```

Pass it with `--reset-argv-json <file>`. Keep reset arguments explicit and use
the supported system-reset procedure for the connected target.

## Pass criteria

- Both captures observe `Storage log end` and exit normally.
- Each session creates unique files: at least one fixed 4 MiB ECF2/ECB2 ECG
  recording, one ACF3/ACB1 accelerometer recording with its ACT2 trailer at the
  existing endpoint, and one storage log.
- Every prior file is byte-identical after each later session, the two ECG recording IDs differ,
  and CRC/structure/sample checks pass using the proven validators.
- Each new storage log contains, in order: `Leaving ECG collection mode`,
  `Closing storage log`, and `Storage log end`.
- If requested, reset/remount preserves every prior file byte-identically and recordings validate
  again.

`summary.json` is the machine-readable result; any failed required check makes
the runner exit nonzero. Raw/text/chunked UART evidence and copied files remain
under each session directory.

## Cleanup

Leave the ECG awake and idle with MSC remounted. Confirm all runner, Python,
`nrfutil`, and debugger-related processes started for the run have exited, and
confirm the target is not halted. Preserve the exact ELF, UART captures,
snapshots, copied files, and `summary.json` as evidence.

## Extended 3-4 hour collection campaign

`run_extended_collection_hil.py` reuses the capture, button, media-copy, and
production file validators above. It is destructive only at its prerequisite:
an operator must pristine-flash the reviewed ECG image, use the Central's BLE
`reset 68` (`0x44`) full format, wake the ECG if necessary, and retain a
complete native preflight log. The runner never flashes or formats. It refuses
to start unless the exact destructive confirmation is supplied and the mounted
volume contains only `uuid.txt`.

The default 3.5-hour schedule is one 30-minute mixed case, three 10-minute
normal stop/start collection cycles, and a roughly 145-minute collection-only
soak; each case also includes 60 seconds of shutdown UART capture. During the
mixed case it runs a complete finite stream, intentionally disconnects during
an infinite stream, waits through a BLE dropout, reconnects and proves a finite
recovery, resets only the Central with `RESET_SYSTEM` during another infinite
stream, reconnects, and proves recovery again. Each later short collection
cycle includes a finite stream. The soak is deliberately disconnected and
long enough to exercise multiple natural ECG and accelerometer rollovers.

There is intentionally no ECG reboot case. The production image does not expose
the storage-aware reboot routine: its shell is disabled, BLE `121` is an abrupt
emergency reset, and `68`/`132` are destructive. An ECG crash-consistency reset
would be a separate, explicitly destructive campaign.

Run hardware-free checks first:

```powershell
python ECGv0/tests/hil/run_cleanup_hil.py self-test
python ECGv0/tests/hil/run_extended_collection_hil.py self-test
python ECGv0/tests/hil/run_extended_collection_hil.py run --help
```

Then substitute current, independently verified identities (placeholders only):

```powershell
python ECGv0/tests/hil/run_extended_collection_hil.py run `
  --tools-dir <directory-containing-capture_uart-short_button-validator> `
  --probe <ECG-probe-serial> --elf <exact-ECG-zephyr.elf> `
  --port <ECG-native-COM> --usb-serial <ECG-USB-serial> `
  --drive <ECG-MSC-root> --output <new-evidence-directory> `
  --command-port <Central-command-COM> --relay-port <Central-relay-COM> `
  --central-jlink-serial <Central-J-Link-serial> --nrfutil <nrfutil.exe> `
  --peer-name <exact-MSense4ECG-name> --preflight-log <complete-format-boot-log> `
  --wake-first `
  --destructive-confirmation I_FLASHED_PRISTINE_ECG_AND_FORMATTED_FATFS_0X44
```

Use `--already-awake` only if the post-format wake release was already performed
and verified. Do not enable `--relay-rtscts` for the
nRF54L15 DK; use it only when the Central firmware/board explicitly reports
binary hardware flow control.

The campaign fails on a peer name/address/type change, ambiguous identity,
unexpected disconnect, Central protocol/relay error, native assertion/fatal/
overflow/recorder or storage error, missing ordered shutdown, altered earlier
media, copied/live hash mismatch, malformed or non-contiguous MRLY/NUS data,
invalid fixed-size ECF2/ACF3 records, CRC/tag/trailer/erased-tail error, or
within/across-file ECG tick/index and accelerometer sequence gaps. Evidence
includes native UART raw/text/metadata, complete Central command raw/text,
per-stream MRLY, reset stdout/stderr, pre/post snapshots, copied media and logs,
per-case checkpoints, SHA-256/size/mtime manifest, and final `summary.json`.
Stop on the first failure, leave the ECG awake/idle after the runner's normal
stop cleanup, and verify no Python, `nrfutil`, serial, or debugger process from
the campaign remains.
