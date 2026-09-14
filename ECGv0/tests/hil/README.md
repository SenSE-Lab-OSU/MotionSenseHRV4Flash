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
  evidence directory. Do not infer them from historical COM or drive letters.
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
  --acquisition-seconds 30
```

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
- Existing recording files are unchanged, the two ECG recording IDs differ,
  and CRC/structure/sample checks pass using the proven validators.
- Each new storage log contains, in order: `Leaving ECG collection mode`,
  `Closing storage log`, and `Storage log end`.
- If requested, reset/remount preserves byte-identical files and they validate
  again.

`summary.json` is the machine-readable result; any failed required check makes
the runner exit nonzero. Raw/text/chunked UART evidence and copied files remain
under each session directory.

## Cleanup

Leave the ECG awake and idle with MSC remounted. Confirm all runner, Python,
`nrfutil`, and debugger-related processes started for the run have exited, and
confirm the target is not halted. Preserve the exact ELF, UART captures,
snapshots, copied files, and `summary.json` as evidence.
