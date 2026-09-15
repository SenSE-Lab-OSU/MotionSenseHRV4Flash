# PPG production functional HIL

## Extended streaming endurance HIL

`run_extended_streaming_hil.py` runs the non-destructive 150-minute streaming
campaign. It preserves existing media by content hash and never formats,
flashes, performs DFU, or uses reset codes 68/132. It does intentionally issue
one Central `RESET_SYSTEM`, one PPG storage-aware reset (`reset 120`), BLE
disconnects, and ordinary collection/stream commands. The reset case stops the
stream, sends `collect off`, confirms remote status bytes 1 and 2 are zero,
validates and hashes the closed files, and only then requests reset 120. It
requires the expected disconnect, advertising return, exact-peer reconnect,
service rediscovery, collection restart, post-reboot streaming, preservation of
the pre-reboot hashes, and a newly validated PPG/accelerometer/log file set.

`reset 121` remains available only for a separately labeled emergency or
crash-consistency test. It is not part of this ordinary endurance campaign, and
such a test must not require preservation of media interrupted by reset 121.

Run its hardware-free parser checks first:

```powershell
python PPGv2/tests/hil/run_extended_streaming_hil.py self-test
```

Use current, explicitly verified identities and a new output directory:

```powershell
python PPGv2/tests/hil/run_extended_streaming_hil.py run `
  --command-port <Central-command-COM> --relay-port <Central-relay-COM> `
  --central-jlink-serial <Central-debugger-serial> --nrfutil <nrfutil.exe> `
  --ppg-port <PPG-native-COM> --ppg-usb-serial <PPG-USB-serial> `
  --peer-name <exact-MSense4PPG-name> --drive <PPG-MSC-root> `
  --session-id-base <unique-nonzero-id> --output <new-evidence-directory>
```

The runner records continuous native UART and Central command/relay evidence,
strictly checks MRLY sequence and NUS data offsets, validates each new on-media
PPG/accelerometer/log set, and verifies all prior files after every case. A
planned stream abort is classified expected only after a complete 128 KiB
recovery stream passes. Checkpoints are append-only numbered JSON files; the
final disposition is `summary.json`. On failure, the runner performs bounded
best-effort stop, collection-off, and disconnect cleanup while retaining all
partial evidence.

The exact advertised PPG name is mandatory because `connect ppg` is a
first-match operation. Do not access the MSC volume with another process while
collection owns storage.

This destructive, production-like campaign formats the PPG FatFS volume,
installs the exact production PPG image through the Central's supported MDFU
path, and runs two collection sessions without an intervening format or reboot.
It validates the finite 128 KiB stream and all newly persisted recording files.

## Prerequisites and scope

- Supply explicit paths to `central_nus_test/msense_dfu.py`, the proven
  `ppg_nand4k_production_hil.py`, and the proven `capture_uart.py`.
- Supply the exact MCUboot PPG application `.bin`, Central J-Link serial and its
  command/relay ports, PPG native USB port and USB serial, MSC root, two unique
  stream IDs, DFU transaction, and a new evidence directory. The output
  directory must not already exist; if it does, the runner exits without
  changing it. Historical
  COM/drive/device values are never inferred.
- Supply the literal `--format-confirmation FORMAT_PPG_FATFS_CODE_68`. The
  runner uses the explicit Central command port to connect to a PPG, sends the
  normal BLE `reset 68`, and requires the bounded disconnect/reconnect and
  rediscovery events. Do not substitute 132: it changes bad-block state and is
  outside this campaign. No debugger-based format is permitted. The baseline
  fails if any prior PPG, accelerometer, or log recording remains; `uuid.txt`
  is allowed.
- The format destroys all files on the selected PPG volume. The two sessions
  then permanently consume clusters under the append-only operating model.
  Confirm the exact target and preserve the resulting files; do not delete them
  as cleanup.

## Run

Offline checks:

```powershell
python PPGv2/tests/hil/run_production_hil.py self-test
python PPGv2/tests/hil/run_production_hil.py run --help
python <ppg_nand4k_production_hil.py> --self-test
python -m unittest discover -s central_nus_test/tests -p "test_*.py"
```

Rescan preserved native/Central text evidence without hardware:

```powershell
python PPGv2/tests/hil/run_production_hil.py scan-evidence `
  --input <native-uart.txt> <command.decoded.txt> --summary <scan.json>
```

Campaign skeleton (placeholders are mandatory current values):

```powershell
python PPGv2/tests/hil/run_production_hil.py run `
  --dfu-tool <central_nus_test/msense_dfu.py> `
  --stream-tool <ppg_nand4k_production_hil.py> `
  --capture-tool <capture_uart.py> --image <signed-or-unsigned-app.bin> `
  --command-port <Central-command-COM> --relay-port <Central-relay-COM> `
  --central-jlink-serial <Central-serial> `
  --ppg-port <PPG-native-COM> --ppg-usb-serial <PPG-USB-serial> `
  --drive <PPG-MSC-root> --output <new-evidence-directory> `
  --format-confirmation FORMAT_PPG_FATFS_CODE_68 `
  --preflight-log <complete-native-boot-log.txt> `
  --session-ids <id-1> <id-2> --dfu-transaction <nonzero-id>
```

`--preflight-log` accepts one or more complete native logs captured after the
format/DFU boot and before acquisition. The runner preserves and scans them for
NAND initialization, package/die setup, mount, timeout/`-ETIMEDOUT`,
`P_FAIL`/`E_FAIL`, and SPIM4/shared-storage-bus errors. It does not require a
success marker because successful production initialization may be silent. If
CDC was unavailable during early boot and no complete external capture exists,
the summary records `NOT_PROVIDED`; that setup interval remains an explicit
observability gap.

After successful DFU and MSC baseline settling, the runner opens native CDC
capture. Set `--native-capture-seconds` long enough to remain active through
both sessions. The runner fails if capture ends early, then waits for it to
close normally so the complete UART evidence is preserved. Each session uses
the proven Central flow: connect, `collect on`, finite 131072-byte/8192-record
stream validation, `collect off`, and disconnect. After MSC returns, only newly
named `ppg*.bin`, `ac*.bin`, and `log*.txt` files are copied. All must be
nonempty. PPG and accelerometer files remain exactly 4 MiB with only an erased
`0xFF` suffix after their record-aligned data prefixes. PPG prefixes retain
16-byte records, 19-bit channel values, and plausible wrap-aware ticks.
Accelerometer prefixes must contain complete 26-byte records; no quaternion or
physical-signal assumptions are made. Logs must be UTF-8 text shorter than 4 MiB,
end at their exact logical length without an exposed erased suffix, and contain
the ordered shutdown markers.

Format, DFU, native UART, and Central text are scanned for
NAND/program/write/erase/EIO,
filesystem-corruption, duplicate-program, and related storage failures. Any
such failure from a storage subsystem fails the run; unrelated sensor-register
and connection messages are not storage failures. Bad-block lines are retained
separately
in `summary.json`; they are not evaluated in this campaign. An optional
`--reset-argv-json` performs a post-session reset/remount and verifies all copied
recording files remain byte-identical. After each session and reset, every file
from the preceding phase—including `uuid.txt` and earlier recordings—must still
exist with the same SHA-256 content hash.

Without PPG debugger access or temporary firmware hooks this campaign cannot
observe physical NAND program counts, erasedness before program, internal FIL
ownership, bad-block tables, or native logs emitted before CDC reconnects after
DFU (and the optional reset check is on-media only). It establishes normal
BLE/DFU/stream/MSC and on-media functional behavior only.

## Pass and cleanup

Pass requires successful format and DFU, two distinct no-reboot finite streams,
new valid on-media PPG, accelerometer, and log files after each stop, two ordered
native shutdown-marker sequences, clean native/Central storage scans,
byte-identical preservation of every prior file, and `summary.json` result
`PASS`. Optional reset persistence must pass when
requested. Leave PPG idle with MSC mounted, close all task-owned Python/serial
processes, and preserve the image, snapshots, raw logs, copied files, and summary.
