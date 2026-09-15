# PPG production functional HIL

## Extended streaming endurance HIL

`run_extended_streaming_hil.py` runs a destructive-at-start, production-image
150-minute campaign (configurable from 120 through 180 minutes). It begins with
the normal BLE `reset 68` full FatFS format. It does not build, flash, perform
DFU, use reset 132, or alter firmware. It issues
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
  --format-confirmation FORMAT_PPG_FATFS_CODE_68 `
  --session-id-base <unique-nonzero-id> --output <new-evidence-directory>
```

Default timeline (elapsed time includes control and validation overhead):

- 0--10 minutes: inventory and hash the pre-format root, start identity-checked
  native UART capture, verify the exact BLE peer, issue reset 68, require all
  disconnect/advertise/reconnect/rediscovery milestones, and prove the clean
  root contains at most `uuid.txt`.
- 10--25: 15-minute INFINITY baseline followed by a commanded record-boundary
  stop and media validation.
- 25--40: stream for 5 minutes, deliberately disconnect BLE for 3 minutes,
  reconnect to the same address/name/type, then require a complete FINITE
  128-KiB recovery stream.
- 40--55: stream for 5 minutes, reset only the explicitly identified Central
  debugger with `RESET_SYSTEM`, reopen both VCOMs, reconnect to the same PPG,
  and require a complete recovery stream.
- 55--70: stop a 5-minute BLE stream at a record boundary, send `collect off`,
  confirm remote status bytes 1 and 2 are zero, validate and hash the closed
  files, issue storage-safe reset 120, require exact-peer rediscovery, then run
  a complete recovery and revalidate preservation.
- 70--90: eight collection on/off cycles alternating complete FINITE streams
  and 60-second INFINITY streams; disconnect/reconnect after every second cycle.
- Remaining time through about 145 minutes: one INFINITY soak. The final five
  minutes are reserved for stop, unmount/remount, copied-media validation,
  hashes, idle-state confirmation, and cleanup.

The runner continuously captures native UART across PPG resets and verifies it
by USB serial (a changed COM number fails unless explicitly allowed). It keeps
raw Central command and MRLY relay bytes, checks relay sequence, NUS framing and
offset continuity, validates 16-byte PPG and 26-byte accelerometer records and
their wrap-aware ticks, and validates every new exact-length log. All copied
media must match its settled on-device SHA-256. Size, modification time, and
SHA-256 for every captured evidence file (excluding the final summary and the
manifest itself) are written to `evidence-manifest.json`.
The long soak is expected to exercise PPG file rollover: a completely written
4-MiB PPG file is valid only when every record passes, while every partial PPG
or accelerometer file must retain a record-aligned erased `0xFF` suffix.

Pass requires exact-peer reset-68/120 recovery, clean format, every planned
stream and collection transition, a full FINITE recovery after each planned
abort, confirmed collection/storage shutdown before and after reset 120,
complete file sets, record/tick checks, byte-identical preservation of all
files created after format, and no NAND/storage, assertion/fatal,
protocol/relay, overflow, or reset-timeout finding. Expected disconnects and
non-storage warnings are retained under `review_lines`; they do not mask hard
failures. A failed recovery makes the preceding planned dropout a failure.
Checkpoints are append-only numbered JSON files. On failure the runner performs
bounded best-effort stop, collection-off, and disconnect cleanup while retaining
partial evidence. Re-scan text evidence without hardware using:

```powershell
python PPGv2/tests/hil/run_extended_streaming_hil.py scan-evidence `
  --input <native-uart.txt> <central-command.txt> --summary <rescan.json>
```

The literal format confirmation, exact advertised PPG name, USB serial, current
Central debugger serial/VCOMs, and a new evidence directory are mandatory.
`connect ppg` is a first-match operation, so a name mismatch is disconnected
and never accepted. The format permanently destroys the selected PPG volume;
do not access that MSC volume from another process while collection owns it.

## Two-session format/DFU functional HIL

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
