# MotionSense Central NUS and DFU operations guide

This application is a test Central for the MotionSense PPG and ECG NUS stream
and for application-image BLE DFU. It runs on either an nRF5340 DK application
core or an nRF54L15 DK application core. It is intentionally a controlled
test tool, not a general-purpose production fleet updater.

## Safety and scope

- Do not treat a build artifact as authorization to program a DK. Program or
  reset hardware only through the approved lab procedure.
- DFU v1 uploads **application image 0 only**. It neither erases the active
  image nor confirms a candidate until the Central has reconnected and
  verified the exact expected image as active.
- The host retains the MCUboot `.bin`; the DK receives acknowledged MDFU
  frames over its data VCOM and performs all BLE SMP/MCUmgr sequencing.
- The default Central-to-peer BLE link has no pairing or encryption
  requirement, and the host needs no signing key at run time. MCUboot image
  authentication is a separate target policy: the current PPGv2 and Blinky
  builds use the bundled NCS development RSA-2048 key and therefore require a
  compatible signed image. The host parser checks MCUboot structure and
  SHA-256 TLVs but does not cryptographically verify the signature. See
  [Security policy](#security-policy).
- Never assume `COM4` and `COM5` identify a particular board. They are only
  examples; use the interface-MCU serial/grouping or pass both ports explicitly.

## Build targets and artifacts

Use the managed NCS 2.9.3 workspace and wrapper. Run the two builds serially.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash\central_nus_test' `
  -Board 'nrf5340dk/nrf5340/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340' `
  -Pristine `
  -NcsRoot 'C:\ncs\SenSEv2.9.3' `
  -ToolchainRoot 'C:\ncs\toolchains\b620d30767'

powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
  -ApplicationRoot 'D:\MotionSenseHRV4Flash\central_nus_test' `
  -Board 'nrf54l15dk/nrf54l15/cpuapp' `
  -BuildDirectory 'D:\MotionSenseHRV4Flash\central_nus_test\build_nrf54l15' `
  -Pristine `
  -NcsRoot 'C:\ncs\SenSEv2.9.3' `
  -ToolchainRoot 'C:\ncs\toolchains\b620d30767'
```

The nRF5340 sysbuild output includes `build_nrf5340\merged.hex` for the
application core and `build_nrf5340\merged_CPUNET.hex` for its network-core
controller. The nRF54L15 output uses `build_nrf54l15\merged.hex`; its
controller is application-core-local. The wrapper writes dated logs under
`D:\senselab-tools\logs`.

After identifying the intended DK with `nrfutil device list`, program both
nRF5340 domains in the `domains.yaml` flash order. These commands erase and
reprogram the selected DK, so run them only with explicit authorization:

```powershell
nrfutil --log-output stdout device program --serial-number <serial-number> `
  --core network `
  --firmware D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\merged_CPUNET.hex `
  --options verify=VERIFY_READ,chip_erase_mode=ERASE_CTRL_AP,reset=RESET_NONE

nrfutil --log-output stdout device program --serial-number <serial-number> `
  --core application `
  --firmware D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\merged.hex `
  --options verify=VERIFY_READ,chip_erase_mode=ERASE_CTRL_AP,reset=RESET_NONE

nrfutil --log-output stdout device reset --serial-number <serial-number> `
  --core application --reset-kind=RESET_PIN
```

For nRF54L15, program and verify its single merged image, then reset the
application core:

```powershell
nrfutil --log-output stdout device program --serial-number <serial-number> `
  --core application `
  --firmware D:\MotionSenseHRV4Flash\central_nus_test\build_nrf54l15\merged.hex `
  --options verify=VERIFY_READ,chip_erase_mode=ERASE_CTRL_AP,reset=RESET_NONE

nrfutil --log-output stdout device reset --serial-number <serial-number> `
  --core application --reset-kind=RESET_PIN
```

## Interface-MCU VCOMs

Connect the DK **interface-MCU USB** connector. Assert DTR on both opened
ports. The command VCOM is always ASCII, 115200 baud, 8N1; the binary VCOM is
always binary, 1,000,000 baud, 8N1.

| Board | Command VCOM | Binary VCOM | Flow control |
| --- | --- | --- | --- |
| nRF5340 DK | UARTE1, interface-MCU Serial Port 0 / usually VCOM0 | UARTE0, Serial Port 1 / usually VCOM2 | Binary RTS/CTS enabled |
| nRF54L15 DK | UARTE30, Serial Port 0 | UARTE20, Serial Port 1 | No binary RTS/CTS |

The host script opens the binary port with RTS/CTS when firmware reports
`hwfc=1`; `--binary-rtscts on|off` overrides it. To identify ports manually,
open a candidate at 115200, assert DTR, send `help` plus newline, and keep the
other VCOM exclusively for binary traffic. RTT is diagnostics only.

### Binary-port ownership

The binary UART has exactly three modes:

1. `IDLE`: no NUS relay or DFU reception owns it.
2. `NUS_RELAY`: raw NUS notifications are emitted as `MRLY` frames.
3. `DFU_RX`: MDFU frames are accepted; no NUS relay can start.

After a NUS terminal message is parsed, `STREAM_OK` or `STREAM_END` may appear
before physical UART draining finishes. The reliable hand-off signal is
`RELAY_IDLE`: it is emitted only after the relay queue is empty and its async
UART TX is inactive. A DFU begin request refuses the port until that condition
holds. On DFU completion/abort/failure, async RX is disabled before the port
returns to `IDLE`.

## NUS capture operation

For `MSense4PPG-...` and `MSense4ECG-...` peers, the Central discovers NUS
first, then SMP; missing SMP must not break normal NUS capture. Exactly
`MSenseBlinky` is SMP-only: NUS is skipped and is never required during
postboot DFU verification. Other `MSense` names retain the NUS-first policy.

Send one case-sensitive ASCII command terminated by CR, LF, or CRLF:

| Command | Meaning |
| --- | --- |
| `help` | Print the command summary. |
| `scan` | Report the first advertiser whose name begins `MSense` without connecting. |
| `connect ppg` | Scan and connect to the first `MSense4PPG-...` peer. |
| `connect ecg` | Scan and connect to the first `MSense4ECG-...` peer. |
| `connect any` | Scan and connect to any `MSense` name, including `MSenseBlinky`. |
| `status` | Print stream state, NUS/SMP readiness, MTU, relay state, live throughput, link information, and machine-parseable `peer_name`, `peer_addr`, and `peer_addr_type`. |
| `start [id]` | Request a PPG/ECG stream. `id` is an optional nonzero decimal or `0x` uint32. |
| `cancel [id]` | Cancel a receiving NUS stream; without `id`, uses the active session. |
| `disconnect` | Disconnect the selected BLE peer. |

Important NUS/control events are:

| Event | Meaning |
| --- | --- |
| `CONNECTED`, `DISCONNECTED` | Connection lifecycle. |
| `ATT_MTU`, `BLE_LINK` | Negotiated ATT/connection/PHY/DLE diagnostics. |
| `NUS_READY` / `NUS_UNAVAILABLE` | NUS discovery and subscription result. |
| `NUS_SKIPPED reason=smp_only` | Exact `MSenseBlinky` bypassed NUS discovery. |
| `SMP_READY` / `SMP_UNAVAILABLE` | SMP discovery result (after NUS except for `MSenseBlinky`). |
| `PEER_READY nus=<0|1> smp=<0|1> peer_name=<name> peer_addr=<address> peer_addr_type=<type>` | Final discovery availability and machine-parseable advertised identity for the peer. |
| `START_SENT`, `START_ACK`, `START_RESULT` | Stream request state. |
| `CANCEL_SENT`, `CANCEL_RESULT` | Stream cancellation state. |
| `THROUGHPUT_HISTORY`, `THROUGHPUT_FORWARD`, `THROUGHPUT`, `THROUGHPUT_LIVE` | Phase and total timing statistics. |
| `STREAM_OK` | Protocol and byte-count checks passed; wait for `RELAY_IDLE` before reusing the binary port. |
| `STREAM_END`, `PROTOCOL_ERROR`, `ERROR ...` | Invalid or failed capture. |
| `RELAY_IDLE` | Binary relay has completely drained and can be handed to DFU. |

The PPG stream carries 131,072 sensor bytes and ECG carries 131,076. Relay
frames are little-endian `MRLY` v1 records: magic (4), version (1), type (1),
raw NUS length (u16), relay sequence (u32), then exact raw NUS bytes. They do
not include command text or delimiters.

## DFU command protocol

All DFU commands use the command VCOM. `tx` is a nonzero uint32 transaction
identifier and lets scripts associate events with an operation.

| Command | Meaning |
| --- | --- |
| `dfu capabilities` | Report MDFU version, image scope, baud rates, payload cap, `hwfc`, and security mode. |
| `dfu status` | Report engine state, current transaction, target offset, credit maximum, dropped frames, and binary ownership. |
| `dfu list <tx>` | Read and print the target image list. |
| `dfu begin <tx> <bytes> <file_sha256> <tlv_sha256> [allow-same]` | Preflight then begin/resume application image 0 upload. `file_sha256` is SHA-256 of the complete `.bin`; `tlv_sha256` is the MCUboot SHA-256 TLV. |
| `dfu abort <tx>` | Abort a receiving upload safely. It never confirms an image. |
| `dfu erase <tx> [slot]` | Request image-slot erase (default secondary slot 1). Use only through an intentional recovery procedure. |
| `dfu test <tx> <tlv_sha256>` | Mark a known secondary image for trial boot. |
| `dfu confirm <tx> <tlv_sha256>` | Confirm a known current image. Do not use if postboot identity is uncertain. |
| `dfu reset <tx>` | Request a peer reset. |

Input lines are capped at 255 bytes. An overlong command gets a visible
`ERR command exceeds 255 bytes` response rather than being silently discarded.

### MDFU v1 binary frame

The host sends one frame only after the matching `DFU_CREDIT`. All multibyte
fields are little-endian.

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 4 | ASCII `MDFU` |
| 4 | 1 | Version `1` |
| 5 | 1 | Type `1` = DATA |
| 6 | 2 | Header size `24` |
| 8 | 4 | Transaction ID |
| 12 | 4 | Image offset |
| 16 | 2 | Payload length, at most 384 |
| 18 | 2 | Reserved, zero |
| 20 | 4 | IEEE CRC-32 over bytes 4..19 followed by payload |
| 24 | variable | Image data |

The Central safely resynchronizes after invalid bytes, CRC errors, or malformed
headers. It grants one application-level frame credit at a time. The credit
limit is negotiated ATT MTU minus SMP/CBOR/ATT overhead, aligned down to four
bytes and capped at 384. It accounts for 251-byte LL DLE fragmentation through
the configured ACL/L2CAP buffer sizing. The binary UART uses a 5 ms receive
inactivity timeout at 1 Mbaud, so short invalid frames are delivered to the
parser promptly rather than waiting for a 512-byte buffer to fill.

### DFU events

Scripts should parse whitespace-delimited `key=value` fields and ignore events
with a different `tx`.

| Event | Meaning |
| --- | --- |
| `DFU_CAPS`, `DFU_STATUS` | Static capabilities and current engine state. |
| `DFU_QUEUED` | Command accepted by the dedicated DFU thread. |
| `DFU_LIST_BEGIN`, `DFU_IMAGE`, `DFU_LIST_END`, `DFU_DONE operation=list` | Image-list response. `DFU_IMAGE` includes image/slot/hash/bootable/pending/confirmed/active/permanent fields. |
| `DFU_BEGIN_READY` | Preflight succeeded; includes requested bytes and negotiated chunk maximum. |
| `DFU_CREDIT tx=<...> off=<...> max=<...>` | Send exactly one MDFU DATA frame at that offset, with no more than `max` data bytes. |
| `DFU_PROGRESS` | Target acknowledged an advanced upload offset. |
| `DFU_RETRY` | Repeat the current credit after a wire error, wrong frame, or target offset response. |
| `DFU_RESTART` | The target reported offset zero; resend from zero using new credits. |
| `DFU_RECONNECT`, `DFU_RECONNECTED`, `DFU_RESET_DISCONNECTED` | BLE disconnect/resume processing; reset confirmation waits for the observed disconnect. |
| `DFU_SECONDARY_VERIFIED`, `DFU_TESTED`, `DFU_PENDING_VERIFIED`, `DFU_POSTBOOT_READY`, `DFU_ACTIVE_VERIFIED` | Safe automatic apply milestones. |
| `DFU_SUCCESS` | Exactly one terminal success: expected image is active and confirmed. |
| `DFU_FAIL` | Terminal failure with operation, negative code, and reason; no uncertain image is confirmed. |
| `DFU_ABORT_REQUESTED`, `DFU_ABORTED` | Requested and terminal abort state. |

## Host CLI

`msense_dfu.py` needs only Python standard library and `pyserial`. It enumerates
and groups candidate VCOMs by interface/J-Link serial when metadata is present.
If Windows reports indistinguishable J-Link CDC ports, it queries the read-only
`nrfutil device list --json` inventory and maps VCOM 0 to command and VCOM 1
to binary, honoring `--jlink-serial`. If that inventory is unavailable or
ambiguous, it fails closed and asks for explicit ports.

```powershell
# Inspect candidates; this makes no DFU request.
python .\msense_dfu.py --list-ports

# Explicit ports (COM4/COM5 are examples only).
python .\msense_dfu.py --image D:\images\app_update.bin --target ppg `
  --command-port COM4 --data-port COM5

# Restrict automatic grouping to a known interface-MCU/J-Link serial.
python .\msense_dfu.py --image D:\images\app_update.bin --target ecg --jlink-serial 1050123456

# Preflight image list only, emit machine-readable events, or deliberately
# permit an image that already matches the active slot.
python .\msense_dfu.py --image D:\images\app_update.bin --target ppg --command-port COM4 --data-port COM5 --preflight-only
python .\msense_dfu.py --image D:\images\app_update.bin --target any --command-port COM4 --data-port COM5 --json
python .\msense_dfu.py --image D:\images\app_update.bin --target ppg --command-port COM4 --data-port COM5 --allow-same
```

`--target ppg|ecg|any` is required for an unattended DFU run. The CLI first
queries `status` and verifies its machine-parseable `peer_name` and
`peer_addr`: PPG accepts `MSense4PPG*`, ECG accepts `MSense4ECG*`, and `any`
accepts any `MSense*` name, including `MSenseBlinky`. It preserves only a
matching SMP-ready peer; a mismatched peer is disconnected and observed as
`DISCONNECTED` before `connect <target>`. The
new `PEER_READY ... smp=1` identity is verified again before preflight. In
`--json` mode, command events are JSON Lines on stdout and human upload
progress is written to stderr. Exit status is stable: `0` success,
`2` CLI usage, `3` invalid image, `4` port error, `5` protocol/timeout, `6`
DFU terminal failure, and `130` Ctrl+C. Ctrl+C sends `dfu abort <tx>` on a
best-effort basis and never sends a confirm command.

The parser accepts only `.bin` files with a valid MCUboot image header and a
single 32-byte SHA-256 TLV. It separately computes whole-file SHA-256 for the
upload session and exposes detected signature TLV types. No signature key is
needed or requested by the host at run time. This structural check does not
override the target MCUboot policy; a signature-validating target will reject
an unsigned or incompatibly signed image.

## Automatic apply workflow and recovery

`msense_dfu.py` performs `dfu list` before `dfu begin`. `dfu begin` has one
automatic policy: full upload, trial boot, postboot verification, and confirm.
There is no unattended upload-only or test-only policy selector; use
`--preflight-only` for list-only inspection and the explicit command-port
operations only for deliberate manual recovery. The Central also
preflights image state and rejects an image matching active application image 0
unless `allow-same` is explicit. During the upload it serializes exactly one
SMP request at a time on a dedicated DFU thread; BLE callbacks and the system
workqueue never block on management calls.

When the last byte is acknowledged, the Central checks that the secondary image
has the requested MCUboot TLV hash and is bootable, sends the exact-hash test
request, verifies pending state through image-list data, resets the peer,
requires an observed disconnect, reconnects to the same BLE address, requires
a newly completed discovery epoch, and rediscovers the services required for
the postboot identity. `MSense4PPG-...`, `MSense4ECG-...`, and other generic
`MSense` peers require NUS and SMP. Exact `MSenseBlinky` skips NUS and requires
SMP only. The Central then verifies the requested hash is active, confirms the
currently running image, and re-lists that exact hash as active and confirmed.
Only then is `DFU_SUCCESS` emitted.

### Test #4: PPG to Blinky and restoration

Test #4 is a state-changing end-to-end test. Use an explicitly authorized,
recoverable PPG target and the PPGv2-compatible Blinky application image. With
the Central DK on explicit VCOMs, run:

```powershell
python D:\MotionSenseHRV4Flash\central_nus_test\msense_dfu.py `
  --image D:\MotionSenseHRV4Flash\blinky\build_ppgv2_nrf5340\blinky\zephyr\zephyr.signed.bin `
  --target ppg --command-port COM4 --data-port COM5
```

Do not infer COM4/COM5 on another machine; discover or pass the actual paired
DK ports. A pass requires all of the following:

1. The uploaded Blinky hash is bootable in slot 1 and then marked pending.
2. Reset produces an observed disconnect.
3. The same BLE address reconnects as exact `MSenseBlinky`.
4. `NUS_SKIPPED reason=smp_only`, `SMP_READY`, and
   `PEER_READY nus=0 smp=1 peer_name=MSenseBlinky` are reported.
5. The expected hash is active but unconfirmed before the Central confirms it.
6. The final list reports that hash as active and confirmed, followed by
   `DFU_SUCCESS`.

To restore the normal PPG application after Blinky has been confirmed, use
`--target any`; `--target ppg` intentionally will not select
`MSenseBlinky`:

```powershell
python D:\MotionSenseHRV4Flash\central_nus_test\msense_dfu.py `
  --image D:\MotionSenseHRV4Flash\PPGv2\build\PPGv2\zephyr\zephyr.signed.bin `
  --target any --command-port COM4 --data-port COM5
```

Require `DFU_SUCCESS`, then verify the peer advertises as `MSense4PPG-...` and
reports both `nus=1` and `smp=1`. If the normal PPG artifact matches the active
image, the Central rejects it unless `--allow-same` is deliberately supplied.

Upload frame waits and ordinary upload-resume reconnects are bounded at 30
seconds. After an observed reset disconnect, the Central allows 60 seconds for
MCUboot image swapping, application startup, same-address reconnection, and
service discovery. A wire CRC, incorrect transaction, incorrect offset, or
target-reported offset produces a new/repeated credit; target offset zero
restarts safely from zero. A BLE disconnect during upload causes a same-peer
reconnect and resume; the target remains authoritative for the next offset. If
the host disappears, a management operation fails, or postboot identity cannot
be verified, the terminal result is `DFU_FAIL` and the Central never confirms
the image.

If recovery is needed:

1. Read `dfu status` and `dfu list <new-tx>` after the peer is ready.
2. For an unconfirmed trial image, let MCUboot roll back rather than issuing a
   blind confirm. Use `dfu confirm` only after an exact image-list check.
3. Use `dfu erase` only for a deliberately selected secondary slot and then
   re-list the target. It is a state-changing operation.
4. If an async VCOM transition reports `ERROR DFU binary RX did not stop
   cleanly`, do not begin NUS relay/DFU again on that session; resolve the
   UART/VCOM condition or reset through approved procedures.

## Security policy

`CONFIG_MSENSE_DFU_REQUIRE_SECURITY=n` is the v1 default. It is deliberate:
existing peripherals need no BLE pairing keys, and the host does not need
signature verification material to transmit a prebuilt image. This setting
controls BLE link security only. It does not disable MCUboot image signature
validation. The current PPGv2 and Blinky MCUboot builds validate RSA-2048
signatures using the bundled NCS development key; that key is suitable for lab
interoperability only and must not be treated as a production credential.

A future product deployment can enable BLE SMP/security in the peripheral and
Central and set `CONFIG_MSENSE_DFU_REQUIRE_SECURITY=y`. With that configuration
the Central rejects DFU unless the BLE link has at least `BT_SECURITY_L2`.
MCUboot signing-key provisioning and image acceptance remain an independent
policy and must be designed and tested separately.

Enabling this option is not by itself a key-management solution. Any future
rollout must provision, protect, rotate, and test its BLE and MCUboot keys
separately. The v1 host image parser intentionally remains structure/hash
validation only unless a future explicit policy adds signature verification.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No `MSENSE_CENTRAL_READY` after opening a VCOM | It is likely the binary VCOM or DTR is not asserted. Try `help` at 115200 on the command VCOM. |
| CLI says ports are ambiguous | Use `--list-ports`, then give both `--command-port` and `--data-port`; do not infer identities from COM number. |
| `SMP_UNAVAILABLE` but NUS works | The peer lacks SMP or its GATT service is unavailable; NUS capture remains supported. |
| `NUS_UNAVAILABLE` but SMP is ready | This is valid only for exact `MSenseBlinky`; PPG/ECG and other `MSense` peers require NUS after automatic DFU reboot verification. |
| `ERR dfu ... -16` / binary busy | A NUS relay is active or not drained. Wait for `RELAY_IDLE`; do not close/reopen the data port as a substitute. |
| `DFU_RETRY` or `DFU_RESTART` | Keep the command and data VCOMs open. The script automatically obeys the next credit; do not inject uncredited frames. |
| `DFU_FAIL ... timeout` | Check link range, target power, SMP configuration, and command/data VCOM ownership. Re-list before deciding on recovery. |
| NUS capture reports relay error | Treat capture as invalid. Confirm 1 Mbaud data-port setup, reader availability, and RTS/CTS on the nRF5340 DK. |
| Image parser rejects a file | Supply an MCUboot application `.bin`, not HEX/ELF/ZIP. It must contain a valid image header and SHA-256 TLV. The host parser reports but does not require a signature TLV; the target MCUboot may still require a compatible signature. |

## Validation for future changes

Run host tests and the existing CTest geometry regression before hardware work:

```powershell
python -m unittest discover -s D:\MotionSenseHRV4Flash\central_nus_test\tests -p test_msense_dfu.py
C:\ncs\toolchains\b620d30767\opt\bin\ctest.exe --test-dir D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\central_nus_test --output-on-failure
C:\ncs\toolchains\b620d30767\opt\bin\ctest.exe --test-dir D:\MotionSenseHRV4Flash\central_nus_test\build_nrf54l15\central_nus_test --output-on-failure
```

Then perform the two wrapper builds serially. Hardware validation should begin
with read-only VCOM discovery and `dfu capabilities` / `dfu list`. Programming
or state-changing DFU validation requires explicit authorization, an identified
recoverable target, and a documented restoration path. Once those conditions
are satisfied, the host CLI may run the authorized test unattended.
