# NUS bounded sensor stream: firmware handoff

Status: implemented in both firmware targets; hardware validation pending  
Protocol version: 1  
Targets: `PPGv2` and `ECGv0` on nRF5340  
SDK: project-managed nRF Connect SDK 2.9.3 workspace

## Purpose

This document specifies an on-demand Bluetooth stream that returns recent and
future sensor records over a Nordic UART Service (NUS)-compatible GATT service.
It is the firmware handoff for implementation and verification.

A successful request returns a device-specific bounded sensor-record payload:

- PPG: exactly 131,072 bytes (128 KiB);
- ECG: exactly 131,076 bytes;
- approximately 32 KiB captured before the request from a RAM history ring;
- exactly 96 KiB captured after the request in a forward buffer.

The records sent over Bluetooth are byte-for-byte copies of the records
accepted by the disk-storage pipeline. Protocol framing is additional to the
device-specific sensor bytes reported in START_ACK.

The feature is available only while the device is already recording. A stream
request must never start recording, change acquisition settings, or change
storage ownership. Only one Bluetooth connection and one stream session are
supported.

## As-built implementation status

Protocol version 1 is implemented for both applications. The shared module is:

```text
shared/include/msense_sensor_stream.h
shared/include/msense_sensor_stream_protocol.h
shared/sensor_stream.c
```

Both applications enable `CONFIG_MSENSE_SENSOR_STREAM=y` and disable
`CONFIG_LOG_BACKEND_BLE`. The shared implementation owns the standard NUS
service, one dedicated 2,048-byte stream thread stack, three fixed TX slots,
the 131,076-byte capture buffer sized for the larger whole-record payload,
command processing, packetization, and session cleanup. It performs no dynamic
allocation.

The PPG and ECG producer integrations call the stream module only after
`store_data()` accepts the corresponding disk-format record. Recording
lifecycle and storage-failure notifications are connected to the existing
authoritative application transitions.

Serial wrapper builds completed successfully for both targets on 2026-09-01:

- PPG log: `D:\senselab-tools\logs\ncs-build-20260901-231241-195.log`;
- ECG log: `D:\senselab-tools\logs\ncs-build-20260901-231526-509.log`.

Static review, configuration inspection, linker inspection, and
`git diff --check` passed. No new stream-related compiler errors or warnings
were found. A source-only CMake regression test guards the exact PPG and ECG
geometry. Hardware/on-air interoperability, throughput, chronology,
disconnect, cancellation, and fault-injection testing remain pending.

## Existing system

### Applications

The repository contains two separate Zephyr applications:

- `PPGv2`, built for `ppgv2/nrf5340/cpuapp`;
- `ECGv0`, built for `ecgv0/nrf5340/cpuapp`.

Both use the project-managed NCS workspace at `C:\ncs\SenSEv2.9.3` and Nordic
toolchain at `C:\ncs\toolchains\b620d30767`.

### Replaced NUS-shaped logging service

Before this feature, both applications set `CONFIG_LOG_BACKEND_BLE=y`. That
patched Zephyr BLE log backend defined the standard NUS service, TX
characteristic, RX characteristic, and TX CCCD in:

`C:\ncs\SenSEv2.9.3\zephyr\subsys\logging\backends\log_backend_ble.c`

That service streamed logs on TX when notifications were enabled. Its RX
characteristic existed for application compatibility but had no write handler.
It is not Nordic's `CONFIG_BT_NUS` library; `CONFIG_BT_NUS` is disabled.

The implementation now sets `CONFIG_LOG_BACKEND_BLE=n` in both applications
and defines one application-owned NUS-compatible service. There is no second
service with the NUS UUID. BLE log streaming is intentionally out of scope for
protocol version 1. A later version may add logs as another framed message
class without changing the sensor record contract.

### Device identity and Git metadata

`shared/device_identity.c` obtains an eight-byte hardware ID with
`hwinfo_get_device_id()`. `shared/include/msense_device_identity.h` defines:

```text
MSENSE_DEVICE_ID_LEN = 8
MSENSE_DEVICE_NAME_MAX_LEN = 16
```

The device name is the configured model prefix followed by a five-character
Crockford Base32 digest, for example `MSense4PPG-8NR1S`.

The build generates `msense_git_metadata.h`. `MSENSE_GIT_COMMIT` is the full
40-character repository commit hash in lowercase hexadecimal for a valid
build, or `unknown` if metadata generation failed. Production stream builds
must fail a source-level or runtime initialization check if the commit is not
exactly 40 hexadecimal characters. `MSENSE_GIT_TREE_STATE` is `clean`, `dirty`,
or `unknown` and is included in START_ACK as provenance.

### Stored sensor records

PPG records are documented in `PPG_PACKED_16_BYTE_FORMAT.md`.

- Size: 16 bytes.
- Record rate: 256 records/s.
- Record tick basis: 512 Hz; consecutive records normally differ by two ticks.
- Producer: `PPGv2/src/ppgSensor.c`.
- Mirror point: after `store_data(ppg_record, sizeof(ppg_record), ppg)` returns
  zero.

ECG records are documented in `ECG_TEMP_DATA_FORMAT.md`.

- Size: 12 bytes.
- Record rate: 512 records/s.
- Record tick basis: 512 Hz; consecutive time-valid records differ by one tick.
- Producer: `ECGv0/src/ecgRecorder.c`.
- Mirror point: after `store_data(frame, sizeof(frame), ecg)` returns zero.

"Accepted by disk" means accepted into the filesystem's asynchronous RAM
buffering pipeline. It does not mean that `fs_write()` has physically completed.
No record may enter the NUS history or forward buffer if `store_data()` rejected
that record.

## Fixed capture geometry

The shared implementation allocates 131,076 bytes, which fits the larger ECG
payload. Each START_ACK and END reports its device-specific payload total;
protocol TX packet buffers and the stream thread stack are separate.

### PPG geometry

| Portion | Records | Bytes | Nominal duration |
| --- | ---: | ---: | ---: |
| History | 2,048 | 32,768 | 8 s |
| Forward | 6,144 | 98,304 | 24 s |
| Total | 8,192 | 131,072 | 32 s |

### ECG geometry

Twelve-byte records do not divide 32 KiB exactly, so the existing 2,731-record
history remains 32,772 bytes. The 8,192-record forward portion is exactly
96 KiB; the full payload is therefore four bytes larger than 128 KiB.

| Portion | Records | Bytes | Nominal duration |
| --- | ---: | ---: | ---: |
| History | 2,731 | 32,772 | 5.333984375 s |
| Forward | 8,192 | 98,304 | 16 s |
| Total | 10,923 | 131,076 | 21.333984375 s |

Compile-time assertions enforce record size, portion counts, portion byte
sizes, and total byte size. These values are not computed from floating-point
durations at runtime.

## Implemented firmware architecture

### Shared module

The reusable stream module resides in the repository's shared Zephyr module
and exposes a small application-facing API:

```text
shared/include/msense_sensor_stream.h
shared/sensor_stream.c
shared/include/msense_sensor_stream_protocol.h
```

The shared module owns:

- the standard NUS GATT service;
- command validation;
- stream state and synchronization;
- the 131,076-byte record buffer;
- history-ring and forward-buffer cursors;
- notification packetization and pacing;
- connection and CCCD state;
- terminal status generation;
- stream diagnostics.

The PPG and ECG applications supply an immutable configuration at
initialization:

- device type;
- record-format version;
- record size;
- record rate numerator and denominator;
- history and forward record counts;
- initialized device identity;
- full Git commit and tree state;
- a recording-state query or explicit lifecycle notification.

The module must not depend on PPG, ECG, filesystem, or sensor-driver headers.

### Implemented public API

The shared module exposes this application-facing API:

```c
int msense_sensor_stream_init(const struct msense_sensor_stream_config *config);
void msense_sensor_stream_recording_started(void);
void msense_sensor_stream_recording_stopped(void);
int msense_sensor_stream_accept_record(const void *record, size_t record_size);
void msense_sensor_stream_storage_failed(int error);
```

`accept_record()` is called only after `store_data()` returns zero. It must be
bounded, nonblocking, allocation-free, and safe in the PPG workqueue and ECG
recorder-thread contexts. It returns quickly after copying one 12- or 16-byte
record or determining that no copy is currently needed.

The recording lifecycle must use the authoritative collection transition,
not infer recording from BLE connection state. Starting a new recording clears
old history before accepting new records. Stopping recording aborts a stream,
clears both portions, and changes the state to `NOT_RECORDING`.

### Execution contexts

Use these ownership rules:

- Sensor producer: copies one accepted record and advances bounded counters.
- NUS RX GATT callback: validates length/magic/version and enqueues a command;
  it performs no blocking work.
- CCCD callback: updates subscription state and signals the stream thread.
- BLE connection callbacks: hold/release the connection with correct
  `bt_conn_ref()`/`bt_conn_unref()` ownership and signal the stream thread.
- Dedicated stream thread: owns session transitions, packet construction,
  notification submission, retry policy, and terminal messages.
- Notification completion callback: releases a fixed TX slot and signals the
  stream thread; it does not construct or submit additional packets directly.

Use statically allocated storage. Do not allocate in acquisition steady state,
GATT callbacks, or notification callbacks.

### Synchronization

Protect record bytes and multi-field cursor updates with a short spinlock or an
equally bounded mechanism. Never hold that lock while calling Bluetooth,
logging, filesystem, or kernel wait APIs.

The producer copies the record before publishing its updated count. The TX
thread must never observe a count for a partially copied record.

Use a semaphore/event to wake the TX thread when:

- a command arrives;
- a new forward record arrives;
- TX capacity is released;
- the CCCD changes;
- connection state changes;
- recording starts or stops;
- storage fails.

## State model

The externally meaningful states are assigned these protocol values:

| Value | State | Meaning |
| ---: | --- | --- |
| `0x00` | `NOT_RECORDING` | Acquisition/storage recording is inactive. |
| `0x01` | `HISTORY_FILLING` | Recording is active but fresh history is incomplete. |
| `0x02` | `READY` | Recording is active and a complete fresh history is available. |
| `0x03` | `ACTIVE` | One accepted request is being captured/transmitted. |
| `0x04` | `ABORTING` | Cleanup of a failed/cancelled session is in progress. |
| `0x05` | `UNINITIALIZED` | Stream configuration/identity is not ready. |

### Recording start

1. Clear the history and forward counters.
2. Set `HISTORY_FILLING`.
3. Append every successfully stored primary sensor record to the history ring.
4. After exactly the configured history record count, set `READY`.
5. Continue overwriting the oldest history record for every subsequent record.

No history is retained across recording sessions.

### Accepted START

A START is accepted only when all of these are true:

- state is `READY`;
- the device is connected;
- NUS TX notifications are enabled;
- the stream module is initialized;
- the negotiated ATT MTU is at least 128;
- no session is active.

On acceptance:

1. Use the request ID as the session ID.
2. Freeze the history ring and snapshot its oldest-record index.
3. Reset the forward count to zero.
4. Set state to `ACTIVE`.
5. Queue START_ACK before any DATA notification.
6. Transmit frozen history oldest-first.
7. Route the next configured number of accepted sensor records only to the
   forward portion, never to history.
8. Transmit whole forward records as they become available.
9. Queue END/SUCCESS after all configured device-specific sensor bytes have
    been submitted in DATA messages in order.

### Rebuilding history without retransmitting the forward window

The 96 KiB forward window is excluded permanently from later history. Reuse
the history region only after both conditions are true:

- every frozen-history DATA notification has completed and no TX slot refers
  to the history region;
- forward capture has reached its configured record count.

At that point, reset the history ring and begin filling it with records produced
after the forward window, even if the BLE worker is still draining the separate
forward region. A new START remains rejected until the prior session is
terminal and the fresh history ring is full. This guarantees that every
successful response is contiguous and that previously transmitted forward
records cannot be returned as later history.

### Disconnect

On disconnect:

- stop submitting notifications;
- discard the active session and all unsent data;
- release the connection reference;
- clear history and forward state;
- if recording continues, start a new `HISTORY_FILLING` window with the next
  accepted record;
- do not resume the old session after reconnect.

No terminal message can be delivered after the link is gone. The central
infers abort from disconnect or failure to receive END.

### CANCEL

A valid CANCEL names the active session ID. Stop producing DATA, exclude every
record already captured for that session from future history, send END with
`CANCELLED` if the connection remains usable, clear both portions, and begin a
fresh history window from the next accepted record. A CANCEL with a different
session ID returns `WRONG_SESSION`.

### Storage error

If storage rejects the primary record, do not mirror it. The existing storage
fault policy remains authoritative. If a stream is active, terminate it with
`STORAGE_ERROR` when possible, clear its buffers, and follow the application's
normal collection/storage-fault transition. The stream module must not attempt
storage recovery.

## GATT service

Use the standard NUS UUIDs:

```text
Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
RX:      6E400002-B5A3-F393-E0A9-E50E24DCCA9E
TX:      6E400003-B5A3-F393-E0A9-E50E24DCCA9E
CCCD:    00002902-0000-1000-8000-00805F9B34FB
```

Properties and permissions:

- TX: Notify, no direct read requirement.
- TX CCCD: Read and Write.
- RX: Write and Write Without Response may both be exposed, but commands are
  idempotent only by request/session ID; Write With Response is recommended.
- No authentication or encryption permissions in protocol version 1.

The service does not need to be present in advertising data. Existing
advertising space is used by flags, control-service data, the eight-byte device
ID, and the complete local name. Centrals discover NUS after connecting.

## Byte-order and general protocol rules

- All multibyte framing integers are unsigned little-endian unless explicitly
  marked signed.
- Sensor record bytes are opaque to the framing layer and retain their existing
  documented byte order.
- Reserved bytes and flag bits must be transmitted as zero.
- Receivers ignore unknown flag bits but reject unsupported protocol versions.
- No framing CRC is used in version 1. BLE link-layer integrity is accepted as
  sufficient. Sequence numbers, record indices, and terminal counts detect
  incomplete application streams.
- One ATT notification contains exactly one complete protocol message.
- DATA messages contain one or more whole sensor records and never mix history
  and forward records.
- Notifications are submitted in protocol order.

## RX command format

Every RX command is exactly eight bytes:

| Offset | Size | Field | Value |
| ---: | ---: | --- | --- |
| 0 | 1 | Magic 0 | `0x4D` (`M`) |
| 1 | 1 | Magic 1 | `0x53` (`S`) |
| 2 | 1 | Protocol version | `0x01` |
| 3 | 1 | Opcode | See below |
| 4 | 4 | Request/session ID | Nonzero `uint32_le` |

Opcodes:

| Value | Name | Semantics |
| ---: | --- | --- |
| `0x01` | `START` | Request the fixed history-plus-forward capture. |
| `0x02` | `CANCEL` | Cancel the active session whose ID is in bytes 4-7. |

The central selects a nonzero START request ID. An accepted request uses it as
the session ID in every response. Repeating a START with the active ID must not
create a second session; return `BUSY`. Version 1 does not persist request IDs
across disconnects or reboot.

Malformed writes return the appropriate ATT error for invalid length when the
write uses a response. If a message can be sent safely, also send RESULT with
`INVALID_COMMAND` or `UNSUPPORTED_VERSION`.

## Common TX header

Every TX notification begins with this 12-byte header:

| Offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 1 | Magic 0 | `0x4D` (`M`) |
| 1 | 1 | Magic 1 | `0x53` (`S`) |
| 2 | 1 | Protocol version | `0x01` |
| 3 | 1 | Message type | See below |
| 4 | 4 | Session ID | START request ID, `uint32_le` |
| 8 | 2 | Payload length | Bytes following this header, `uint16_le` |
| 10 | 2 | Flags | Zero in version 1 |

Message types:

| Value | Name |
| ---: | --- |
| `0x81` | `START_ACK` |
| `0x82` | `DATA` |
| `0x83` | `END` |
| `0x84` | `RESULT` |

## START_ACK payload

START_ACK is sent only for an accepted START. Its payload is exactly 96 bytes.
The complete notification is 108 bytes and therefore fits the required ATT
MTU of at least 128.

| Payload offset | Size | Field | PPG | ECG |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | Device type | `0x01` | `0x02` |
| 1 | 1 | Record-format version | `0x01` | `0x01` |
| 2 | 2 | Record size, `uint16_le` | 16 | 12 |
| 4 | 4 | Record-rate numerator, `uint32_le` | 256 | 512 |
| 8 | 4 | Record-rate denominator, `uint32_le` | 1 | 1 |
| 12 | 4 | History record count, `uint32_le` | 2,048 | 2,731 |
| 16 | 4 | Forward record count, `uint32_le` | 6,144 | 8,192 |
| 20 | 4 | Total sensor payload bytes, `uint32_le` | 131,072 | 131,076 |
| 24 | 8 | Device ID | Raw `msense_device_identity_bytes()` order |
| 32 | 1 | Device-name length | 1-16 |
| 33 | 16 | Device name | UTF-8/ASCII, zero padded, no required terminator |
| 49 | 40 | Git commit | Full lowercase hexadecimal ASCII commit hash |
| 89 | 1 | Git tree state | `0=clean`, `1=dirty`, `2=unknown` |
| 90 | 6 | Reserved | All zero |

For display, convert Device ID bytes to 16 uppercase hexadecimal characters in
the same byte order used by `msense_device_identity_hex()`. Do not reinterpret
the ID as a little-endian integer.

The Git commit field names the firmware repository commit embedded at build
time, not the NCS or Zephyr commit. A dirty tree is explicitly reported because
the commit alone cannot identify uncommitted source changes.

## DATA payload

The DATA payload begins with a 12-byte data prefix followed by sensor records:

| Payload offset | Size | Field | Description |
| ---: | ---: | --- | --- |
| 0 | 4 | Data sequence | Starts at zero; increments for each DATA message |
| 4 | 4 | First record index | Zero-based index in the full device-specific stream |
| 8 | 2 | Record count | Whole records in this message |
| 10 | 1 | Phase | `0x00=history`, `0x01=forward` |
| 11 | 1 | Reserved | Zero |
| 12 | variable | Record payload | `record_count * record_size` bytes |

Common-header payload length must equal:

```text
12 + record_count * record_size
```

The first DATA message has sequence zero and record index zero. Record indices
increase without gaps. History occupies indices `[0, history_count)`. Forward
occupies `[history_count, history_count + forward_count)`. A DATA message must
not cross that boundary.

At runtime, calculate maximum records per DATA message as:

```text
floor((bt_gatt_get_mtu(conn) - 3 - 12 - 12) / record_size)
```

The subtracted terms are ATT notification overhead, common header, and DATA
prefix. Never assume the configured local L2CAP MTU was negotiated by the
central.

## RESULT payload

RESULT is a compact four-byte rejection or command result that fits even at
the default ATT MTU of 23:

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Status, `uint16_le` |
| 2 | 1 | Current state |
| 3 | 1 | Reserved, zero |

Rejected START commands produce RESULT and no START_ACK or DATA. RESULT for a
START uses the requested session ID from the command.

## END payload

END is exactly 24 bytes:

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Final status, `uint16_le` |
| 2 | 1 | State after terminal cleanup |
| 3 | 1 | Reserved, zero |
| 4 | 4 | History records sent, `uint32_le` |
| 8 | 4 | Forward records captured, `uint32_le` |
| 12 | 4 | Total sensor bytes sent, `uint32_le` |
| 16 | 4 | DATA message count, `uint32_le` |
| 20 | 4 | Detail, signed `int32_le` |

For SUCCESS, detail is zero and the counts must equal START_ACK. For internal
or storage failures, detail contains the negative Zephyr errno when one is
available. END is terminal for the session. No DATA follows END.

## Status values

| Value | Name | Meaning |
| ---: | --- | --- |
| `0x0000` | `SUCCESS` | Complete configured device-specific stream. |
| `0x0001` | `NOT_RECORDING` | Device is not recording. |
| `0x0002` | `HISTORY_NOT_READY` | A complete fresh history window is unavailable. |
| `0x0003` | `NOT_SUBSCRIBED` | TX notifications are not enabled. |
| `0x0004` | `BUSY` | Another session or transition is active. |
| `0x0005` | `MTU_TOO_SMALL` | Negotiated ATT MTU is below 128. |
| `0x0006` | `INVALID_COMMAND` | Invalid magic, length, opcode, or zero ID. |
| `0x0007` | `UNSUPPORTED_VERSION` | Protocol version is not supported. |
| `0x0008` | `CANCELLED` | Session was cancelled by the central. |
| `0x0009` | `STORAGE_ERROR` | Storage rejected a primary sensor record. |
| `0x000A` | `INTERNAL_ERROR` | Unexpected bounded implementation failure. |
| `0x000B` | `NOT_INITIALIZED` | Stream metadata/configuration is unavailable. |
| `0x000C` | `WRONG_SESSION` | CANCEL did not name the active session. |
| `0x000D` | `DISCONNECTED` | Local diagnostic only; normally cannot be sent. |

Unknown status values are treated as terminal failures by the central.

## Notification flow control

Use `bt_gatt_notify_cb()` with a fixed pool of packet buffers and parameter
objects. Do not point an in-flight notification at stack storage or mutable
ring data that may be overwritten.

The implementation uses three fixed stream TX slots
(`CONFIG_MSENSE_SENSOR_STREAM_TX_SLOTS=3`). Increase that count or related
Bluetooth TX resources only if measurement shows starvation. `-ENOMEM` can
mean the payload does not fit the negotiated MTU or TX resources are
temporarily exhausted. Payload size is validated before submission; the
remaining `-ENOMEM` case is treated as congestion and retried after bounded
waiting for completion/capacity.

The notification completion callback means the host has handed the packet to
the controller; notifications have no application-layer acknowledgment. This
is accepted for version 1. Buffer reuse follows the callback contract. END is
submitted only after all DATA messages have been submitted in order; completion
tracking must prevent session memory from being reused while an in-flight
notification references it.

## Performance requirements

Primary acquisition and disk recording have priority over BLE. The stream must
not change their sampling rates or block their producer contexts.

Steady sensor rates are:

- PPG: 4,096 sensor bytes/s;
- ECG: 6,144 sensor bytes/s.

With an ATT MTU of 185, the maximum sensor portion after 24 bytes of framing is
158 bytes, allowing nine PPG records or thirteen ECG records per DATA message.
This is approximately 29 PPG or 40 ECG DATA notifications/s. At ATT MTU 247,
the values are thirteen PPG or eighteen ECG records per DATA message. Exact
throughput depends on the phone, negotiated connection interval, PHY, data
length, and controller scheduling.

Required behavior under a slow link:

- history and forward capture remain lossless within the fixed device-specific
  payload window;
- disk recording remains correct;
- forward capture stops after its exact record count;
- BLE may finish draining after capture ends;
- a second START remains rejected;
- no busy loop or unbounded retry occurs.

## Memory requirements

The expanded build includes the 131,076-byte capture buffer, stream code,
three TX slots, and the 2,048-byte stream thread stack. Final linker reports
must preserve the 64 KiB application-RAM headroom gate for both products.

Remaining memory-validation gates:

- preserve at least 64 KiB linker RAM headroom in both application images;
- use a dedicated, measured stream-thread stack;
- run thread-analyzer/high-water measurements during simultaneous acquisition,
  NAND writes, and BLE streaming;
- use no unbounded heap allocation;
- report RAM/flash deltas against a directly comparable pre-feature build if
  that historical measurement is required.

## Implemented integration sequence

Steps 1-10 below are complete except that dedicated host-native tests from
steps 1-2 were not added. The authorized serial wrapper builds in step 11
passed. Hardware work in step 12 remains pending.

1. Add protocol constants and host-native protocol-layout tests.
2. Add shared stream state/history logic without BLE and test wrap/freeze/order.
3. Disable `CONFIG_LOG_BACKEND_BLE` in both applications.
4. Add the application-owned NUS service and command/CCCD handling.
5. Pass initialized identity and Git metadata into stream initialization.
6. Connect recording lifecycle transitions for both applications.
7. Add the PPG post-`store_data()` mirror call.
8. Add the ECG post-`store_data()` mirror call.
9. Add the dedicated TX thread, fixed packet pool, MTU packetization, and END.
10. Add disconnect, CANCEL, storage-failure, and history-rebuild behavior.
11. Run static/source tests, host-native tests, then authorized serial wrapper
    builds for both targets.
12. Perform hardware throughput, chronology, and fault-injection tests.

## Remaining verification

The following are acceptance requirements, not claims of completed testing.
The initial implementation was checked by static audit and successful PPG and
ECG firmware builds. It has not yet been exercised over a physical BLE link.

### Host-native tests not yet implemented

- All command and TX structs have the specified sizes and offsets.
- Little-endian encoding is explicit and independent of host alignment.
- START_ACK contains exact identity/name/commit bytes and zero padding.
- PPG geometry is 2,048 + 6,144 records and exactly 131,072 bytes.
- ECG geometry is 2,731 + 8,192 records and exactly 131,076 bytes.
- History wrap is emitted oldest-first.
- START freezes a complete history snapshot.
- Exactly the configured next records enter forward storage.
- Forward records never enter the next history generation.
- No DATA message crosses history/forward phase boundary.
- Packetization is correct at MTU 128, 185, 247, and 498.
- Sequence and record indices are gap-free.
- Requests are rejected in every invalid state with the expected RESULT.
- CANCEL and simulated disconnect discard the session and rebuild history.
- Storage rejection excludes the failed record and terminates the session.

### Hardware acceptance tests pending

- Discover the standard NUS service and subscribe to TX.
- Verify START_ACK name, 64-bit ID, device type, and running firmware commit.
- Verify exactly 131,072 PPG or 131,076 ECG sensor bytes and a SUCCESS END.
- Compare every streamed byte against the corresponding persisted disk records.
- Verify history-to-forward tick continuity, including 32-bit tick wrap tests if
  practical.
- Confirm PPG response is 32 seconds and ECG response is about 21.334 seconds
  by record count/ticks.
- Request before history is full and observe `HISTORY_NOT_READY`.
- Complete a stream, immediately request again, and observe
  `HISTORY_NOT_READY`; retry after the fresh window and succeed without any
  record from the prior forward window.
- Disconnect during history, forward capture, and forward drain; reconnect and
  verify the old session does not resume.
- Exercise slow BLE conditions and confirm no acquisition or storage loss.
- Exercise storage failure and confirm stream termination follows storage fault
  policy.
- Confirm logging remains available through UART/RTT as configured but is not
  sent over NUS in version 1.

## Out of scope

- Starting or stopping recording through NUS START.
- Variable response sizes or requested durations.
- Reading historical records from NAND.
- Multiple simultaneous stream sessions or BLE connections.
- Application-layer acknowledgments or retransmission.
- Resume after disconnect.
- Authentication, encryption, bonding, or authorization changes.
- BLE log streaming over NUS.
- Changes to PPG or ECG disk record formats.
