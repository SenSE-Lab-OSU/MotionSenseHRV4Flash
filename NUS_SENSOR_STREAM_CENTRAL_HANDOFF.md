# NUS bounded sensor stream: BLE central handoff

Status: firmware protocol implemented; Central integration and hardware validation pending  
Protocol version: 1

## Purpose

This document tells a BLE Central implementation how to find a MotionSense PPG
or ECG peripheral, request its fixed bounded sensor stream, parse the NUS
framing, validate completion, and handle errors.

The Central may be a phone, computer, gateway, or embedded BLE Central. It is
assumed to be controlled by the same system owner as the firmware. Protocol
version 1 is unauthenticated and uses GATT notifications without an
application-layer acknowledgment.

For the sensor record payload itself, refer to:

- PPG: `PPG_PACKED_16_BYTE_FORMAT.md`;
- ECG: `ECG_TEMP_DATA_FORMAT.md`.

Treat record payload as opaque bytes until START_ACK identifies the device and
record format.

## Implementation status

Protocol version 1 is compiled successfully into both the `PPGv2` and `ECGv0`
firmware targets. The peripheral implementation uses an application-owned
standard NUS service; the former optional BLE logging backend is disabled.
The framing, constants, state values, record geometry, and metadata layouts in
this document describe the as-built firmware contract.

The firmware uses a dedicated 2,048-byte stream thread stack and three fixed
notification TX slots. Those details do not alter Central parsing or ordering
requirements. PPG and ECG wrapper builds passed on 2026-09-01, but no Central
implementation or physical BLE interoperability test has yet validated the
recommended timeout and retry values below.

## Successful transaction summary

1. Scan for a MotionSense peripheral and connect.
2. Discover the standard Nordic UART Service.
3. Discover its TX, TX CCCD, and RX characteristics.
4. Negotiate or observe an ATT MTU of at least 128.
5. Subscribe to TX notifications.
6. Write one fixed START command to RX.
7. Receive START_ACK.
8. Receive ordered DATA notifications containing history and then forward
   sensor records.
9. Receive END with SUCCESS.
10. Confirm that DATA contained exactly 98,304 sensor bytes and the record/count
   metadata matches START_ACK.

The first portion was recorded immediately before START; the second portion is
captured after START. The peripheral may take approximately 16 seconds (PPG)
or 10.7 seconds (ECG) merely to acquire the future portion. Do not impose a
short request timeout.

## Locating the peripheral

Current device names are complete local names of up to 16 ASCII characters,
for example:

```text
MSense4PPG-8NR1S
MSense4ECG-xxxxx
```

The name suffix is derived from the hardware identity. Exact ECG/PPG prefixes
are firmware configuration, so production discovery should prefer known
MotionSense prefixes plus post-connect service discovery rather than matching
one example literally.

The advertising payload currently contains:

- standard advertising flags;
- vendor-specific service data whose first 16 bytes are the application's
  control-service UUID and whose following eight bytes are the raw hardware
  device ID;
- the complete local name in scan response data.

The NUS UUID is not guaranteed to be advertised because legacy advertising is
space constrained. Do not reject a device merely because the NUS UUID is
absent from advertising. Connect to a candidate and discover services.

The authoritative identity for a stream transaction is START_ACK, not a cached
advertising name.

## NUS GATT discovery

Discover these UUIDs after connection:

```text
Nordic UART Service:    6E400001-B5A3-F393-E0A9-E50E24DCCA9E
UART RX characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
UART TX characteristic: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
TX CCCD:                 00002902-0000-1000-8000-00805F9B34FB
```

Direction names are from the peripheral's perspective:

- Central writes commands to peripheral RX (`...0002...`).
- Central receives notifications from peripheral TX (`...0003...`).

Subscribe to TX before sending START. Wait for the platform's subscription
completion callback or successful CCCD write. A command sent before the
subscription is active is rejected as `NOT_SUBSCRIBED`.

The service previously carried optional Zephyr log text. Protocol version 1
uses it for framed binary sensor streaming; do not interpret notifications as
UTF-8 log lines.

## ATT MTU and connection behavior

The protocol requires a negotiated ATT MTU of at least 128. The START_ACK
notification is 108 bytes. On platforms such as iOS where MTU exchange is
managed by the OS, query the maximum update value length or wait until service
discovery/subscription completes and use the platform-reported value.

If the effective MTU is below 128, START is rejected with `MTU_TOO_SMALL` in a
compact RESULT message. Reconnect or adjust the Central's GATT configuration;
do not loop START requests.

The peripheral supports one connection. Notifications remain ordered on the
ATT bearer. The Central must nevertheless validate DATA sequence and record
indices so incomplete sessions are detected explicitly.

## Byte-order and message boundaries

- All framing integers are little-endian unless explicitly signed.
- Every GATT TX notification is exactly one complete protocol message.
- Do not concatenate notification byte arrays and search for delimiters.
- Validate each notification independently using its common header and payload
  length.
- Sensor bytes retain their record-format-specific byte order.
- There is no protocol CRC in version 1.

## Sending commands

Every command written to RX is exactly eight bytes:

| Offset | Size | Field | Value |
| ---: | ---: | --- | --- |
| 0 | 1 | Magic 0 | `0x4D` (`M`) |
| 1 | 1 | Magic 1 | `0x53` (`S`) |
| 2 | 1 | Protocol version | `0x01` |
| 3 | 1 | Opcode | See below |
| 4 | 4 | Request/session ID | Nonzero `uint32_le` |

Opcodes:

| Value | Name |
| ---: | --- |
| `0x01` | START |
| `0x02` | CANCEL |

Use Write With Response when available so malformed write errors are visible.
A write acknowledgment means only that the command reached the GATT server;
START_ACK or RESULT determines whether START was accepted.

Choose a nonzero START request ID that is unique among this connection's
outstanding/recent operations. A random 32-bit value or monotonic counter is
acceptable. The accepted request ID becomes the session ID in every TX message.

START has no size or duration argument. Every accepted version-1 START requests
the fixed 96 KiB sensor payload.

To cancel, write CANCEL with the active session ID. Do not assume local
cancellation is complete until END/CANCELLED arrives or the connection closes.

## Common TX header

Every notification begins with this 12-byte header:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | Magic 0: `0x4D` |
| 1 | 1 | Magic 1: `0x53` |
| 2 | 1 | Protocol version: `0x01` |
| 3 | 1 | Message type |
| 4 | 4 | Session ID, `uint32_le` |
| 8 | 2 | Payload length, `uint16_le` |
| 10 | 2 | Flags, zero in version 1 |

Message types:

| Value | Name |
| ---: | --- |
| `0x81` | START_ACK |
| `0x82` | DATA |
| `0x83` | END |
| `0x84` | RESULT |

For every notification:

1. Require at least 12 bytes.
2. Verify magic and protocol version.
3. Verify `notification_length == 12 + payload_length`.
4. Verify the session ID matches the pending or active request.
5. Reject unsupported message types or invalid reserved fields.

Ignore notifications belonging to an explicitly abandoned prior session, but
log them as a protocol violation if they arrive after another START was
accepted on the same connection.

## START_ACK

An accepted START produces exactly one START_ACK before DATA. Its payload is 96
bytes:

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | Device type: `1=PPG`, `2=ECG` |
| 1 | 1 | Record-format version |
| 2 | 2 | Record size, `uint16_le` |
| 4 | 4 | Record-rate numerator, `uint32_le` |
| 8 | 4 | Record-rate denominator, `uint32_le` |
| 12 | 4 | History record count, `uint32_le` |
| 16 | 4 | Forward record count, `uint32_le` |
| 20 | 4 | Total sensor payload bytes, `uint32_le` |
| 24 | 8 | Raw 64-bit hardware Device ID bytes |
| 32 | 1 | Device-name length |
| 33 | 16 | Zero-padded device name bytes |
| 49 | 40 | Full firmware Git commit, lowercase hexadecimal ASCII |
| 89 | 1 | Git tree state: `0=clean`, `1=dirty`, `2=unknown` |
| 90 | 6 | Reserved, all zero |

Version-1 validation expectations:

| Device | Record size | Rate | History records | Forward records | Total bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| PPG | 16 | 256/1 Hz | 2,048 | 4,096 | 98,304 |
| ECG | 12 | 512/1 Hz | 2,731 | 5,461 | 98,304 |

Reject START_ACK if:

- name length exceeds 16;
- bytes after the name through the 16-byte name field are nonzero;
- the Git commit is not 40 hexadecimal characters;
- reserved bytes are nonzero;
- counts multiplied by record size do not equal the total;
- total is not 98,304 in protocol version 1;
- the device-type record geometry does not match the table.

Render Device ID as 16 uppercase hexadecimal characters by formatting each of
the eight bytes in transmitted order. Do not decode the field as a host-endian
integer. The result matches the firmware's existing `uuid.txt` Device ID.

The device name is exactly the first `device_name_length` bytes of the fixed
name field. It is ASCII-compatible UTF-8 and has no required terminator.

Store START_ACK metadata with the received recording so provenance includes at
least device type, name, 64-bit ID, Git commit, Git tree state, protocol
version, and record-format version. A dirty Git tree means the commit does not
fully identify the running source.

## DATA framing

Each DATA payload begins with this 12-byte prefix:

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | DATA sequence, `uint32_le` |
| 4 | 4 | First record index, `uint32_le` |
| 8 | 2 | Record count, `uint16_le` |
| 10 | 1 | Phase: `0=history`, `1=forward` |
| 11 | 1 | Reserved, zero |
| 12 | variable | Whole sensor records |

Validation for every DATA message:

```text
payload_length == 12 + record_count * START_ACK.record_size
```

Additional requirements:

- `record_count` is nonzero.
- First DATA sequence is zero and increments by one.
- First record index is zero and then advances by prior record count.
- History messages contain indices below `history_record_count`.
- Forward messages begin at `history_record_count`.
- No message crosses the history/forward boundary.
- Final record index plus count equals history plus forward counts.

Append only the bytes after the DATA prefix to the output sensor payload. Do
not write NUS framing into the raw `.ppg`/`.ecg` stream file unless defining a
separate container format.

The Central may process records incrementally or retain the 98,304-byte payload
in memory. Retaining the full payload makes validation and atomic persistence
simpler.

## RESULT

Rejected START and immediate command results use a four-byte payload:

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Status, `uint16_le` |
| 2 | 1 | Current peripheral stream state |
| 3 | 1 | Reserved, zero |

RESULT is terminal for a rejected START. Do not wait for END after RESULT.

## END

An accepted session terminates with END. Its payload is 24 bytes:

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Final status, `uint16_le` |
| 2 | 1 | Peripheral state after cleanup |
| 3 | 1 | Reserved, zero |
| 4 | 4 | History records sent, `uint32_le` |
| 8 | 4 | Forward records captured, `uint32_le` |
| 12 | 4 | Total sensor bytes sent, `uint32_le` |
| 16 | 4 | DATA message count, `uint32_le` |
| 20 | 4 | Signed detail value, `int32_le` |

For SUCCESS require:

- history records equal START_ACK history count;
- forward records equal START_ACK forward count;
- sensor bytes equal 98,304 and the locally accumulated byte count;
- DATA message count equals the number locally received;
- detail is zero;
- all sequence and record-index checks passed.

Only after those checks should the Central mark the stream complete and make
the raw sensor payload available to consumers. A non-success END is terminal
and the partial payload must be marked incomplete or discarded according to
the Central application's policy.

## Status and state values

Statuses:

| Value | Name | Central action |
| ---: | --- | --- |
| `0x0000` | SUCCESS | Validate and commit the complete payload. |
| `0x0001` | NOT_RECORDING | Do not retry until the user/device starts recording. |
| `0x0002` | HISTORY_NOT_READY | Retry only after the history fill interval. |
| `0x0003` | NOT_SUBSCRIBED | Fix TX subscription before retrying. |
| `0x0004` | BUSY | Wait for current transition/session to terminate. |
| `0x0005` | MTU_TOO_SMALL | Reconfigure/reconnect with ATT MTU at least 128. |
| `0x0006` | INVALID_COMMAND | Treat as Central implementation error. |
| `0x0007` | UNSUPPORTED_VERSION | Stop; negotiate/update software out of band. |
| `0x0008` | CANCELLED | Discard or mark partial payload cancelled. |
| `0x0009` | STORAGE_ERROR | Surface device fault; do not immediate-loop retry. |
| `0x000A` | INTERNAL_ERROR | Surface device fault and retain diagnostics. |
| `0x000B` | NOT_INITIALIZED | Treat as firmware startup/configuration fault. |
| `0x000C` | WRONG_SESSION | Correct the CANCEL session ID. |
| `0x000D` | DISCONNECTED | Normally inferred locally rather than received. |

Peripheral stream states:

| Value | Name |
| ---: | --- |
| `0x00` | NOT_RECORDING |
| `0x01` | HISTORY_FILLING |
| `0x02` | READY |
| `0x03` | ACTIVE |
| `0x04` | ABORTING |
| `0x05` | UNINITIALIZED |

Unknown statuses are terminal failures. Unknown states may be displayed for
diagnostics but must not override status handling.

## Timing and retry policy

The successful stream includes future acquisition, so use progress-aware
timeouts rather than a single short transaction timeout.

Recommended initial Central policy:

- START_ACK/RESULT timeout after successful RX write: 5 seconds.
- In-session idle timeout while connected: 5 seconds without any DATA or END.
- Overall PPG transaction timeout: at least 45 seconds.
- Overall ECG transaction timeout: at least 35 seconds.

These are host safety bounds, not expected durations. Reset the idle timer on
every valid session notification. They are provisional until measured against
the implemented firmware on representative Central platforms and under slow
BLE link conditions.

After `HISTORY_NOT_READY`, minimum theoretical fresh-history intervals are:

- PPG: 8 seconds;
- ECG: approximately 5.334 seconds.

Use a margin before retrying, or expose a user-driven retry. The protocol does
not send an unsolicited READY event in version 1.

Do not automatically retry `STORAGE_ERROR`, `INTERNAL_ERROR`,
`UNSUPPORTED_VERSION`, or malformed framing in a tight loop.

## Disconnect handling

Any disconnect before validated END/SUCCESS aborts the transaction. The
peripheral discards the session and does not resume it after reconnect.

On reconnect:

1. Rediscover handles if required by the platform/cache policy.
2. Re-establish TX subscription.
3. Wait for a fresh history interval before expecting START to succeed.
4. Use a new nonzero request ID.

Never append data from a new connection/session to an incomplete prior payload.

## Central parser state machine

A minimal parser uses:

```text
DISCONNECTED
  -> DISCOVERING
  -> SUBSCRIBED
  -> START_PENDING
  -> RECEIVING
  -> COMPLETE
```

Terminal side paths are `REJECTED`, `CANCELLED`, `FAILED`, and
`DISCONNECTED_ABORT`.

Rules:

- In `START_PENDING`, accept only START_ACK or RESULT for the request ID.
- In `RECEIVING`, accept DATA and one terminal END for the active session.
- DATA before START_ACK is a protocol error.
- A second START_ACK is a protocol error.
- RESULT after accepted START_ACK is a protocol error; accepted sessions use
  END for termination.
- DATA after END is a protocol error.
- A session is successful only after END/SUCCESS and all local count checks.

## Suggested stored capture metadata

The Central should persist a sidecar or container header containing:

- protocol version;
- device type;
- device name;
- 64-bit Device ID hex string;
- full firmware Git commit;
- Git tree state;
- record-format version;
- record size and rate;
- history and forward record counts;
- START request/session ID;
- Central wall-clock request and completion times;
- total bytes and DATA message count;
- final status.

Do not inject this metadata into the raw sensor byte sequence if existing tools
expect a bare concatenation of disk records.

## Central acceptance tests pending

None of the tests in this section are implied by the successful firmware
builds; they remain required for Central integration and hardware acceptance.

- Locate a device without relying on NUS advertising.
- Discover the correct NUS RX/TX directions.
- Verify START before subscription returns NOT_SUBSCRIBED.
- Verify MTU below 128 returns MTU_TOO_SMALL.
- Parse and validate fixed START_ACK metadata for both device types.
- Confirm Device ID formatting matches the device's `uuid.txt` identity.
- Confirm Git commit matches the flashed build artifact.
- Reassemble exactly 98,304 sensor bytes across different negotiated MTUs.
- Validate sequence, record indices, phase boundary, and END counts.
- Confirm PPG and ECG record streams parse using their referenced documents.
- Confirm NOT_RECORDING and HISTORY_NOT_READY handling.
- Confirm immediate request after success is rejected until fresh history fills.
- Cancel during history and forward phases.
- Disconnect during START_PENDING, history, forward acquisition, and final drain.
- Inject a dropped notification into a parser unit test and prove the session is
  rejected by sequence/index/count validation.
- Prove partial sessions are never mislabeled complete.

## Security scope

Protocol version 1 does not require encryption, bonding, authentication, or
authorization. Any connected Central that can discover and write the NUS RX
characteristic may request data. This is an explicit product decision for the
initial implementation, not an implicit security guarantee.

## Out of scope

- Starting recording from the Central.
- Variable history/forward durations.
- Fetching records from disk.
- Application-layer ACK/retransmission.
- Resume after disconnect.
- Multiple concurrent sessions.
- Log text over NUS.
- Interpretation of the PPG/ECG record internals beyond the referenced format
  documents.
