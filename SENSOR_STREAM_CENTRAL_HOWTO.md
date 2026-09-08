# Central implementation guide: ECG and PPG streaming v0

This guide defines connection setup, stream control, wire decoding and output
retention for a Central receiving ECG or PPG. It is independent of Central
hardware, operating system and UI. The transport version byte is **0** for both
products; ECG's `ECB2` record format and the PPGv2 product name are separate.

All streaming layouts, status values and sensor decoder rules are included here.
[ECG_BLOCK_FORMAT.md](ECG_BLOCK_FORMAT.md) additionally specifies peripheral
recording/files; [PPG_PACKED_16_BYTE_FORMAT.md](PPG_PACKED_16_BYTE_FORMAT.md)
describes PPG files. No file header or trailer is sent in a sensor stream.

## 1. Connect and arrange acquisition

Discover the Nordic UART Service (NUS) and its characteristics:

| Attribute | UUID | Central operation |
| --- | --- | --- |
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | Discover |
| RX | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | Write commands with response |
| TX | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | Receive notifications |
| TX CCCD | `00002902-0000-1000-8000-00805F9B34FB` | Enable notifications |

Discover handles; do not hard-code them. NUS need not appear in advertising.
Know the selected product before decoding: names normally begin with `MSense4ECG`
or `MSense4PPG`, but the application's device-selection context determines the
product. ACK contains no product identity. Do not infer it from notification size.

Negotiate ATT MTU **at least 128** and complete TX subscription before START.
Use the negotiated MTU, not the requested value. The protocol requires no specific
PHY, connection interval or data-length setting; throughput must sustain live
production and drain history. One peripheral TX slot does not mean one
notification per connection interval.

**START never starts acquisition.** Acquisition must already be active or starting;
otherwise START returns NOT_RECORDING. ACK confirms stream acceptance, not
successful sensor startup. History need not be full, and startup may still fail
after ACK.

Both products expose acquisition control separately from NUS:

| Attribute | UUID |
| --- | --- |
| Control service | `da39c930-1d81-48e2-9c68-d0ae4bbd351f` |
| Acquisition enable | `da39c931-1d81-48e2-9c68-d0ae4bbd351f` |

Write one byte with response: `1` requests acquisition and `0` requests normal
acquisition stop. Discover the value handle. ATT success confirms acceptance of
the request, not completion of startup/shutdown. The device must first be awake
and connectable; this characteristic is not a remote ship-mode wake mechanism.
Stream STOP leaves acquisition and storage recording running.

## 2. Commands and session state

Send exactly eight bytes in one RX write with response. All command, envelope
and payload metadata integers are **unsigned little-endian**.
Write-without-response is unsupported.

| Offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 2 | ASCII `MS` (`4d 53`) |
| 2 | 1 | Version `00` |
| 3 | 1 | Opcode |
| 4 | 4 | Nonzero stream ID |

| Opcode | Command | Behavior |
| ---: | --- | --- |
| `0x01` | START (FINITE) | 32,768 history bytes, then 98,304 future bytes |
| `0x02` | STOP | End the matching stream immediately |
| `0x03` | START_INFINITY | 32,768 history bytes, then continuous future data |

Use a fresh ID for every START attempt, including rejected attempts. STOP uses
the active ID. Bind receive/validation state to the connection instance and ID.
Only one stream is active, including cleanup. Reconnect starts a new context;
there is no resume, retransmission or automatic deduplication.

Track the ATT write and its application response separately. Permit only one
unanswered command; do not reuse write parameters before ATT completion. An
application response may arrive before the ATT completion callback.

| Event | Receiver action |
| --- | --- |
| Ready → START | Reset counters/decoder, retain requested ID/mode, await ACK or RESULT |
| Valid ACK | Enter receiving state; expect DATA offset zero |
| START RESULT | Record rejection; return to ready after outstanding write cleanup |
| User Stop before ACK | Hold intent; send STOP after both ACK and START write completion |
| START rejection/disconnect | Discard held Stop intent |
| Receiving → STOP | Continue receiving; await END or matching rejection RESULT |
| STOP RESULT | Resolve command; existing stream remains active |
| END | Accept no further DATA; finish queued complete-record validation and cleanup |
| Disconnect/timeout/failure | Retain valid prefix, discard incomplete tail, retire context |

Reject repeated local STOP while awaiting its response. ATT admission/resource
errors mean a command was not accepted: do not wait for its application response.
Restore the prior state (ready for a rejected START write, receiving for a rejected
STOP write); connection errors may instead require disconnect. A rejected STOP
does not permit another concurrent START. After END and local receive/write
cleanup, another START can reuse the connection.

## 3. Notifications

Each TX notification is one complete binary envelope and payload. Envelopes do
not span notifications; sensor records may span any number of notifications.

| Offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 2 | ASCII `MS` |
| 2 | 1 | Version `0` |
| 3 | 1 | Message type |
| 4 | 4 | Stream ID |
| 8 | 2 | Payload length |
| 10 | 2 | Flags, must be zero |

Require total length exactly `12 + payload_length`. Validate magic, version,
flags, type, ID and type-specific length before consuming the payload.

| Type | Name | Payload |
| ---: | --- | --- |
| `0x81` | START_ACK | Exactly 16 bytes; layout below |
| `0x82` | DATA | u64 byte offset plus at least one sensor byte |
| `0x83` | END | Exactly one u16 terminal status |
| `0x84` | RESULT | Exactly one u16 command-rejection status |

### START_ACK

| Payload offset | Bytes | Required value |
| ---: | ---: | --- |
| 0 | 1 | Mode: 0 FINITE, 1 INFINITY; must match START |
| 1 | 1 | History units: 8 |
| 2 | 6 | All zero |
| 8 | 8 | Total sensor bytes: 131,072 FINITE, 0 INFINITY |

History units are **4,096 bytes for both products**, not PPG records. INFINITY's
zero total means unknown length. Accepted START has exactly one ACK before DATA
or END, even if no DATA follows. Rejected START has RESULT instead. Reject
duplicate ACK or an ACK with an unexpected mode/ID.

### DATA and reassembly

Read the first eight payload bytes as `offset`; the remaining bytes are sensor
data. Require `offset == next_offset`, initially zero. Before addition, check
`fragment_length <= UINT64_MAX - next_offset`. For FINITE also require
`next_offset + fragment_length <= 131072`. Offsets never wrap.

Maximum sensor bytes per notification are `ATT_MTU - 23`, possibly smaller due
to sender capacity. Do not require full-sized fragments. A fragment can split or
combine records and cross the history boundary at offset 32,768.

Append bytes into a bounded partial-record buffer. Validate/decode every complete
record in order, emit it, then reuse the buffer. Advance offsets for every byte,
including zero history. Do not accumulate an entire INFINITY stream or wait for
END before emitting complete records.

Reject DATA before ACK, after END, with empty sensor payload, beyond FINITE's
total, or with duplicate/missing offsets. There is no resynchronization scan or
silent gap skipping. Malformed current-stream data ends reception and triggers
disconnect; retain the earlier valid prefix. Stale callbacks from retired
connections/streams must not mutate the current decoder. Unexpected current-session
messages are errors. An unrelated RESULT must not alter the stream or its timer.

### END and RESULT codes

END contains no byte count: derive it from accepted DATA. Terminal codes are:

| Code | Name | Meaning |
| ---: | --- | --- |
| `0x0000` | SUCCESS | FINITE reached its declared total |
| `0x0001` | NOT_RECORDING | Acquisition ended |
| `0x0008` | STOPPED | Matching STOP won the terminal decision |
| `0x0009` | STORAGE_ERROR | Recording storage fault |
| `0x000a` | INTERNAL_ERROR | Acquisition/internal failure |
| `0x000d` | DISCONNECTED | Locally synthesized connection-loss outcome; END is not expected |
| `0x000e` | BUFFER_OVERFLOW | Required data could not be queued |

RESULT rejects a command without introducing DATA or resetting the active offset:

| Code | Name |
| ---: | --- |
| `0x0001` | NOT_RECORDING |
| `0x0003` | NOT_SUBSCRIBED |
| `0x0004` | BUSY |
| `0x0005` | MTU_TOO_SMALL |
| `0x0006` | INVALID_COMMAND |
| `0x0007` | UNSUPPORTED_VERSION |
| `0x000b` | NOT_INITIALIZED |
| `0x000c` | WRONG_SESSION |

`0x0002` is reserved, not a valid response. Reject unknown codes or codes used
with the wrong message type. Correlate RESULT with the pending command ID.
Wrong/retired-ID STOP receives WRONG_SESSION. Malformed commands that cannot
identify a valid command/ID need not receive RESULT; always form commands correctly
and retain the timeout below.

## 4. ECG decoder

Each ECG record is a **4,096-byte ECB2 block** with exactly **1,358 samples**.
Metadata is little-endian; each three-byte sample word is **MSB first**.

| Block offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 4 | ASCII `ECB2` |
| 4 | 4 | `first_rtc_tick`, u32 |
| 8 | 4 | `first_sample_index`, u32 |
| 12 | 4 | CRC32, u32 |
| 16 | 4,074 | 1,358 three-byte samples |
| 4,090 | 6 | All zero |

### Padding and validation

Recognize an entirely zero block only when wholly within offsets `[0, 32768)`
and before every real block. Skip it without emitting samples or establishing
continuity. Zero history is normal at boot/after acquisition discontinuity.
Partly filled history has leading zero blocks followed by real blocks.

Every other block must have ECB2 magic, zero reserved bytes, valid CRC, ETAG 0–3
for every sample, and continuity with the preceding real block. A zero block
after real data or in future data is an error. A partly zero block is not padding;
it must pass normal validation. Validate the entire block before exposing samples.

CRC is **CRC-32/ISO-HDLC**: reflected polynomial `0xEDB88320`, initial register
`0xffffffff`, final XOR `0xffffffff`. Compute over all 4,096 bytes with bytes
12–15 treated as zero; compare with the stored little-endian CRC. The check value
for ASCII `123456789` is `0xcbf43926`. Use unsigned 32-bit arithmetic:

```text
crc = 0xffffffff
for j in 0..4095:
    crc ^= (0 if 12 <= j < 16 else block[j])
    repeat 8 times:
        crc = (crc >> 1) ^ (0xedb88320 if (crc & 1) else 0)
crc ^= 0xffffffff
```

CRC failure, invalid tags or continuity failure ends decoding at that block;
do not expose it or scan for another magic sequence.

### Samples and timing

For sample `i` in `0..1357`, read bytes at `16 + 3*i`:

```text
raw24 = (byte0 << 16) | (byte1 << 8) | byte2
etag  = (raw24 >> 3) & 7
ptag  = raw24 & 7
u18   = raw24 >> 6
ecg   = u18 - 0x40000 if (u18 & 0x20000) else u18
```

ETAG 0 and 2 contain usable signed ECG counts. ETAG 1 and 3 occupy sample time
but are not usable voltage measurements; preserve their quality indication and
timing. ETAG 4–7 are invalid in a block. Preserve PTAG 0–7 as sensor metadata,
not a stream sequence number. The protocol defines raw counts, not conversion
to volts or calibration.

Sample `i` has tick `first_rtc_tick + i` and ordinal `first_sample_index + i`.
Both sample and tick rates are 512 Hz; time in seconds is tick / 512. Tick zero
is approximately device boot, not UTC or notification-arrival time. The first
real block can start at any recording-local index. Subsequent blocks must satisfy:

```text
next.first_sample_index == previous.first_sample_index + 1358
next.first_rtc_tick     == previous.first_rtc_tick     + 1358
```

Acquisition pauses do not reset the tick, but a new acquisition session resets
the recording-local index. One stream cannot span that discontinuity. Reboot
resets the clock origin. ECG counter-wrap recovery (about 97.1 days for ticks)
is outside scope; use wider arithmetic to detect unsupported overflow rather
than infer continuity across wrap. Do not fabricate samples across time gaps.

## 5. PPG decoder

Each PPG record is **16 bytes**, with unsigned little-endian fields:

| Record offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 3 | Infrared channel 1 |
| 3 | 3 | Infrared channel 2 |
| 6 | 3 | Green channel 1 |
| 9 | 3 | Green channel 2 |
| 12 | 4 | `global_tick_512hz` |

Decode each channel as `b0 | (b1 << 8) | (b2 << 16)`. Reject values above
`0x7ffff`; the upper five bits must be zero. There is no PPG magic, CRC, sample
index or validity flag. No ECG block header/tag decoder applies.

The tick is a 32-bit 512-Hz counter with unsigned modulo-2^32 rollover semantics,
not UTC. Nominal production is 256 records/s (4 KiB/s). Do not impose ECG's exact
continuity arithmetic or require every PPG tick delta to be two. Preserve recorded
timestamps; cadence/order checks are diagnostics, not a CRC substitute.

History may include leading all-zero records. They consume normal offsets/counts.
The format cannot reliably distinguish them from legitimate content: **do not
automatically delete zero records** or claim they are validated measurements.
Report complete framed-record counts and signal quality separately. Complete
framing and valid field ranges do not prove end-to-end content integrity.

## 6. History, endings and retention

| Geometry | ECG | PPG |
| --- | ---: | ---: |
| History bytes | 32,768 | 32,768 |
| History records | 8 blocks | 2,048 records |
| FINITE future bytes | 98,304 | 98,304 |
| FINITE future records | 24 blocks | 6,144 records |
| Live queue capacity | 65,536 bytes | 65,536 bytes |

START snapshots records already incorporated in peripheral history when the owner
processes it. Pending ingress becomes future data, which can therefore include
samples acquired before START was written. Real history and future are consecutive
within a stream; captures may overlap. Cross-stream comparison/deduplication needs
device and boot context because reboot can repeat timestamps. Neither a boot ID
nor a UTC mapping is supplied by this stream envelope.

STOP ends new DATA submission when processed. Already submitted DATA may still
arrive before END. Do not require record-aligned STOP: after processing complete
records, discard the final 1–4,095 ECG bytes or 1–15 PPG bytes. Zero DATA followed
by END is permitted. STOP must not erase previously retained complete output.

For FINITE SUCCESS require exactly 131,072 bytes, no partial record, and successful
product validation: every non-padding ECG block valid, or complete PPG records
with valid fields. ECG padding counts toward length; SUCCESS need not mean 32
usable ECG blocks. INFINITY never ends with SUCCESS. Reaching FINITE's byte total
without END is not a successful terminal outcome.

Most faults attempt a status END; timeout/failed END submission can disconnect.
The first peripheral terminal decision wins: a concurrent fault need not replace
STOPPED/SUCCESS. Acquisition stop/fault clears history and ends the stream; a new
session can immediately stream zero history. Live-queue overflow leaves healthy
rolling history updating. Producer-handoff loss can clear history while storage
recording continues. Status does not certify that all concurrent subsystems were
healthy or that streamed data was persisted in the peripheral file.

At END/disconnect finish queued complete-record validation in order, then discard
the incomplete or invalid trailing record and finalize counts. Invalid records
end the retained prefix; do not expose later records beyond a failure. Keep
validated ECG blocks or complete valid-field PPG records even on fault. Old
asynchronous validation must never write into a new session. Present session
outcome separately from retained-data validity; file and stream tails may differ.

## 7. Bounded scheduling and timeout

Keep notification work bounded. Use a bounded decoder/output queue when decoding
or storage cannot run promptly in the callback. Queue exhaustion is a local
failure: disconnect and retain the prior prefix, rather than silently dropping
bytes. Do not stop receiving just because STOP was requested.

Use a configurable **15-second no-progress timeout**. Start at START; reset on
valid ACK and contiguous accepted DATA, including padding. Unrelated traffic,
duplicates, malformed frames and unrelated RESULT do not refresh it. Stop on END
or START rejection; a rejected STOP leaves the stream and timer active. Expiry
records local TIMEOUT and disconnects. TIMEOUT has no wire status code. There is
no overall INFINITY duration limit.

The peripheral times out ready/in-flight transmissions without completion,
including failed submissions and command RESULT delivery. Waiting solely for the
next sensor record is not a peripheral transmission stall. Do not depend on END
to release resources after connection loss.

## 8. Implementation test vectors

Examples use stream ID 1. Spaces separate hexadecimal bytes:

```text
FINITE START:  4d 53 00 01 01 00 00 00
INFINITY:      4d 53 00 03 01 00 00 00
STOP:          4d 53 00 02 01 00 00 00

FINITE ACK:
4d 53 00 81 01 00 00 00 10 00 00 00
00 08 00 00 00 00 00 00 00 00 02 00 00 00 00 00

STOPPED END:
4d 53 00 83 01 00 00 00 02 00 00 00 08 00

NOT_RECORDING RESULT:
4d 53 00 84 01 00 00 00 02 00 00 00 01 00

DATA at offset zero containing one PPG record:
4d 53 00 82 01 00 00 00 18 00 00 00
00 00 00 00 00 00 00 00
01 00 00 45 23 01 ff ff 07 00 01 00 78 56 34 12
```

The PPG example yields channels `1`, `0x12345`, `0x7ffff`, `0x100` and tick
`0x12345678`. It illustrates framing, not a complete stream.

For an ECG vector, start with 4,096 zero bytes. Set ECB2 magic, first tick 100000,
first index 0. For `i = 0..1357`, write
`raw24 = ((i & 0x3ffff) << 6) | ((i % 4) << 3) | (i % 8)` MSB first at
`16 + 3*i`. CRC is `0xb27dde56`, stored `56 de 7d b2` at byte 12. The next
contiguous block starts at tick 101358 and index 1358.

Exercise fragments split within fields/samples, spanning records and offset 32768;
zero/partial/full history; both ACK/write-completion orderings; STOP before DATA
and mid-record; FINITE totals; invalid CRC/tags/channel values; reserved/unknown
statuses; stale IDs; disconnect; timeout; and bounded-queue exhaustion.
