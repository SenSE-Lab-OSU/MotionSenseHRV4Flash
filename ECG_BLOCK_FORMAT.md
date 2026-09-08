# ECG block format v2 and shared stream protocol v0

Status: **implemented; software review and bounded ECG and PPG HIL accepted**.
Updated September 8, 2026. This is the authoritative target contract for ECB2/ECF2
and the shared PPG/ECG v0 transport. This revision adds serialized owner execution,
single-producer ingestion, one TX slot, sequential commands and first-decision termination,
and relaxes startup/storage guarantees. Prior implementation and verification
in sections 14, 16 and 17 do not establish compliance with these new changes.
Implementation verification is recorded in sections 19 and 20. Historical HIL
used transport version 3. Current source specifies version byte 0; those runs
do not establish hardware verification of that later byte change.
Historical build paths are mutable and may hold newer artifacts.

The central is the receiving BLE application. The deployed central will be a
smartphone; `central_nus_test` is a lab-only implementation, not a deployed
architecture requirement. Companion implementation guides are
[SENSOR_STREAM_CENTRAL_HOWTO.md](SENSOR_STREAM_CENTRAL_HOWTO.md) and
[SENSOR_STREAM_FIRMWARE_HOWTO.md](SENSOR_STREAM_FIRMWARE_HOWTO.md).

The existing ECB2/ECF2 layouts and v0 envelope identifiers are retained for this
lab-stage revision. This is not backward behavioral compatibility: sender and
receiver must be updated together. There is no negotiation or legacy fallback.

## 1. Scope and agreed decisions

Keep dense raw ECG samples, exactly 1,358 samples per 4,096-byte block, fixed
preallocated 4 MiB files, no trailer and no power-loss recovery contract. PPG keeps
its existing 16-byte record and file formats. Both products support FINITE and
continuous INFINITY through one bounded transport implementation.

The revised design has four principal decisions:

1. One serialized owner context owns session state. Producers and Bluetooth callbacks publish
   bounded records/events; they do not independently transition the session.
2. A 32 KiB rolling history updates continuously during acquisition. START copies
   it into the first 32 KiB of the existing 96 KiB future-buffer allocation; the
   remaining 64 KiB is the live queue. This partition never changes during a stream.
3. Stream STOP is immediate when processed: submit no further DATA, discard unsent
   backlog, and send END after already-submitted DATA. A partial tail is permitted.
4. START snapshots the history already incorporated by the owner when processed. It
   does not wait for an ECG acquisition boundary or completion of a partial block.

History starts zeroed at boot. Acquisition pause, stop or fault zeros it again;
stream START, STOP, disconnect and stream-only faults do not clear it while
recording remains healthy. START requires recording active or startup underway
and a usable subscribed link,
but never requires a filled history. Zero history is sent as unavailable data.

Recording and streaming remain separate. Normal recording stop completes the
current acquisition block; stream STOP does not stop acquisition or NAND writes.
A recording fault ends its stream. A stream-only failure leaves healthy recording
and rolling history active, except that producer-handoff loss clears history to
avoid retaining a gap (section 6). No automatic resume, retransmission or gap repair
is added. Only one stream is active, including cleanup.

Storage supplies initialized unused pages (`0xFF`). Individual files cannot be
deleted/reused; only full reformat reclaims space. RAM history zeroing is unrelated
to this storage guarantee and adds no per-file initialization writes.

## 2. Fixed representation

This specification assigns block/file format **v2** (`ECB2`, `ECF2`) and ECG stream
protocol **v0**. They distinguish the incompatible contract from `ECB1`/`ECF1`/
`ECT1` and ECG protocol v2. PPG uses the same v0 protocol while retaining its existing record/file format; see section 15.

All metadata integers are little-endian. The format version fixes all geometry,
sample rate and sample encoding; there are no negotiable block parameters.

| Property | Value |
| --- | ---: |
| Block size | 4,096 bytes |
| Samples per block | Exactly 1,358 |
| Bytes per sample | 3 |
| Sample payload | 4,074 bytes |
| Sample rate and RTC tick rate | 512 Hz |
| Block duration | 2.65234375 seconds |
| File size | 4,194,304 bytes |
| File header | One 4,096-byte page |
| Data capacity per file | 1,023 blocks |

Each sample preserves the complete MAX30001 24-bit FIFO word, MSB first:

```text
raw24 = (byte0 << 16) | (byte1 << 8) | byte2
etag  = (raw24 >> 3) & 7
ptag  = raw24 & 7
u18   = raw24 >> 6
ecg   = u18 - 0x40000 if u18 & 0x20000 else u18
```

Only ETAG 0..3 is encoded. Each occupies one sample time slot. ETAG 1 and 3
retain their timing but do not represent usable voltage samples. FIFO EOF tags
do not terminate blocks. An empty marker ends that FIFO drain without appending
a sample; it is not itself a fault. Reserved tags, overflow or lost acquisition
continuity end the recording session. There is no
new calibration or voltage-conversion rule.

## 3. Common block layout

| Offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 4 | Literal ASCII `ECB2` |
| 4 | 4 | `first_rtc_tick` |
| 8 | 4 | `first_sample_index` |
| 12 | 4 | `crc32` |
| 16 | 4,074 | Exactly 1,358 raw samples |
| 4,090 | 6 | Zero reserved bytes |

There is no sample count, sample-format selector, FINAL flag or discontinuity
flag. This moves the payload from offset 20 in ECB1 to offset 16 in ECB2; decoders
must select the layout by magic, not infer it from file date or size.

Use CRC-32/ISO-HDLC (reflected polynomial `0xEDB88320`, initial register and final
XOR `0xffffffff`, check value `0xcbf43926` for ASCII `123456789`). Calculate over
all 4,096 bytes with the CRC field treated as four zero bytes. Store the result
little-endian. The file header uses the same CRC rule at its own CRC offset.

### Sample-event timing and continuity

`first_rtc_tick` is the first sample's acquisition time in the continuously
advancing 32-bit extended 512-Hz counter. Tick zero is approximately boot time.
Recording pauses/restarts and file rotation do not reset this clock. It must
advance through acquisition pauses and applicable sleep states. Reboot starts a
new time origin; there is no UTC mapping. Counter wrap after about 97.1 days is
outside the supported scope; do not add wrap recovery to this revision.

**Preserve physical ECG/IMU sample-event alignment.** Before changing timing code,
trace the existing post-SYNCH SAMP/RTC anchor, FIFO sample ordering and IMU counter
mapping. Preserve the relationship between each ECG sample and its actual hardware
sample event, including required captured timing, phase/latency compensation and
sample-index relationships. Interrupt service, FIFO-read, block-finalization,
storage and transmission times must not replace sample-event time. An acquisition
restart may need to reestablish the sensor's physical anchor against the continuing
clock; removing stream START arming does not remove this acquisition requirement.
The IMU timing changes must retain alignment to the same continuing timebase.

`first_sample_index` remains the recording-local ordinal: zero at the first sample
of a new recording and incremented once per encoded sample. This revision changes
the timestamp's origin, not the sample-index field or block layout. For adjacent
blocks within one recording, require:

```text
next.first_sample_index == previous.first_sample_index + 1358
next.first_rtc_tick     == previous.first_rtc_tick     + 1358
```

Within a block, sample i has timestamp `first_rtc_tick + i` and ordinal
`first_sample_index + i`. Missing acquisition intervals remain gaps in boot-relative
time between recordings; do not synthesize samples to fill them. Files with a new
recording identity start a new continuity check. A stream contains at most one
acquisition session because acquisition discontinuity terminates it and clears
rolling history. Zero history establishes no continuity anchor; begin checks at
the first valid ECB2 block. A stream or isolated later file chunk need not begin
with sample index zero. Timestamp deduplication across streams must distinguish
device and boot context; reboot can repeat timestamps.

## 4. Recording lifecycle

1. Prepare a new file before accepting acquisition data. Start a new recording
   identity and acquire the physical timing anchor against the continuing boot clock.
2. Fill a private block. Only a completely filled, finalized, CRC-protected block
   may be published to storage or streaming.
3. Published bytes are immutable until all owners release them. Disk and stream
   use the same bytes for any block that both retain; stream receipt does not
   certify that its disk write succeeded. A finalized block may enter streaming
   before storage accepts it; neither ACK nor validated DATA proves persistence.
   Streaming and storage may retain different tails after failure. Preserve buffer
   lifetime without adding a storage-acceptance barrier to streaming.
4. A normal recording-stop request completes the current nonempty block, stops
   acquisition at that boundary, waits for retained writes, syncs and closes.
   If no block has begun, stop immediately; do not collect an extra block.
5. A recording fault stops acquisition and discards the incomplete block. Complete blocks
   already accepted by storage may finish writing when safe. Do not continue
   writes after an uncertain/failed file write, or append later blocks past it.
6. A subsequent acquisition start creates a new recording identity and physical
   anchor, without resetting the boot-relative counter.
   File rotation alone does not create a new recording session.

Acquisition pause, stop or fault advances an acquisition generation. The owner
zeros rolling history on a generation change. Reject stale producer records so
pre-pause records cannot refill it afterward. An active snapshot remains immutable
until safely retired.

Normal stop may require up to one block period of additional acquisition, plus
storage completion time. Samples arriving after the chosen final block boundary
may be discarded. Fault loss is acceptable; fabricated samples, reusing mutable
buffers prematurely and hiding a continuity gap are not.

There is no requirement that every finalized block reach every consumer after a
fault. Loss must remain at the end of the affected output session, not create a
hole followed by nominally valid continuation.

## 5. Files: fixed size, no trailer

```text
page 0         ECF2 header
pages 1..1023  consecutive ECB2 blocks, followed by invalid unused pages
```

Retain the existing recording filename convention and independent zero-based
chunk numbering. Never overwrite an existing recording file.

Header layout:

| Offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 4 | Literal ASCII `ECF2` |
| 4 | 4 | `chunk_index` |
| 8 | 8 | `recording_id`, unique within the device's retained recordings |
| 16 | 4 | `header_crc32` |
| 20 | 4,076 | Zero reserved bytes |

The magic fixes geometry, rate and encoding. The header is written once and is
never updated with counts or closing status. It contains no absolute-time mapping.

Before using a chunk, preallocate it to exactly 4 MiB. Unwritten data pages are
already initialized to `0xFF` by the storage contract. The recorder does not clear
them, verify the entire unused allocation, or implement stale-data protection.
No individual files are deleted or their allocations reused; full reformat is
the only reclamation operation and reestablishes the initialized-space guarantee.
Write and synchronize the valid header before acquisition uses the file. Zero
reserved bytes in encoded headers and blocks remain required; these are distinct
from the erased bytes in unused pages.

Write finalized blocks sequentially into data pages. Keep the existing periodic
sync policy of eight full blocks and sync at normal rotation/stop. The policy is
an implementation choice separate from block encoding; it makes no power-loss
promise here. After page 1023 is occupied, close the file and continue in a newly
prepared chunk with contiguous sample indices and ticks. Do not create an empty
next chunk merely because a stop coincides with the full-file boundary.

The former trailer page becomes a data page: 1,389,234 samples, or approximately
45.22246 minutes, fit in a full file. Actual-length files and truncation are not used.

### File decoding

1. Require file size 4 MiB and validate the ECF2 header and CRC.
2. Read data pages in order. A page whose four magic bytes are all `0xFF` marks the
   end of recorded data under the initialized-space guarantee; ignore later pages.
3. Otherwise require ECB2 magic, valid CRC, zero reserved bytes, valid sample tags
   and continuity. An invalid nonempty page is an error, not ordinary end-of-data.
4. Stop on the first error. Never search later pages to resume the recording.
5. Derive sample count as accepted block count times 1,358. Apply continuity across
   consecutive chunks of the same recording identity.

If all 1,023 data pages validate, reaching the fixed file boundary is normal;
no additional sentinel page is required. Validate zero header reserved bytes.
For a recording beginning at chunk zero, its first data block starts at sample
index zero. Chunk indices must be consecutive when decoding a complete recording;
a missing chunk is reported, not silently joined across. A nonfinal chunk must
be full: an early sentinel ends that recording's valid prefix. An isolated later
chunk may be decoded as a segment without claiming the earlier chunks are present.

Erased unused pages cannot be mistaken for ECB2 blocks. This is not a scheme
for detecting arbitrary corruption of unused space: corruption of an occupied
page into the erased sentinel may look like an earlier end. No trailer or clean-close
certificate remains to disambiguate such cases.

Decode only files whose writer has released them under the existing storage
ownership rules. This specification adds no concurrent host-file-reading feature.
Normal close and fault termination at a block boundary are indistinguishable in
the file. Valid-prefix samples may be returned alongside a reported decoding
error, but must not be labeled a complete, cleanly stopped recording.

There is no power-loss, reboot-interruption, FAT repair or durability-recovery
contract in this specification.

## 6. Streaming modes, history and memory

FINITE sends 32 KiB of history plus 96 KiB of future data, exactly 131,072 sensor
bytes. INFINITY sends the same history then future data until STOP, disconnect,
recording termination or fault. History bytes include any boot/reset zero padding.

| Fixed RAM region | Bytes | ECG records | PPG records |
| --- | ---: | ---: | ---: |
| Rolling history | 32,768 | 8 | 2,048 |
| Immutable session snapshot | 32,768 | 8 | 2,048 |
| Live future queue | 65,536 | 16 | 4,096 |

The snapshot occupies the beginning of the former 96 KiB future allocation. Total
payload RAM remains 128 KiB; bounded producer/event handoffs and existing transport
buffers have separate explicit budgets. Do not reclaim snapshot storage midstream.
FINITE's future count remains 24 ECG blocks or 6,144 PPG records: capture length
and physical queue capacity are separate constants.

Only one logical stream is active, including termination and cleanup. START while
busy returns BUSY. Keep connection/session identity and late-completion protection;
serialization does not permit reuse of memory still owned by transport. Stream IDs
are nonzero 32-bit identifiers, distinct from recording IDs. Use a fresh ID for
every START attempt on a connection, including rejected attempts. STOP targets the
active ID. Reconnection creates a new transport context; it does not resume data.

### Immediate START snapshot

Require recording active or startup underway, initialization, subscription and
ATT MTU at least 128. START does not initiate acquisition. ACK confirms stream
acceptance, not successful sensor startup. Zero history is allowed while startup
is pending; startup failure terminates the stream under section 8.
Do not check history fullness. When the owner processes START, copy the current
rolling history into the snapshot in oldest-to-newest order (at most two copies
if the ring wraps). Serialize this with record ingestion: records incorporated
before this boundary are history; pending ingress and later records are future.
There is no promise about the boundary's relation to command arrival/publication.
Do not drain to a command-publication frontier or wait for ingress to empty.

Do not wait for a partially acquired ECG block. That block becomes future data
when completed; some of its samples may predate START. There is no stream arming
state or special acquisition-boundary callback. The snapshot/future transition
must introduce no unintended gap or duplicate within a stream. Separate streams
may intentionally overlap. ACK precedes any DATA or END.

Copy outside spinlocks and interrupt-disabled regions. Producers publish through
bounded nonblocking handoffs while the owner copies. Measure copy time and size
handoff capacity for arrivals during it; do not promise zero latency or allocate
unbounded buffering. Recording never waits for BLE.

### Rolling history and acquisition resets

Boot initializes all history bytes to zero. Treat it as a full logical ring;
new complete records replace the oldest entries. A snapshot before real history
fills contains leading zero slots followed by complete records. There is no
history-ready state, count prerequisite or refill delay.

History updates continuously during recording, including snapshot transmission,
future streaming and session cleanup. START, stream STOP, disconnect, timeout
and live-queue overflow leave it intact. Acquisition pause, stop or fault zeros
it and resets its ring position. Order this reset with producer events and exclude
stale records from the previous acquisition epoch. Resume accepts new records
without waiting for the ring to fill. START while recording is stopped is still
NOT_RECORDING. Clearing history never changes an existing snapshot or resets the
ECG/IMU timebase.

### Live future queue

Producer-handoff overflow is distinct from live-queue overflow. If a required
record cannot enter the bounded producer handoff, terminate the affected stream
with BUFFER_OVERFLOW, zero rolling history and discard stale pre-loss ingress.
NAND recording continues; subsequent complete records refill history with no START
readiness delay. This approved fault exception avoids presenting discontinuous
mirrored history as consecutive samples. It does not reset the sample timebase.

Only complete immutable producer records enter the queue. Preserve FIFO order.
Release a queue slot only when its bytes have been copied into independently owned
transport storage or transport ownership otherwise ends. Snapshot bytes likewise
remain protected until cleanup; no send may reference rolling history directly.

A required new record arriving at a full queue terminates the stream with
BUFFER_OVERFLOW. Filling the final available slot alone is not overflow. Never
overwrite queued data and continue. Healthy recording/history continue. FINITE
stops enqueueing after its 96 KiB future quota; later records cannot overflow that
completed capture. INFINITY keeps recycling released slots. NAND rotation does not
end streaming.

The live queue holds about 42.4375 seconds of ECG or 16 seconds of PPG. This is
capacity, not guaranteed outage tolerance; history transmission, occupancy and
radio throughput reduce headroom. Sustained throughput must support acquisition
and drain initial history. Use one application TX notification slot shared by all
envelope types. Bounded HIL found essentially unchanged history throughput versus
three slots at MTU 498; see section 18 for evidence and limits.

## 7. Shared sensor stream protocol v0

Both ECG and PPG use this version-0 MS envelope. The ECG payload and lifecycle
are specified above; section 15 defines the PPG profile without changing PPG records.
The former PPG v1 transport path is removed. Commands:

| Command byte offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 2 | ASCII `MS` |
| 2 | 1 | Version 0 |
| 3 | 1 | Opcode |
| 4 | 4 | Stream ID |

| Opcode | Meaning |
| ---: | --- |
| `0x01` START | FINITE capture |
| `0x02` STOP | End the matching stream; replaces ECGv2 CANCEL semantics |
| `0x03` START_INFINITY | Continuous future capture |

STOP affects streaming only, not disk recording or sensor acquisition. A physical
normal collection-stop request is a different operation, governed by section 4.
The central permits only one unanswered control command: after START wait for
ACK or RESULT before another command; after STOP wait for END before another
START. A rejected STOP receives RESULT and leaves the active stream unchanged;
that response resolves the command but does not permit a second active stream.
If the user requests Stop before ACK, the central holds the intent and sends STOP
after ACK; discard that intent on START rejection or connection termination.
ATT write completion alone is not the application ACK/RESULT/END. Excess commands
must be explicitly rejected through ATT admission/resource errors rather than
accepted and silently dropped. Use writes with response for command admission;
write-without-response is not part of the revised control contract. Retain bounded
response reservations until sequential-command handling safely replaces them.
Wrong-session STOP is rejected. Invalid commands do not alter an active session.
Repeated STOP while a prior STOP awaits END is an excess command; reject it without
changing the session or sending a second END.
Once a stream has terminated, STOP for that ID returns WRONG_SESSION.

Retain the 12-byte notification envelope: ASCII MS (2 bytes), version 0 (1 byte), type
(1), stream ID (4), payload length (2), zero common flags (2). Complete length
must equal 12 plus payload length. Types remain ACK `0x81`, DATA `0x82`, END
`0x83`, RESULT `0x84`. Only the message types and payload sizes below are valid.

For an accepted START, send exactly one START_ACK before any DATA or END, including
when STOP or a fault occurs before any DATA. If the connection cannot deliver it,
terminate locally. A rejected START gets RESULT instead of ACK/DATA/END for that
attempt. The central validates fixed ACK values and reserved bytes and rejects
duplicate ACK, DATA before ACK, unknown message/status values, and DATA after END.
Malformed command packets that cannot identify a valid command/ID are discarded
without affecting an active session; they need not receive RESULT.

### START_ACK: 16-byte payload

| Offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 1 | Mode: 0 FINITE, 1 INFINITY |
| 1 | 1 | History size in 4,096-byte units: 8 |
| 2 | 6 | Zero reserved bytes |
| 8 | 8 | Planned total sensor bytes: 131,072 for FINITE; 0 for INFINITY |

INFINITY's zero total means unknown/unbounded, not an empty capture. This is
an unavoidable exception to declaring a finite total upfront. Mode must match
the client's START command. The selected product profile fixes its record format and
geometry; ACK does not identify the product. The central must know which product
it connected to. The lab central uses the advertised MSense4ECG/MSense4PPG name.
For ECG, each history unit is one ECB2 block; for PPG it is 256 existing records.

Do not repeat device name, Git identity or fixed sample geometry in each ACK.
These remain diagnostics obtainable separately; the transport decoder does not
depend on them. This specification does not require a new identity-discovery protocol.

### DATA: 8-byte offset followed by nonempty bytes

The payload begins with an unsigned 64-bit byte offset, followed by capture bytes.
Offset zero is the beginning of the first history block. There is no DATA sequence
number, inner fragment length, phase field or block-start/end flags.

The fragment length is payload length minus eight. A fragment may cross a block
boundary and the history/future boundary. It contains available, retained finalized
records or the explicitly permitted zero-filled history slots. Do not wait for a not-yet-finalized next
block solely to fill a notification; send a shorter fragment instead.

Maximum fragment bytes are negotiated ATT MTU minus 23 (ATT overhead 3, envelope
12, offset 8), additionally limited by the actual local NUS transmit capacity.
Retain the existing minimum ATT MTU of 128 to avoid expanding supported cases.
Packing across noncontiguous queue slots may require two copies into a TX buffer;
cross-boundary fragments are permitted, not required.

Receiver requirements:

- Require offset equal to the next expected byte offset, initially zero.
- Require nonempty bytes and a valid total message length.
- For FINITE, reject bytes beyond the declared total. For INFINITY, consume
  incrementally with bounded buffering; never allocate based on indefinite length.
- Use overflow-safe arithmetic. Offsets do not wrap; terminate before uint64 overflow.
- Validate and expose only complete 4,096-byte blocks, in order.

Offsets remain 64-bit independently of the 32-bit boot-relative sample clock.
Zero history consumes transport offsets just like real record bytes.

### END: 2-byte terminal status

END contains only a uint16 status; totals are derived from received data. No DATA
may follow END for that stream. Distinguish termination reason from validity of
the complete prefix already received:

| Code | Meaning |
| ---: | --- |
| `0x0000` SUCCESS | FINITE reached exactly its declared length |
| `0x0001` NOT_RECORDING | Acquisition stopped before stream completion |
| `0x0008` STOPPED | Explicit matching stream STOP |
| `0x0009` STORAGE_ERROR | Recording ended due to a storage failure |
| `0x000a` INTERNAL_ERROR | Sensor/acquisition or other internal failure |
| `0x000d` DISCONNECTED | Locally synthesized outcome; END delivery not expected |
| `0x000e` BUFFER_OVERFLOW | Future queue could not accept a complete block |

INFINITY never ends with SUCCESS merely because some amount of data arrived.
STOPPED is its normal user-requested ending. FINITE SUCCESS requires exact byte
count and successful validation of all non-padding blocks, not just receipt of END.
Permitted history padding counts toward transport length but never valid samples.

Command rejection uses RESULT with a 2-byte uint16 status. RESULT introduces no
DATA sequence and does not reset the byte offset of an active stream.
The rejection-code assignments are (the reserved entry is not a valid new response):

| Code | Meaning |
| ---: | --- |
| `0x0001` | NOT_RECORDING |
| `0x0002` | Reserved former HISTORY_NOT_READY; never emitted by this design |
| `0x0003` | NOT_SUBSCRIBED |
| `0x0004` | BUSY |
| `0x0005` | MTU_TOO_SMALL |
| `0x0006` | INVALID_COMMAND |
| `0x0007` | UNSUPPORTED_VERSION |
| `0x000b` | NOT_INITIALIZED |
| `0x000c` | WRONG_SESSION |

RESULT refers to the command's stream
ID and does not terminate a different active session.

## 8. Termination, validation and loss policy

### Immediate stream termination

Serialize STOP with DATA submission in the owner. After STOP is processed, submit
no further DATA, discard unsent session backlog, and send END after all
already-submitted DATA in transport order. Prepared but unsubmitted fragments are
discarded. Already-submitted notifications may still arrive; their buffers remain
owned until safely retired. There is no rounding to a record boundary or draining
of a begun record. The status is STOPPED if STOP wins the terminal decision.
STOP before any DATA may produce ACK then END with zero DATA.

Normal recording stop still completes its acquisition block as in section 4.
When the owner processes recording termination, end the stream immediately with
NOT_RECORDING if no terminal decision already exists, without flushing queued
records. The streamed tail need not equal
the stored tail. Zero rolling history for that acquisition discontinuity.

The first terminal decision processed by the owner wins, whether normal or fault.
FINITE SUCCESS is eligible only after the full declared payload is submitted.
A later fault does not override STOPPED or any other chosen status, even before
END submission. It still triggers required acquisition/history cleanup. Exact
ordering and fault-cause preservation across coalesced acquisition updates are
not promised; detailed causes may remain in diagnostics. An observed generation
change ends an active stream with NOT_RECORDING unless the observed update carries
a known fault status, subject to the first-decision rule. Send no second END.
Failed END delivery/disconnect is a local abnormal outcome regardless of the
chosen wire status; never claim delivery when it failed.
Callbacks report events rather than independently changing these decisions.

Overflow/fault stops further DATA submission; partial tails are permitted. On
disconnect, retire ownership safely and end locally. Do not wait indefinitely for
callbacks that may never arrive or permit late callbacks to affect a new session.
A new START waits for cleanup, not history refill.

### ECG history padding and validated output

Reassemble every logical 4,096-byte slot across arbitrary fragments. A slot is
unavailable history only if **all 4,096 bytes are zero** and its entire byte range
is inside offsets [0, 32768). Skip these slots without exposing samples or setting
continuity state. They are leading padding: a zero slot after the first valid
block is a protocol error. Never accept zero padding in future data or ECG files.
Any other malformed block, including a partly zeroed block, is a validation error.

The first valid block establishes sample-index/timestamp continuity. Subsequent
valid blocks must satisfy section 3. Track received transport bytes separately
from valid sample/block counts and skipped history slots. Do not report
`valid_blocks * 4096` as a transport offset when padding was skipped.

A complete zero history snapshot followed by valid future blocks is a normal
stream. It can finish FINITE successfully with fewer than 32 valid ECG blocks.
Zero padding never represents fabricated ECG samples. FINITE SUCCESS still requires
exactly 131,072 received bytes, complete slot framing, and successful validation
of every non-padding block. INFINITY has no promised total or SUCCESS endpoint.

### Retention and partial tails

The receiver preserves already validated output regardless of terminal status.
STOPPED and NOT_RECORDING permit arbitrary partial trailing bytes, just like a
fault/disconnect. Discard that tail without treating a normal STOP as corruption.
Finish validation of complete queued blocks on END/disconnect before finalizing
counts, without letting their work affect a later stream. Never retract valid
output solely because the stream ended. PPG retains complete 16-byte records;
it has no ECB integrity check (section 15).

Malformed framing, missing/duplicate offsets, CRC or continuity failures end
reception at the first failure. There is no resynchronization scan, silent loss
or gap repair. Report termination separately from preserved data. A finite stream
ending early is stopped/failed, never SUCCESS; an INFINITY stream reports running,
stopped or failed. Neither mode requires accumulating all output in RAM.

### No-progress timeout

Retain a configurable initial default of 15 seconds, with no whole-session limit
for INFINITY. The sender measures time without notification completion while ACK,
DATA, END or a command RESULT is ready or in flight, including RESULT delivery
outside an active stream. Failed submissions do not reset it. Waiting
solely for the next producer record is not stalled transmission. The central starts
its timer at START and resets it on accepted ACK and contiguous valid DATA; zero
history DATA is progress too. Unrelated RESULT, malformed or duplicate traffic is
not progress. Stop the network timer on END/rejection; complete pending validation.

Expiry terminates locally and disconnects to retire the session. TIMEOUT is local,
not a new wire status. A central detecting malformed data or bounded receive-resource
exhaustion also terminates/disconnects and preserves validated output. END delivery
on fault is best effort. Stream-only timeout does not stop healthy recording or
clear rolling history. Validate this default on supported hardware links later.

## 9. Implementation ownership and timing boundaries

One serialized owner context owns session transitions, cursors, mode, terminal
status and snapshot/live-queue decisions; a dedicated stream thread is not required.
Use a bounded system-workqueue handler to own streaming and submit notifications
directly through one shared TX slot. Bound work per invocation
and reschedule remaining work so other system work and TX callbacks can run. Keep
blocking disconnect/HCI operations on a separate non-system-work execution path.
Measure snapshot-copy and scheduling latency; do not move an unbounded loop onto
the system workqueue. Enforce `CONFIG_BT_CONN_TX_NOTIFY_WQ=n`, matching the managed
SDK's normal completion callback context. Keep callback tokens, connection lifetimes and stale-event
protection even when the separate TX submission handoff is removed.

Exactly one context per peripheral publishes sensor records, in acquisition order.
Lifecycle/fault and Bluetooth notifications may originate elsewhere. Use a bounded
FIFO handoff; no multi-producer record sorting or command frontier is required.
Commands take effect against the owner's current state when processed.

Acquisition updates may coalesce to the latest state plus an acquisition generation.
Every acquisition discontinuity advances that generation, even if stop/fault/start
occurs before the owner runs. Before accepting commands or ingesting new-generation
records, a changed generation terminates any old stream, clears rolling history
and rejects stale ingress. Latest state may already say startup/recording active;
that does not erase the discontinuity. Do not require replay of every transition
or preservation of the first fault. Generation/state publication must remain
reliable when record or command queues are full.

Do not hide producer-memory lifetime behind a pointer queue. Transfer ownership
or copy into bounded storage before the producer can reuse it. Reserve a reliable
way to observe acquisition-generation changes even when the record queue is full.
Overflow ends the affected stream safely; it must not silently splice data.
Callbacks outside the owner context publish handoffs. Completion callbacks in that
serialized context may update TX bookkeeping without a second concurrency layer;
terminal decisions remain with the owner. Keep one immutable TX buffer occupied
until its notification completion is retired; successful submission alone does not
release it. ACK, DATA, END and RESULT share this slot, so no next envelope is
submitted while it is occupied. Remove multi-slot allocation/scanning, per-slot
submission work and multi-completion coordination. Keep a minimal completion
handoff only where callback context requires one, and retain connection/session
identity and safe disconnect retirement. A late callback must not release a reused
slot. The managed SDK copies notification bytes into stack-owned storage during
submission and may suppress completion on disconnect. Owner cleanup therefore
invalidates the submission token and retires the application slot without waiting
for a callback. Do not reduce Bluetooth stack/controller buffer counts as part of this change.

Command admission and TX-buffer ownership have different retirement points.
Successful ACK/END/RESULT submission releases the command mailbox reservation;
DATA never releases it. The next command can be admitted while the slot is busy,
but the owner processes it only after TX retirement, with completed END cleanup
first. The central still waits for the application response before writing again.
Completion updates the single slot directly in the enforced system-workqueue
context; no separate TX-completion handoff is used. Delayed retries/timeouts must
not replace an immediate wake already requested by a producer or command.

No BLE wait may block acquisition or storage. No snapshot copy or large record
copy belongs in an interrupt-disabled session critical section. Handoff memory
and scheduling latency must be budgeted explicitly. Do not replace spinlocks with
mutexes while leaving ownership unchanged and call that the reduction.

Preserve storage ownership and existing USB MSC rules. No unrelated storage
framework rewrite is required. Preserve physical ECG/IMU event timing as specified
in section 3; stream boundary simplification does not authorize removing the
sensor acquisition anchor or using callback execution time as sample time.

The central validates complete ECB2 blocks before exposing samples. Its scheduling,
UI and storage mechanisms are platform choices. `central_nus_test` is only the
lab implementation. No retransmission, multi-client capture, adaptive memory
partitioning, stream pause/resume or automatic gap recovery is added.

## 10. Verification requirements for the revised design

The verification scope below applies to implementation changes. Test
the new execution, admission and coalescence rules as well as the retained format.

- Codec/file layout, all 1,358 samples, CRC, tags, reserved bytes, 4 MiB sizing,
  erased-file sentinel, chunk continuity/rotation and no trailer remain covered.
- Verify the boot-relative ECG counter is not reset by recording stop/restart;
  trace actual SAMP event alignment, FIFO ordering and IMU timing, including pauses
  and applicable sleep states. No 97-day wrap or power-loss recovery tests required.
- Test START with entirely zero, partly filled, full and physically wrapped history.
  Verify chronological snapshot, padding bytes and the owner-processing boundary
  with pending ingress. Test startup acceptance followed by both success and failure.
- Zero history on acquisition pause/stop/fault, excluding stale producer events.
  Keep it on stream STOP/disconnect/timeout/live-queue overflow; producer-handoff
  overflow instead clears history and excludes stale ingress. Preserve active
  snapshot ownership and verify NAND recording continues for both overflow types.
- Exercise FINITE's 96 KiB future quota using a 64 KiB recyclable queue, and INFINITY
  through repeated turnovers. No unbounded allocation or snapshot reclamation.
- STOP before DATA, during history/future and with an outstanding cross-boundary send:
  no new DATA submission after STOP processing, no partial-record drain, ordered END.
- Test STOP/SUCCESS/fault ordering, failed END submission, repeated commands,
  cleanup before restart, and stale completions/disconnects.
  The first terminal decision must remain fixed. Coalesced stop/fault/start must
  still terminate the old stream, clear history and exclude stale records.
- Test sequential commands, queued user Stop before ACK, excess-command rejection,
  rejection-capacity recovery and successful START after END without reconnect.
- Measure bounded owner work and snapshot latency while other system work and
  single-slot TX completions run; verify blocking disconnect uses a separate path.
  Verify immutable buffer lifetime, completion-before-reuse, failed submission,
  ordered ACK/DATA/END/RESULT and stale callbacks after disconnect/restart.
- Central skips only leading all-zero history slots. Test padding split across DATA,
  all-zero history then valid live data, zero future blocks, other invalid blocks,
  and transport counts distinct from valid-sample counts. Keep PPG framing unchanged.
- Validate incomplete tails on normal STOP and faults; preserve complete queued
  validation at END/disconnect. Test offsets near 2^32 and uint64 overflow bounds.
- Test producer/event/receiver queue exhaustion and no-progress timeout without
  losing the terminal event or extending the timer on unrelated traffic.
- Run focused host tests and serial firmware builds for ECGv0, PPGv2 and
  `central_nus_test` on nRF5340DK and nRF54L15DK when implementing. Report logs,
  artifacts and final exit codes when authorized. This document edit does not
  authorize implementation, builds, HIL or commits.

Later HIL must measure snapshot copy/handoff latency, ECG/IMU physical timing,
sustained live throughput, initial-history drain, real backpressure and cleanup
races. Host tests and earlier builds do not establish these properties.

## 11. User-visible tradeoffs

| Topic | Previous implementation | Revised target |
| --- | --- | --- |
| History at early START | Wait/reject until full | Send zero prefix immediately |
| Stream START | ECG acquisition-boundary arming | Immediate complete-history snapshot |
| Repeat stream | History refresh/preservation conditions | Latest rolling history; overlap accepted |
| Acquisition pause | Session-relative timing behavior | Zero history; boot clock continues |
| Stream STOP | Finish begun record | Immediate; discard partial receiver tail |
| Live queue capacity | 96 KiB | 64 KiB; snapshot uses other 32 KiB |
| FINITE payload | 128 KiB | Unchanged, including unavailable history bytes |
| INFINITY | Continuous bounded queue | Preserved |
| Session state | Multiple contexts mutate it | One owner processes events |
| Commands | Bounded pipelining | One unanswered command; early UI Stop waits for ACK |
| Terminal races | Fault may override a normal result | First owner decision wins; reason is best-effort |
| Startup ACK | Spec required active recording | Startup underway is sufficient |
| Stream/file relation | Spec required storage acceptance | Finalized streamed blocks need not be persisted |

Normal recording stop still takes up to one ECG block period plus write completion.
Stream STOP leaves recording active and may omit samples present in the file.
History padding means transport byte counts are not usable-sample counts. Repeated
captures can overlap; deduplication must account for device and boot origin.
Smaller live capacity reduces burst tolerance. There is no durable-delivery promise,
reconnect resume, UTC mapping or power-loss recovery. Receiver and sender must be
updated together for the revised semantics despite unchanged lab wire identifiers.

## 12. Implementation constraints and readiness

The additional reductions are agreed: serialized owner execution, sequential
commands, one record producer with owner-processing START, and first-decision
termination with coalesced acquisition updates. Retain persistent rolling history,
the fixed snapshot/live partition, immediate STOP and immediate START. History is
cleared on acquisition discontinuity, not stream termination. Never gate START on
history fullness. Keep actual ECG/IMU sample-event timing aligned to the continuing
boot clock. Preserve PPG record/files and ECB2/ECF2 layouts.

Keep one FINITE/INFINITY transport path. Separate finite capture quota from queue
capacity. Use one shared application TX notification slot, as supported by the
bounded throughput POC in section 18. Avoid reference counting for history snapshots,
midstream repartitioning and general event frameworks unless a demonstrated need
requires reconsidering the design. Stop for clarification if existing hardware
timing or ownership makes the agreed design materially infeasible.

Section 19 distinguishes completed work from remaining verification. Do not label
HIL successful until the respective work is actually completed. Report future
line-count savings against a named baseline and Git HEAD separately.

## 13. Decoder interoperability vectors

These are deterministic byte-layout vectors, not claims of revised implementation
verification. Begin with a zero-filled 4,096-byte page before encoding fields.

- Block: magic `ECB2`, first RTC tick `100000`, first sample index `0`.
  For each i = 0..1357 encode
  `raw24 = ((i & 0x3ffff) << 6) | ((i % 4) << 3) | (i % 8)`
  as three MSB-first bytes at `16 + 3*i`. Leave trailing reserved bytes zero.
  CRC at byte 12 is **`0xb27dde56`**, stored little-endian.
  The next contiguous block starts at tick `101358`, sample index `1358`.
- File header: magic `ECF2`, chunk index `7`, recording ID
  `0x0123456789abcdef`, zero reserved bytes. CRC at byte 16 is
  **`0x3fdb88b5`**, stored `b5 88 db 3f`.
- A completely zero-filled 4,096-byte slot has no ECB2 magic or valid encoded CRC.
  It is skipped only as leading stream history padding, never as a stored block
  or future block. It advances transport offset by 4096 and sample count by zero.

Sample i uses timestamp `first_rtc_tick + i` and ordinal `first_sample_index + i`.
Decode signed ECG and ETAG/PTAG as in section 2. ETAG 1/3 preserve timing but are
unusable for voltage interpretation. Neither UTC nor calibration is inferred.

## 14. Historical initial implementation verification

Before this revision, the ECB2/ECF2 codec, recorder, Python file decoder and initial
ECG stream implementation, before protocol renumbering, passed host checks
and firmware builds on September 7,
2026. Those tests included the former boundary-STOP, arming and counter-wrap rules.
They must not be treated as proof of this revised design. See section 16 for the
later shared-transport baseline and retained build links. No HIL was performed.

## 15. PPG profile

PPG shares the v0 command/ACK/DATA/END/RESULT envelope, owner lifecycle, snapshot,
zero-initialized rolling history, acquisition-reset policy, immediate START/STOP
and timeout. Its [packed 16-byte records](PPG_PACKED_16_BYTE_FORMAT.md), acquisition
encoding and files remain unchanged; no ECB header or CRC is added.

| Property | PPG | ECG |
| --- | ---: | ---: |
| Producer record | 16 bytes | 4,096 bytes |
| History/snapshot records each | 2,048 | 8 |
| History bytes / ACK units | 32,768 / 8 | 32,768 / 8 |
| Live queue records | 4,096 | 16 |
| Live queue bytes | 65,536 | 65,536 |
| FINITE future records | 6,144 | 24 |
| FINITE total bytes | 131,072 | 131,072 |
| Record/sample rate | 256 records/s | 512 samples/s; 1,358/block |
| Full real-history duration | 8 seconds | 21.21875 seconds |
| Live queue duration | 16 seconds | 42.4375 seconds |
| FINITE future duration | 24 seconds | 63.65625 seconds |

DATA may split/combine PPG records. The receiver keeps complete 16-byte records
and discards partial tails on STOP/NOT_RECORDING/fault. Only FINITE SUCCESS requires
the exact declared length and complete framing. PPG zero history has no new
reliable invalidity marker: framing alone cannot distinguish a zero record from
legitimate content. Do not claim CRC validation or automatically delete every
zero-valued PPG record as if the ECG padding rule applied.

The lab central may report `PPG_PREFIX complete_records/complete_bytes` and ECG
validated block/sample counts separately from transport bytes and skipped history.
Raw diagnostic output may include partial or unvalidated bytes. Neither receiver
buffers a whole INFINITY stream in RAM. PPG's existing 512-Hz tick interpretation
is unchanged; the ECG/IMU alignment work must not silently change PPG file encoding.

## 16. Historical shared transport reduction verification

The previous follow-up, before the current design revision, removed duplicate PPG/ECG sender and receiver paths, legacy PPG
metadata, redundant counters and completion transitions. It retained bounded
buffers, asynchronous ownership checks and ECG validation. PPG acquisition and
file encoding are unchanged; existing PPG centrals must adopt the v0 envelope.

| Production source | Before this reduction pass | After | Change |
| --- | ---: | ---: | ---: |
| `shared/sensor_stream.c` | 2,289 | 1,799 | -490 |
| `central_nus_test/src/main.c` | 3,549 | 2,952 | -597 |
| Combined | 5,838 | 4,751 | -1,087 (18.6%) |

These are physical source lines, excluding tests and documentation. The same
files totaled 46 more lines than the original pre-proposal implementation in Git
HEAD at that review, now supporting both products' INFINITY modes. The 1,087-line reduction is
relative to the first ECG proposal implementation, not that original baseline.

The final sender host suite passed all three tests, and the central receiver
production-function harness passed. Coverage includes PPG fragments crossing
records and ring wrap, history retention, terminal submission races, shared ACK
and END handling, complete-prefix retention, deferred ECG validation, stale
disconnect/timeout events and offsets beyond 32 bits. The geometry CTest passed
(1/1). These deterministic host checks do not establish real BLE or scheduler
timing. Section 14 records the earlier unchanged codec/decoder/recorder checks.

Final wrapper builds used the managed NCS workspace
`C:\ncs\SenSEv2.9.3` with toolchain `C:\ncs\toolchains\b620d30767`,
ran serially, and all exited zero:

| Application / board | Build log | Flash artifact | Application binary |
| --- | --- | --- | --- |
| ECGv0 / `ecgv0/nrf5340/cpuapp` | [log](build-logs/ncs-build-20260907-152358-440.log) | [merged.hex](ECGv0/build/merged.hex) | [zephyr.bin](ECGv0/build/ECGv0/zephyr/zephyr.bin) |
| PPGv2 / `ppgv2/nrf5340/cpuapp` | [log](build-logs/ncs-build-20260907-152538-898.log) | [merged.hex](PPGv2/build/merged.hex) | [zephyr.bin](PPGv2/build/PPGv2/zephyr/zephyr.bin) |
| Central / `nrf5340dk/nrf5340/cpuapp` | [log](build-logs/ncs-build-20260907-152933-852.log) | [merged.hex](central_nus_test/build_nrf5340/merged.hex) | [zephyr.bin](central_nus_test/build_nrf5340/central_nus_test/zephyr/zephyr.bin) |
| Central / `nrf54l15dk/nrf54l15/cpuapp` | [log](build-logs/ncs-build-20260907-153119-133.log) | [merged.hex](central_nus_test/build_nrf54l15/merged.hex) | [zephyr.bin](central_nus_test/build_nrf54l15/central_nus_test/zephyr/zephyr.bin) |

Existing unrelated peripheral/SDK warnings remain. No HIL, flashing, serial
sessions or commits were performed. All launched build processes completed;
no debugger was started. Real backpressure, sustained INFINITY and hardware
fault timing remain to be tested using section 10.

These logs record the previous design. Artifact paths are reused by later builds;
use current build logs and hashes to establish provenance. Historical line counts
are not new reduction claims.


## 17. Historical ownership/history revision verification

These results predate the synchronization simplification now specified above.
They do not validate the new owner context, sequential commands or terminal policy.

The implementation uses one stream owner, a 32 KiB rolling history, a 32 KiB
immutable snapshot and a 64 KiB live queue. FINITE still sends 96 KiB of future
data. ECG ingress reserves four 4,096-byte slots; PPG reserves thirty-two 16-byte
slots. The approved ingress-loss policy terminates the stream, clears history
and rejects stale ingress while NAND recording continues.

Compared with the pre-pass source at `c49ec1e`, production C/header files decrease
by **52 physical lines net**, including every changed production C/header file.
`shared/sensor_stream.c` changes from 1,799 to 1,744 (-55), the test central from
2,952 to 2,982 (+30), and remaining production C/header changes total -27. Tests,
documentation and Kconfig/application settings are excluded from this figure.
The settings add 18 lines, making the net reduction **34 lines** when included.
The reduction is modest: bounded handoffs and asynchronous BLE submission and
completion still require explicit ownership and stale-event checks.

The parent independently reran the updated sender host suite (3/3) and central
receiver harness (1/1), both passing. A separate production-function check passed
PPG live-ring wrap and snapshot-to-live fragment copying. These checks cover
zero-history handling, retained history, immediate STOP partial tails, stale
handoffs/TX reports, retry/no-progress handling and bounded byte offsets; they do
not establish real scheduler or BLE timing.

ECG timing starts the existing extended RTC0 counter at boot and retains it
through acquisition pauses. The post-SYNCH hardware SAMP anchor, FIFO sample
ordinal mapping and IMU FSYNC mapping are unchanged. The 24-bit RTC overflow
extension remains active. No new IMU estimator or timestamp format was added.

All four final wrapper builds completed serially with exit code 0, using the
managed NCS 2.9.3 workspace and documented toolchain:

| Application / board | Current build log | Flash artifact |
| --- | --- | --- |
| ECGv0 / `ecgv0/nrf5340/cpuapp` | [log](build-logs/ncs-build-20260907-184443-649.log) | [merged.hex](ECGv0/build/merged.hex) |
| PPGv2 / `ppgv2/nrf5340/cpuapp` | [log](build-logs/ncs-build-20260907-184246-557.log) | [merged.hex](PPGv2/build/merged.hex) |
| Central / `nrf5340dk/nrf5340/cpuapp` | [log](build-logs/ncs-build-20260907-184733-389.log) | [merged.hex](central_nus_test/build_nrf5340/merged.hex) |
| Central / `nrf54l15dk/nrf54l15/cpuapp` | [log](build-logs/ncs-build-20260907-184948-467.log) | [merged.hex](central_nus_test/build_nrf54l15/merged.hex) |

The initial PPG configure failed because the new ingress Kconfig settings had no
prompts; the declarations were corrected and PPG plus ECG were rebuilt. Existing
peripheral/SDK warnings remain. The ECG application uses 423,704 of 458,240 RAM
bytes, an increase of 16,816 bytes versus the preceding build, leaving 34,536
bytes. Its FLASH use is 262,212 bytes. The producer handoff makes the ownership
simpler at a measurable RAM cost.

The implementer also passed the recorder host harness, decoder tests (7/7), and
geometry CTest on all four application/board builds. The parent reviewed the
source and build results and accepted the software: **grade B, PASS**, with no
remaining blocking findings. The grade reflects the modest source reduction and
extra handoff RAM, rather than a claim that all concurrency complexity disappeared.

The Sol-Light agent completed three bounded ECG HIL cases with the accepted ECG
and nRF5340 central images:

- FINITE 7101: 131,072 sensor bytes, 285 DATA notifications, all 32 ECB2 blocks
  validated, including 24 live blocks. STREAM_OK, no relay drops or protocol errors.
- INFINITY 7201/7202: each delivered nine validated blocks, ended STOPPED and
  restarted without a history readiness wait or corruption.
- Acquisition pause/restart: START 7300 while stopped returned NOT_RECORDING;
  START 7301 immediately after recording resumed delivered eight zero history
  slots followed by two valid live blocks. The first new block had tick 231445,
  index 0; the prior captured block had tick 174903, index 99134. This supports
  continued boot-relative time with a reset recording-local index.

STOP captures happened at complete-block boundaries; actual partial-tail delivery
on hardware was not demonstrated. Host checks cover that receiver path. Physical
ECG/IMU phase accuracy, long-duration backpressure/overflow and PPG hardware remain
untested in this bounded run. The parent reviewed and accepted the bounded HIL
result: **PASS**. The [HIL report](build-owner-hil/REPORT.md) records exact image
hashes, raw-capture evidence and limits. That evidence directory is local/ignored.

Cleanup was verified: central reset/idle with no relay drops; ECG recording stopped
normally, then RESET_SYSTEM returned it to ship-mode wait. Temporary GPIO state was
restored, serial contexts closed, and no task-owned debugger/capture tools remained.
No commits were made or authorized.

## 18. Single-slot feasibility HIL (September 7, 2026)

The approved target now uses one application TX notification slot for both ECG
and PPG. A bounded ECG POC on source `8f386cc` changed only the application slot
count from three to one, preserving Bluetooth stack TX limits. It used the existing
sender architecture, not the then-proposed serialized-owner redesign.

Each history trial transmitted 32 KiB of real ECG blocks. Three trials per
configuration produced these sensor-byte rates on the nRF5340 ECG/DK link:

| Application slots | ATT MTU | History drain | Sensor throughput |
| ---: | ---: | --- | --- |
| 3 | 498 | 901 ms | 35.5 KiB/s |
| 1 | 498 | 902 ms | 35.4 KiB/s |
| 1 | 247 | 1,019–1,083 ms | 29.5–31.4 KiB/s |
| 1 | 128 | 1,382–1,387 ms | 23.0–23.1 KiB/s |

The settled link used a 60 ms interval, latency 3, 2M PHY and 251-octet DLE.
A 125-second one-slot INFINITY run validated 55 blocks and passed STOP/restart;
the restarted stream retained nine complete blocks and discarded a 950-byte tail.
CRC, offsets and counter continuity passed, with no reported relay drops or
overflow. One application slot does not mean one notification per interval.

These measurements support the single-slot decision, including margin above ECG
production (~1.51 KiB/s) and PPG production (4 KiB/s). They do not validate PPG
hardware, smartphones, RF interference, deliberate backpressure or the forthcoming
owner-context implementation. Recheck history throughput and sustained streaming
after that scheduling change. No permanent firmware changes resulted from the POC.
The [report and raw evidence](build-single-slot-hil/REPORT.md) are local/ignored;
the summary above preserves the decision evidence in this specification.

## 19. Serialized-owner and single-slot implementation verification

Implemented September 7, 2026 against `8f386cc` plus the approved document changes,
without commits. The sender was replaced with bounded system-workqueue execution,
one notification slot, one command mailbox and a generation-tagged producer FIFO.
The old TX lock, slot allocator, submission/report handoff, command frontier and
event/result queues were removed. Blocking disconnect uses a separate worker.
Central command handling now distinguishes ATT completion from the application
response and defers early Stop across either ACK/write-completion ordering.

The parent reviewed the replacement and accepted the software: **A−, PASS**.
Production C/H decreased by 473 physical lines against the starting revision:
sender 1,788 to 1,246 (−542), central +69, public header unchanged in line count.
Configuration decreased by 14 lines. Test and documentation changes are excluded
from these production totals. Remaining publication/disconnect locks protect
cross-context handoffs; single-slot bookkeeping has no separate TX lock.

Focused sender tests passed 5/5 and the central receiver harness passed, including
command admission, DATA not releasing a command reservation, stale TX tokens,
first terminal decision, history geometry and deferred Stop ordering. The parent
independently reran both suites. Final serial wrapper builds all exited 0:

| Target | Log in `D:/senselab-tools/logs/` |
| --- | --- |
| ECG / `ecgv0/nrf5340/cpuapp` | `ncs-build-20260907-220426-174.log` |
| PPG / `ppgv2/nrf5340/cpuapp` | `ncs-build-20260907-220553-608.log` |
| Central / `nrf5340dk/nrf5340/cpuapp` | `ncs-build-20260907-220725-699.log` |
| Central / `nrf54l15dk/nrf54l15/cpuapp` | `ncs-build-20260907-220827-372.log` |

Artifacts are the normal application build directories' `merged.hex` and, for
nRF5340, `merged_CPUNET.hex`; central builds use `build_nrf5340` and
`build_nrf54l15`. ECG application RAM is 421,072 / 458,240 bytes. These software
checks alone do not establish hardware throughput, scheduling latency or physical
ECG/IMU alignment.

The subsequent bounded revised-owner ECG/DK HIL passed four combined scenarios:

- Connection smoke at MTU 498, 60 ms interval, 2M PHY and 251-octet DLE.
- FINITE: exactly 131,072 bytes and 32 validated blocks; real history drained in
  902 ms at 35.4 KiB/s, matching the earlier single-slot reference.
- Early STOP (~52 ms after START): deferred until ACK, one STOPPED END and a
  discarded 1,425-byte tail. Immediate restart streamed 21 valid blocks over ~33 s.
- Disconnect/reconnect retained the valid prefix and allowed another stream.
  Acquisition stop returned NOT_RECORDING; immediate restart sent eight zero
  history slots followed by a valid live block with continuing boot-relative time.

All complete real blocks passed CRC, offset and counter-continuity checks, with
no observed relay drops or overflow. The parent reviewed the evidence and accepted
the bounded HIL: **PASS**. The [HIL report](build-single-owner-hil/REPORT.md) records
image hashes, raw logs/captures and validation; this evidence directory is ignored.
Both devices retain the new firmware; ECG is in ship mode and central idle.
Temporary register state was restored, serial ports closed and test tools exited.

This run did not inject FIFO overflow, storage failures or BLE backpressure, and
did not measure snapshot latency or physical ECG/IMU alignment. PPG hardware was
not tested in that ECG run; see section 20 for subsequent PPG testing. Smartphones
and RF-range behavior remain untested. No commits were made.

## 20. Historical bounded PPG HIL and acquisition control

The subsequent PPG/DK run passed FINITE (131,072 bytes / 8,192 records), early
deferred STOP with a one-byte partial tail, approximately 35 seconds of INFINITY
with restart, disconnect/reconnect, and acquisition off/on. History measured
26.5–31.2 KiB/s at MTU 498, 60 ms interval, 2M PHY and 251-octet DLE. No offset
gaps, relay drops or stream overflow were observed. Field-range and wrap-aware
tick-order checks passed; PPG has no CRC, so this is not ECG-style integrity proof.

Source was `72375f4` plus the test Central's collection-control addition, using
the then-current transport version 3. The Central needed `collect on|off` support
because stream START never initiates acquisition. PPG uses control service
`da39c930-1d81-48e2-9c68-d0ae4bbd351f`, enable characteristic
`da39c931-1d81-48e2-9c68-d0ae4bbd351f`, and a one-byte write with response:
1 requests acquisition, 0 requests normal stop. Discover the handle; ATT success
confirms request acceptance, not sensor readiness. These product controls are
separate from the shared streaming protocol.

Evidence was saved in `build-ppg-owner-hil/` (ignored/local and no longer present
at this refresh). Practical setup and cleanup are recorded in
[HIL agent notes](docs/nrf5340-ecg-hil-agent-notes.md). The run left PPG collection
off, Central idle and tools closed. Optical accuracy, physical timing, injected
faults and smartphone/RF-range behavior were not tested.
