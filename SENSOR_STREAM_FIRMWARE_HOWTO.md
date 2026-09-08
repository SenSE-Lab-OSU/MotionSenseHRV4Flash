# Shared NUS v0 streaming: firmware design handoff

Status: **implemented; software review and bounded ECG and PPG HIL accepted**.
Updated September 8, 2026. This replaces the historical v1 handoff.
Targets: ECGv0 and PPGv2, managed NCS 2.9.3 on nRF5340.

[ECG_BLOCK_FORMAT.md](ECG_BLOCK_FORMAT.md) is the normative format/lifecycle
specification; [the central guide](SENSOR_STREAM_CENTRAL_HOWTO.md) describes
receiver integration. Earlier software/build/HIL results in canonical section 17
are historical; canonical section 19 records verification of the current implementation.
The current transport version byte is **0** for commands and notifications on
both products, independent of their binary record formats. Historical HIL used
version 3; see canonical section 20 for PPG evidence and version-scope limits.

## 1. Scope

The implementation combines the four agreed reductions:

1. One serialized owner context, using bounded system-workqueue execution instead
   of requiring a dedicated stream thread and separate notification-submission work.
2. One unanswered control command; explicit admission rejection for excess requests.
3. One record producer per peripheral and START against current owner history,
   without command-publication frontiers or multi-producer record sorting.
4. First terminal decision wins; acquisition updates may coalesce to latest state
   plus a generation that cannot hide a discontinuity.

Keep shared FINITE/INFINITY v0 transport, PPG 16-byte record/files, ECB2/ECF2 layouts,
full ECG blocks, fixed preallocated 4 MiB ECG files, and independent NAND recording.
Use exactly one shared application TX notification slot for ACK, DATA, END and
RESULT. No retransmission, reconnect resume, dynamic capture allocation or unrelated
storage-framework rewrite. The bounded single-slot HIL supports this choice; it
does not validate the owner-context redesign or smartphone performance by itself.

## 2. Memory and record ownership

Partition the existing 128 KiB payload allocation permanently:

| Offset | Bytes | Owner/use |
| ---: | ---: | --- |
| 0 | 32,768 | Rolling history, updated by stream owner |
| 32,768 | 32,768 | Immutable per-stream history snapshot |
| 65,536 | 65,536 | Recyclable live future queue |

The snapshot is the beginning of the previous 96 KiB future region. Do not reclaim
it after history transmission or switch queue geometry during a stream. ECG has
8 rolling/snapshot blocks and 16 live slots; PPG has 2,048 rolling/snapshot records
and 4,096 live slots. Separate bounded ingress/event/TX storage must be explicitly
budgeted; the 128 KiB total above describes payload regions only.

FINITE still captures 24 ECG blocks or 6,144 PPG future records (96 KiB). Split
capture quota constants from live queue capacity; do not accidentally shorten
FINITE to the new physical queue size. Stop enqueueing at the finite quota while
continuing rolling history. INFINITY keeps recycling freed queue slots.

Publish only complete immutable finalized records. ECG may publish to streaming
before storage accepts a block; stream/file tails may differ after failure and
validated DATA does not certify persistence. Do not add an acceptance barrier or
simply reorder asynchronous operations without preserving buffer lifetime. Ensure
ingress owns a copy or transferred storage before the producer can reuse memory.
A pointer queue alone does not establish lifetime. Transport references must
remain valid until copied into independently owned TX storage or safely retired.
No notification references the mutable rolling history. Storage and streaming
must not release each other's ownership prematurely.

## 3. One owner, bounded handoffs

Use one serialized owner context for session state, mode, offsets, START/STOP,
history/snapshot cursors and terminal status. A bounded system-workqueue handler
owns streaming and directly submits notifications through one TX slot. Bound work
per invocation and reschedule remaining work; do not
starve TX callbacks, RTC work or other system work. The 32 KiB snapshot-copy
interval and producer latency must be measured. Blocking disconnect/HCI operations
run through a separate non-system-work path; do not move the old loop unchanged.

Exactly one context per peripheral publishes records in acquisition order through
a bounded FIFO. Lifecycle/fault notifications may originate in other contexts.
Bluetooth callbacks publish commands and connection/subscription changes from
outside the owner context.
TX callbacks running in the same serialized context may update TX bookkeeping;
the owner still decides session transitions. Status reads use an owner-produced
snapshot or a small explicit synchronization boundary.

Keep one immutable TX buffer occupied until notification completion; successful
submission alone does not free it. Submit the next envelope only after retirement.
Remove multi-slot allocation/scanning, per-slot submission work and multi-completion
coordination. If the callback shares the serialized owner context, update the single
slot directly; otherwise use a minimal completion handoff. Retain connection/session
identity and safe disconnect retirement so a late callback cannot free a reused slot.
Enforce `CONFIG_BT_CONN_TX_NOTIFY_WQ=n`. The managed SDK copies the payload during
submission and may suppress completion on disconnect; owner cleanup invalidates
the token and retires the application slot without awaiting that callback.
Do not reduce Bluetooth stack/controller buffer counts as part of this change.

Use concrete bounded handoffs, not a generic event framework. Commands take effect
against current owner state. Acquisition updates may coalesce to latest state plus
a generation; every discontinuity advances that generation, even across an
unobserved stop/fault/start sequence. Before accepting commands or ingesting records
from a new generation, terminate the old stream, clear history and discard stale
ingress. Latest state may already be startup/recording active. Generation/state
publication must remain reliable when record or command queues are full; exact
transition replay and first-fault preservation are not required.

Keep synchronization at publication/ownership boundaries. Do not put BLE calls,
large copies or waits under a spinlock. No BLE backpressure may stall acquisition
or NAND recording. Maintain connection references, session identity, bounded TX
ownership and stale-completion protection. Disconnect cleanup cannot depend on a
callback that the transport might never deliver.

## 4. Rolling history, resets and immediate START

Zero rolling history at boot and treat it as a full logical ring. Complete records
replace oldest slots. A chronological snapshot before refill completes contains
leading zero slots followed by real records. Remove history-full START gating and
HISTORY_NOT_READY emission; do not create a special boot waiting state.

On acquisition pause, normal stop or acquisition/storage fault, the owner zeros
history and resets its ring position. Reject stale producer events from that
acquisition epoch. Do not zero an existing immutable snapshot while it is owned.
Resume ingests new records without a refill delay. While acquisition stays healthy,
stream STOP, disconnect, timeout and live-queue overflow leave history updating.
Producer-handoff loss clears mirrored history as specified in section 5.

START requires recording active or startup underway, subscription, initialization,
supported MTU and no active/cleaning-up session. At the owner processing boundary,
copy rolling history chronologically into the snapshot using at most two copies. Records
incorporated before that boundary are history; pending ingress and later records
are future. Do not drain to a command-publication frontier or wait for ingress to
empty. Maintain no unintended gap or duplicate within a stream. Overlap between
separate streams is permitted.

ACK confirms stream acceptance, not successful acquisition startup. The stream
does not initiate acquisition. Zero history is permitted while startup is pending;
startup failure ends the stream under the first-terminal-decision rule. Keep the
physical sample-event anchor and never publish unfinished or fabricated blocks.

The copy totals 32 KiB and runs outside interrupt-disabled regions. Buffer arrivals
through the bounded producer handoff during it; measure latency and ensure the
handoff budget is sufficient. Do not add reference counting or snapshot reclamation
instead of this explicit copy without a demonstrated need.

Remove `start_arming` and the stream-specific ECG block-boundary callback. An ECG
block already acquiring at START becomes future data on completion, after any
older pending ingress records.
This does not remove the physical sensor acquisition anchor (section 7).

## 5. Send loop and immediate termination

Use one packetization path for both products and modes. ACK declares 8 history
units of 4,096 bytes and a FINITE total of 131,072 or INFINITY total zero. DATA is
u64 offset plus bytes and may cross records, snapshot/live boundary or ring wrap.
Include zero history bytes normally. END/RESULT carry only u16 status. Preserve
v0 layout and status assignments in the canonical specification; former
HISTORY_NOT_READY is reserved. No v1 compatibility sender remains.

Commands use writes with response. Allow only one unanswered command: START waits
for ACK/RESULT, STOP waits for END before another START. ATT completion alone is
not the application response. Reject excess requests explicitly at ATT admission;
do not queue an unbounded backlog or silently lose outcomes. The central holds user
Stop intent until START_ACK. A rejected STOP resolves via RESULT without ending
the active stream. The former response-reservation queue is replaced by one command
mailbox. Successful ACK/END/RESULT submission releases admission; DATA submission
does not. The owner waits for the occupied TX slot to retire before processing
the next command, and finalizes completed END cleanup before a new START. This
allows a central that has received a response to write its next command before
the peripheral processes the corresponding completion callback. No new wire
status is needed; write-without-response is outside the revised control contract.

STOP processing ends DATA submission immediately. Discard unsubmitted fragments
and unsent backlog; remove stop-target rounding and partial-record drain state.
Already-submitted sends remain valid and precede END, whose status is STOPPED
if STOP wins the terminal decision. If START was accepted,
ACK precedes END even with zero DATA. Repeated STOP while awaiting END is rejected
as an excess command and never creates another END.
A new START waits for cleanup, never for history fullness.

Recording termination uses the same immediate stream cutoff with NOT_RECORDING
if no terminal decision already exists.
The recording path itself still completes a current nonempty ECG block on normal
stop; stream STOP never requests acquisition stop. A stream need not transmit the
last block that reached NAND. Recording discontinuity clears rolling history.

The first terminal decision processed by the owner wins, normal or fault. FINITE
SUCCESS requires all declared DATA submitted. Later faults never replace the
chosen status, even before END submission, but required acquisition/history cleanup
still occurs. A generation change ends the old stream with NOT_RECORDING unless
the observed update carries a known fault status. Exact fault reason/order is
best-effort; keep detailed causes in diagnostics if needed. Failed delivery is a
local abnormal outcome, not a second END. Disconnect retires ownership safely.

A required record arriving at a full live queue ends the stream with
BUFFER_OVERFLOW; merely occupying its final slot is allowed. Never overwrite old
queued data and continue. Healthy NAND recording and rolling history continue.

Producer-handoff overflow has a different, explicitly approved policy: terminate
the stream with BUFFER_OVERFLOW, zero rolling history, and discard stale pre-loss
ingress so it cannot refill history after the reset. NAND recording continues;
subsequent complete records refill history and START never waits for fullness.
This is an allowed history discontinuity on fault, not an acquisition/timer reset.
Keep loss detection reliable even when the record handoff is full. If a terminal
decision already exists, preserve its status while still resetting history and
discarding stale ingress.

Retain the initial configurable 15-second no-progress policy. The owner measures
pending/inflight work without completion, including inability to submit at all;
failed submissions do not reset the timer. Waiting only for the next acquisition
record does not start it. Expiry disconnects and retires the stream. No overall
INFINITY duration limit. Preserve bounded retries and cleanup without a second
independent lifecycle in timer or completion callbacks.

## 6. Receiver implications to implement alongside firmware

The generic central reassembles records independently of notification boundaries.
For ECG, skip only leading all-zero 4,096-byte slots wholly within the first 32 KiB.
Do not emit samples or initialize continuity from padding. Validate every real ECB2
block; zero future slots or padding after real data are errors. FINITE total includes
padding, so transport offsets and valid-sample counts must be separate.

STOPPED/NOT_RECORDING may end mid-record. Keep complete validated ECG blocks or
framed PPG records and discard partial tails. Finish queued validation even after
END/disconnect. PPG gets no validity marker or CRC; zero-record detection is not
reliable and must not be presented as ECG-style validation.

## 7. ECG/IMU sample-event time: preserve the physical constraint

Use the existing extended 32-bit 512-Hz counter directly as boot-relative ECG
sample time. Do not reset it on recording start/stop, file rotation or history
reset. Keep it advancing through acquisition pauses and applicable sleep. Reboot
resets the origin; handling its approximately 97.1-day wrap is out of scope.
`first_sample_index` stays recording-local; no field removal/reinterpretation is
part of this timing change.

Before edits, trace the existing post-SYNCH SAMP/RTC anchor and IMU timing path.
For every ECG sample, preserve alignment to the actual hardware sample event,
including captured timing, FIFO ordering, phase/latency compensation and ordinal
mapping. Interrupt-service, FIFO-read, block-completion and storage timestamps
are not substitutes. A restarted sensor may require a new physical SAMP anchor
against the continuing boot clock. Removing stream START arming is independent
of this requirement. Rework the IMU counter without losing alignment to the same
timebase. Stop for clarification if the existing clock/sleep arrangement cannot
satisfy this without a major design change.

A new recording retains boot-relative time but starts a new recording identity
and local sample index. A pause creates a time gap between recordings. Individual
files/streams retain strict continuity inside their acquisition session; history
reset and stream termination prevent bridging the pause silently. Keep ECB2 byte
layout, CRC and actual sample-event accuracy intact.

## 8. Implementation status and remaining verification

The shared sender replacement and central command changes are implemented and
accepted (**A−, PASS**). Production C/H decreased by 473 lines against `8f386cc`;
configuration decreased by 14. Sender and receiver host harnesses passed, as did
ECG, PPG and both central-target builds. Bounded ECG/DK HIL passed full FINITE,
early deferred STOP, INFINITY restart, disconnect recovery and acquisition
pause/restart. History throughput remained 35.4 KiB/s at MTU 498. Canonical
section 19 records exact build logs, evidence, cleanup and limitations.

Canonical section 10 specifies tests: wrapped/zero history, stale records after
reset, 96 KiB finite capture through 64 KiB queue, prolonged INFINITY, event/TX races,
partial STOP and validation retention, offsets and overflow. This is the broader
verification scope, not a claim that every case was exercised in the bounded run.

Future implementation changes, builds and HIL require task authorization;
canonical section 19 records completed verification. When authorized,
read AGENTS.md and
`D:\senselab-tools\vscode-wrapper\README.md`; use the managed
`C:\ncs\SenSEv2.9.3` workspace and `C:\ncs\toolchains\b620d30767`. Run wrapper
builds serially, wait for final exit codes, and report logs/artifacts for ECGv0,
PPGv2 and `central_nus_test` on nRF5340DK and nRF54L15DK. No commits are authorized
by this documentation edit.
Remaining hardware verification includes copy/producer latency, physical ECG/IMU
alignment, deliberate overflow/backpressure and fault races, and smartphone
centrals. Bounded PPG hardware testing is recorded in canonical section 20.
Bounded sustained throughput and disconnect/reconnect were
tested; exhaustive concurrency and long-duration behavior were not.

Review code removal against both the pre-pass snapshot and Git HEAD; report tests,
docs and production source separately. Do not present an intermediate reduction
as net savings against the original implementation. Stop for major unresolved
design questions rather than adding compensating architecture.
