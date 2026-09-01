# ECG Filesystem Corruption: Scope and Architecture Review

Date: 2026-08-29  
Status: Engineering note for later consideration; no fix is approved by this document

## Executive conclusion

The observed ECG corruption appears to come from an unsafe filesystem ownership
handoff, not from an inherently invalid storage geometry or ECG sample format.
After collection stops, ECG-specific filesystem logging can continue producing
asynchronous writes while files are being closed and the FAT volume is being
returned to USB mass storage. Log text appearing as FAT root-directory content is
consistent with writes occurring during that transition.

The immediate defect is narrower than the attempted fix. A correct repair needs a
strict producer/queue/file/USB ordering, but it should not automatically require a
broad ECG/PPG/shared-driver redesign. The reviewed implementation combined the
targeted ECG repair with several architectural and defensive changes across roughly
nineteen files. Some of those changes are independently valuable, but combining them
made review harder and introduced a release-blocking workqueue return-value bug.

The recommended approach is to separate:

1. A focused ECG handoff correction and executable lifecycle tests.
2. A later, explicit storage-ownership architecture change shared by ECG and PPG.

## Core issue

Stopping sample collection does not necessarily stop filesystem activity. The ECG
firmware has multiple relevant actors:

- ECG/sample producers;
- the filesystem log backend;
- partial producer buffers;
- asynchronous workqueue items;
- open FatFS file objects;
- the raw NAND disk driver;
- USB mass storage and its host filesystem activity.

The original implementation implicitly treated `collecting_data = false` as a global
filesystem stop condition. That was not true for ECG filesystem logging. Pending work
could also outlive the producer that submitted it. Closing files without first proving
that all writers are quiescent therefore permits a queued or newly produced log write
to use filesystem state after close or during the USB transition.

PPG did not reproduce the observed failure because its logging policy was tied more
closely to collection state. That difference explains why the problem can be
ECG-specific even though much of the lower storage stack is shared.

## Minimum safe lifecycle

A stop-to-USB transition should have one clearly defined owner and follow this order:

1. Prevent every filesystem producer from submitting new work. This includes the
   filesystem log backend, not only ECG sampling.
2. Wait for any logging callback currently inside its critical section to leave.
3. Drain already queued or running writes.
4. Submit the final partial ECG and custom-log buffers.
5. Drain again while preventing later submissions.
6. Sync and close every open file, retaining a fault if either operation fails.
7. Deny firmware writes to the raw disk.
8. Make the mass-storage volume available to the host.

The reverse transition must claim storage ownership before reopening the queue or
reenabling filesystem logging. Any failed transition must leave USB mass storage
unavailable and firmware writes gated until recovery establishes a known state.

This ordering is somewhat delicate because asynchronous work has explicit ownership
and API return-value rules. It is nevertheless a bounded lifecycle problem and can be
implemented locally if its ownership assumptions are made explicit.

## Is the storage architecture wrong?

Presenting the same FAT volume to both firmware and a USB host is not automatically
wrong, but simultaneous access is unsafe. The design requires exclusive ownership:

- while firmware has the filesystem mounted and writable, the host must not access a
  live, changing mass-storage volume;
- before the host sees the volume, firmware producers, work items, buffers, and files
  must all be quiescent;
- the transition itself must be atomic from the perspective of both sides.

The awkward requirement is the desire to retain USB CDC console/logging during
collection. USB transport availability and mass-storage-media availability are
separate concepts. A cleaner architecture would keep CDC active while making only the
mass-storage LUN unavailable, for example by detaching/gating the LUN or reporting no
medium during recording. Disabling the entire USB device is a coarse workaround that
also removes CDC logging.

If the current USB stack cannot independently control CDC and mass storage, that is
the genuinely onerous requirement. It should be addressed as an explicit design
decision rather than hidden behind `CONFIG_USB_ALWAYS_ON` conditionals.

## Assessment of the original implementation

The original implementation was naive about several concurrency and lifecycle facts:

- clearing `collecting_data` did not stop all filesystem producers;
- queued work could survive the stop operation;
- a sampled work-item busy state was not a reservation or ownership guarantee;
- partial buffers required a deliberate final submission and drain;
- file close alone did not establish that no worker could write afterward;
- filesystem logging and USB exposure did not share one authoritative ownership
  policy.

These are repairable lifecycle assumptions, not proof that the entire firmware
architecture must be replaced.

## Assessment of the attempted broad fix

The attempted change included several sound ideas:

- atomically gating the filesystem log producer;
- rechecking the gate while holding the callback lock;
- reserving work-item ownership before submission and releasing it after the worker;
- draining before reusing partial buffers;
- attempting close even after sync failure;
- retaining unresolved close/sync errors as handoff faults;
- keeping USB disabled on a failed handoff;
- isolating lifecycle ordering behind injected callbacks.

However, it also expanded into shared NAND ownership, PPG behavior, Kconfig removal,
multiple call-site changes, and static contract testing. That breadth obscured the
smallest causal repair and increased regression risk.

### Release-blocking defect found during review

The new lifecycle misinterprets `k_work_queue_drain()` in the project's NCS/Zephyr
version. Its contract is:

- `0`: successful drain that did not need to wait;
- `1`: successful drain that had to wait;
- negative: interrupted or failed drain.

The implementation used `ret != 0` as its failure test in the preliminary and final
drains, and used equality to zero when recording plugged-queue recovery. Normal queued
work can therefore cause a successful handoff to enter the fault state. A final drain
after submitting a partial buffer is particularly likely to return `1`. The wrapper
returns the Zephyr value unchanged, so there is no normalization layer that makes the
comparison valid.

This defect demonstrates why a static source-order test is insufficient. The added
CTest checks that operations appear in the desired textual order, but it does not
execute the lifecycle or validate Zephyr-compatible callback results.

### PPG policy inconsistency

The attempted-fix report stated that PPG disables USB before granting firmware write
ownership. The effective PPG configuration contains `CONFIG_USB_ALWAYS_ON=y`, while
the disable call is compiled only when that symbol is absent. Consequently, the built
PPG image does not perform the described transition.

The shared disk mutex and rejection of host writes may reduce risk, but that is a
coexistence policy rather than exclusive USB ownership. Host reads can still observe
a changing FAT volume unless the LUN is otherwise made unavailable. The intended PPG
policy must be stated and tested instead of inferred from conditionally compiled code.

### Retry semantics requiring clarification

The attempted lifecycle advertises explicit retry. At the same time, a sync error can
be stored in `filesystem_close_error` until `setup_disk()` performs a fresh setup. A
retry that merely calls close again may therefore remain faulted indefinitely even if
the original error was transient and all handles are now closed.

That behavior may be a valid conservative policy, but it means the fault requires a
remount or reset rather than an ordinary handoff retry. Recovery categories should be
explicit:

- transient operation failure that can safely be retried in place;
- unknown filesystem state requiring unmount/remount;
- media or lower-layer failure requiring reset, repair, or operator intervention.

No recovery path should expose USB mass storage merely to make a retry appear
successful.

## Recommended scope and sequencing

### Change 1: focused ECG repair

Keep the immediate correction primarily within ECG logging policy, the collection
start/stop path, and filesystem queue handling:

- establish a synchronized ECG filesystem-log gate;
- stop all ECG filesystem producers;
- correctly drain existing work, accepting both `0` and `1` as success;
- flush partial ECG and log buffers;
- perform a final plugged drain;
- sync and close all files;
- expose mass storage only after successful completion;
- claim firmware ownership before reopening the queue on collection start;
- retain a safe fault state on genuine negative errors.

Work-item reservation can be included if analysis confirms that the existing sampled
busy check permits metadata or buffer reuse. It should be described as a distinct race
fix rather than conflated with the ECG logging root cause.

Avoid changing PPG, shared Kconfig, or broad USB policy unless a specific dependency
makes that unavoidable. Any unavoidable shared change should be isolated and justified
with a narrow contract.

### Change 2: storage architecture decision

Handle the broader questions separately:

- Must CDC logging remain active during collection?
- Can the USB mass-storage LUN be independently hidden or marked unavailable?
- Are host reads prohibited as well as host writes while firmware owns the volume?
- Does firmware keep FatFS mounted while the host owns the disk, or should it unmount?
- What operation is the single authoritative ownership transition?
- Should ECG and PPG follow the same policy?
- Which failures are retryable, and which require remount/reset?

Once decided, implement one shared ownership contract instead of product-specific
conditionals that happen to produce different effective behavior.

## Required behavioral validation

The callback-injected lifecycle should have executable native/unit tests. At minimum,
exercise:

- preliminary drain returns `0`;
- preliminary drain returns `1`;
- final plugged drain returns `0`;
- final plugged drain returns `1`;
- negative preliminary and final drain results;
- producer gate, drain, flush, close, read-only, and USB ordering;
- partial-buffer submission failure without discarding the buffer;
- stop success reaching the quiesced state;
- genuine stop failure reaching the fault state;
- retryable fault recovery;
- failed resume restoring no-write and plugged-queue state;
- close failure versus sync failure recovery requirements;
- rejection of collection start while ownership is unresolved.

Static source-contract checks may remain as supplementary guardrails, but they are not
evidence that the state machine behaves correctly. Successful firmware compilation is
also not evidence that concurrency or API semantics are correct.

After host tests pass, build both ECG and PPG because they share the NAND driver. The
ultimate acceptance test remains hardware stop-collection testing with filesystem
inspection. It should verify repeated stop/start cycles, partially filled buffers,
heavy logging during shutdown, USB attach/detach timing, unexpected disconnects, and
power loss around the transition.

## Evidence and current uncertainty

During review, both reported ECG and PPG builds and their three CTests were confirmed
to have completed successfully, and `git diff --check` passed. Those results do not
overcome the workqueue semantic defect because the relevant tests are static rather
than behavioral.

No hardware validation was performed as part of this review. The ownership/lifecycle
explanation strongly matches the corruption signature, but eliminating it in hardware
still needs to be demonstrated. Lower-layer NAND, media, or power-loss behavior should
remain a secondary hypothesis until the focused lifecycle fix passes repeated device
testing.

## Decision guidance

Before accepting another large patch, require answers to these questions:

1. Which individual edits are necessary to eliminate the established ECG race?
2. Which edits address separate pre-existing risks?
3. Which edits change architecture or product policy?
4. Does executable testing cover every nonstandard API return convention?
5. What exact state does each failure leave behind?
6. Can the mass-storage function be gated without disabling CDC?
7. Does the effective build configuration match the stated behavior?

The immediate goal should be a small, auditable repair that establishes exclusive
filesystem ownership. Broader cleanup should proceed only after its requirements and
failure model are agreed upon.
