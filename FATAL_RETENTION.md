# Fatal record retention

This diagnostic feature preserves evidence of intermittent collection-start
reboots; it does not fix or establish their cause. The implementation lives in
`shared/fatal_retention.c`, with ECG/PPG integration and two small hooks in the managed
SDK at `C:/ncs/SenSEv2.9.3/zephyr` (`kernel/fatal.c` and
`arch/arm/core/cortex_m/fault.c`). The Nordic fatal handler is unchanged.

Reserve 512 bytes at `0x2006fe00`, immediately below IPC RAM, as NOLOAD memory.
Both application and MCUboot must exclude it from ordinary RAM. The board DTS
and product Partition Manager inputs establish this; build assertions check agreement.
The current record uses 124 bytes. Keep one record, overwrite on each fatal, and
increment an independently validated 32-bit counter. Commit magic last, after
payload checksum. No allocation, filesystem access or logging occurs in capture.

The ARM hook saves raw fault status before Zephyr clears it; the generic fatal
hook finalizes the record before generic fatal logging. Recoverable ARM faults
discard their snapshot. Startup prints every defined field in 16 short lines.
Collection-start breadcrumbs are defined in `shared/include/msense_fatal_retention.h`.
Fault-address registers are meaningful only when their CFSR validity bits are set.

Storage reset and teardown-reboot paths use `msense_normal_reset()` to clear all
512 bytes, including the counter, before reset. BLE reset command `0x79` issues
a direct software reset and preserves the retained record. Debugger system reset
does not call the clearing wrapper. MCUmgr/DFU and network-core fatal capture are
outside this change.

RAM retention is limited to compatible resets. Fatal-handler software reset and
debugger SYSRESETREQ were verified. Pin reset corrupted the record on this ECG;
Nordic also documents possible RAM corruption on watchdog reset. Power loss is
unsupported. Use debugger system reset when preserving evidence. This approach
cannot guarantee capture if a fault never reaches the hooks or faults again
before capture completes; it is not a complete stack dump.

On 2026-09-07, root reviewed the Sol-High implementation (A-, PASS for software
reset retention), built ECG, and tested probe 1057737963: boot smoke, UDF capture,
debugger system-reset preservation plus second-fault overwrite/count 2, and
production reset-function clearing. All retained fields matched RAM; maximum
visible report width was 69 columns including the log prefix. Reset-function
testing used debugger PC redirection, not an over-air BLE command. No firmware
test helper was needed. Pin-reset retention failed as described above.

ECG build log: `.codex-build-logs/ncs-build-20260907-044647-164.log`.
Local HIL evidence, artifact hashes and helper: `build-fatal-hil/` (ignored).
Final target state: running, waiting for ship-mode button, record/counter zero.
SDK and application changes are uncommitted; both trees are required to reproduce.

PPG was integrated with the same retained layout and hooks on 2026-09-07.
Validation was build and map inspection only; no PPG HIL was performed. The PPG
application and MCUboot RAM both end at `0x2006fe00`, and Partition Manager places
the 512-byte record immediately below IPC. The PPG image uses 271488 of 441856
flash bytes (61.44%), leaving 170368 bytes of slot headroom. PPG build log:
`.codex-build-logs/ncs-build-20260907-050452-914.log`.
