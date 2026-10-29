Bridge MachXO2 JTAG programmer
=============================

Target: ``bridge/nrf5340/cpuapp``, managed NCS ``C:\ncs\SenSEv2.9.3``.
This app embeds the reviewed ``bridge-hello.vme`` and runs Lattice's full ispVM
Embedded interpreter over GPIO JTAG. It does not treat VME bytes as a bitstream
or implement a replacement Flash algorithm. See ``VME-REVIEW.rst`` for the exact
programming scope and validation limits.

Build and commands
------------------

From PowerShell::

  powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 -ApplicationRoot D:\MotionSenseHRV4Flash\bridge-fpga-dev -Board bridge/nrf5340/cpuapp

Output: ``build/merged.hex`` and ``build/bridge-fpga-dev/zephyr/zephyr.elf``.
The wrapper prints the timestamped log path under ``D:\senselab-tools\logs``.
This application's ``prj.conf`` enables ``CONFIG_FPGA_FLASH_WRITE=y`` at the
owner's request. Nothing programs at boot; only the explicit command below
starts FPGA programming. Set it to ``n`` for a diagnostic-only image.

After MCU installation, open the native USB CDC port with DTR asserted
(115200 baud is a nominal USB terminal setting). The Zephyr shell accepts::

  fpga info
  fpga id

``id`` initializes GPIOs, resets only the JTAG TAP, reads IDCODE, checks the IR
capture signature and single-bit BYPASS path, and returns to TAP reset/idle.
It does not assert PROGRAM_N, enable programming, erase, or program the FPGA.
IDCODE must be exactly ``0x012B8043``, matching the supplied VME comparison.
DONE is reported only as a pin level: its external pullup and feature-dependent
function mean that high is not proof the application is running.

Flash-write support is enabled in this MCU image. To execute the reviewed
sequence, the operator must explicitly invoke::

  fpga program

This rechecks file CRC, device ID and chain before executing the VME. One attempt
is allowed per MCU boot. Errors do not trigger an automatic erase, programming
retry, PROGRAM_N pulse, or added REFRESH instruction. Keep power stable during
the operation. Do not infer application operation or power-cycle persistence
from a VERIFIED message; those are separate hardware tests, currently deferred.

Programming output reports identification, start, failure details, and final
verification status. Expected busy-poll retries and scan progress are silent;
polling exhaustion and verification failures remain errors. Image SHA256 is
checked at build time; no digest argument is required at the shell.

Application SPI WHOAMI
---------------------

After programming FPGA logic that implements the WHOAMI protocol, run::

  fpga whoami

Expected output::

  SPI transfer: <measured> us (polling, includes CS delays)
  FPGA WHOAMI: 0x10AC — PASS

Elapsed time brackets the polling HAL transfer using the Nordic timing
subsystem's 16 MHz TIMER2 (62.5 ns timer resolution). It includes DMA setup,
CS delays, cleanup and any unrelated interrupt preemption, but excludes shell
output. It is not an electrical CS-low measurement.
TIMER2 is reserved for this measurement and runs only during the command.

Each invocation sends exactly ``0F 00 00`` in one full-duplex SPI3 transaction
at 8 MHz, mode 0, MSB first, with 8-bit words. GPIO CS (P0.22) stays low across
all 24 clocks, with a 1 us setup delay and 1 us delay before CS deassertion.
The response is ``xx 10 AC``; the
command-byte response is ignored. Mismatches report the actual and expected
values and all three raw RX bytes. Transfer errors/timeouts return failure to
the shell, and CS is returned high after completion or abort. END is polled
with a 1 ms timeout. On timeout, STOP is requested and STOPPED is polled for
at most another 1 ms. If STOPPED never arrives, CS is released, subsequent
transfers are blocked until MCU reboot, and persistent DMA buffers are retained;
the peripheral is not disabled before STOPPED.

The Zephyr SPI driver is disabled, giving the application exclusive SPIM3
ownership. EasyDMA moves the three bytes without a SPIM interrupt or semaphore
wait. Other interrupts remain enabled. Pin routing comes from the board pinctrl
and CS definitions; initialization holds CS high and SCLK low.

This tests the FPGA application's SPI interface only, runs only on request,
and does not change FPGA Flash or configuration.

Image replacement
-----------------

Generate a new **JTAG Full VME Embedded** file in Diamond with compression and
compact mode off, polling enabled, and USERCODE-skip disabled, matching the
current export. Keep the erase/program/verify operation explicit. The adapter
currently admits CRC-protected ``____12.1`` uncompressed files only; the full
vendor interpreter is retained but other export modes are not qualified.

Set the CMake ``VME_FILE`` cache path if using another filename. Inspect the new
file, run the offline tests/trace, review all erase flags, feature values,
security operations and activation semantics, then update ``reviewed-vme.sha256``.
The embed step fails on a digest mismatch. Review and update the instruction and
erase/feature payload policy in ``src/vme.c`` if the new sequence requires a
change; do not bypass it. ``fpga info`` contains this image's operation summary
and must also be updated for a different sequence. Runtime files/uploads are not
accepted. The build embeds the exact original bytes without rewriting the VME.

Pins and timing
---------------

The shared board DT is checked against ``bridge-docs/BRIDGE.EDF``:

* TCK P1.04 output, initially low; TMS P1.05 and TDI P1.06 outputs, initially high.
* TDO P1.07, IRQ P1.08, DONE P1.10 and shared IO P1.13/P1.14 are MCU inputs.
* PROGRAM_N P1.09 is open drain, active low, initialized released. JTAG_EN is
  pulled high externally; no MCU enable pin exists.
* SPI3 uses SCLK/MOSI/MISO at P0.19/.20/.21, CS P0.22 for ``fpga whoami``.
  JTAG programming sends no SPI traffic. BTN_A/P1.11 and BTN_B/P1.12 are initialized at boot as inputs
  with the DT pull-ups, without interrupts or application actions; LED gates
  and 5 V enable stay low.

TCK has at least 5 us low/setup and 5 us high per pulse (at most 100 kHz).
Interrupts may lengthen pulses. Short delays use ``k_busy_wait``; long delays
use ``k_usleep`` so the USB stack can run. VME bit 15 selects milliseconds,
otherwise delay units are microseconds. Actual waveform timing still needs
scope/logic-analyzer qualification; this is not a measured timing claim.

Host tests
----------

Run ``python bridge-fpga-dev/tests/run_host_tests.py`` from the repository root.
Uses ``C:\cygwin64\bin\gcc.exe``; outputs/logs/traces are in
``bridge-fpga-dev/build/host-tests``. No hardware is accessed.

The full-image host backend supplies synthetic expected TDO, explicitly labeled
SIMULATION ONLY. A separate TAP model checks read-only identification; a TAP
observer verifies emitted data order, state and last-bit TMS on every image scan.
Negative cases cover ID/page verification mismatch, selected/unselected mask
bits, busy retry/recovery and exhaustion, GPIO errors, deadline, CRC/version,
truncation, forbidden security opcode, scan bounds and varint overflow.
The local Cygwin installation lacks libubsan, so these tests do not claim
sanitizer coverage. Two unchanged vendor-code compiler warnings concern an
unused local and intentional switch fallthrough.

Port boundaries and license
---------------------------

``src/ispvm/`` retains the supplied 2023 Lattice notices. Distribute
``LICENSE.ispvm`` with binaries. ``tools/patch_vendor.py`` reproduces the small
core changes from ``bridge-docs/FPGA-JTAG-EXAMPLE``: scan/compare/poll diagnostics,
scan and varint bounds, and a checksum printf format fix. TAP transitions,
bit shifting, last-bit handling, masks and retry algorithm remain vendor code.

The adapter replaces file I/O, memory allocation and example parallel-port
hardware. It checks 8-bit bytes, 16-bit short and 32-bit int. Target long is
32-bit; host long is 64-bit, with varints bounded to the common signed 32-bit
range. The vendor checksum long is masked to 16 bits. Allocations have independent
capacity tracking, checked failure and single-owner cleanup, replacing the
example's shared previous-size heuristic. No allocation occurs in an ISR.
The shell thread has 8 KiB stack; allocation pool is bounded to 64 KiB. An
operation deadline is 180 seconds in addition to the VME's bounded polling.
Heap/global interpreter state is deliberately single-use per boot.
