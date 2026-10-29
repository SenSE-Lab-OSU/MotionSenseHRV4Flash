Review of bridge-hello.vme
=========================

File identity and compatibility
-------------------------------

* Size: 49,599 bytes.
* SHA-256: ``023bb9c5f7fe5bd078577e4fc96d88da4503c2aa241f398e38e6006cfbf14b2f``.
* FILE_CRC marker ``0x65``, expected/calculated CRC ``0x0243``; validated before
  any VME-driven JTAG operation. Despite its name, the supplied
  ``ispVMCalculateCRC32`` maintains a 16-bit accumulator.
* Format ``____12.1``, compression marker ``0xF2`` (uncompressed), vendor Lattice.
* Embedded comment identifies Diamond Deployment Tool 3.14. Full interpreter
  source version 12.2 explicitly lists this format as supported.
* MEM operand 128 bits; IR length 8; all IR/DR chain headers/trailers zero.
* Status polling is present; USERCODE skip/continue-if-fail is not used.
  No compact repeat-data path was observed. The file is a self-contained
  algorithm plus data, not the separate algorithm/data export format.

Operations observed from the full interpreter trace
---------------------------------------------------

1. Read IDCODE (0xE0), compare all 32 bits against ``0x012B8043``.
2. Preload 104 boundary bits with ones (0x1C), check status (0x3C).
3. Enable configuration (0xC6), erase SRAM with 0x0E operand ``0x01``.
4. Enable offline configuration with operand ``0x08``; check status.
5. Erase with operand ``0x0E``: Feature Row, configuration Flash and UFM
   selection bits are set. This is broader than configuration-only erase.
6. Poll busy (0xF0) with bounded LCOUNT, check failure/status bits.
7. Initialize address (0x46); write 575 128-bit pages with 0x70, polling each.
8. Read/program USERCODE (0xC0/0xC2); programmed value is zero.
9. Reinitialize address, read back 575 pages (0x73), compare every page's
   expected data, verify USERCODE and status.
10. Program Feature Row (0xE4) with eight zero bytes, poll, read/verify (0xE7).
11. Program FEABITS (0xF8) with wire bytes ``20 06`` (16-bit value ``0x0620``),
    poll, read/verify (0xFB), wire mask ``F2 FF`` (``0xFFF2``).
12. Check status, program DONE (0x5E), poll, check instruction capture,
    disable configuration (0x26), BYPASS, final status comparison.
    Final status expected ``0x00000100``, mask ``0x00002100``.

No 0xCE SECURITY, 0xCF SECPLUS or 0xF9 OTP instruction is present or admitted by
the runtime policy. Feature writes above are part of the supplied file, not
extra commands added by the adapter. The references reviewed do not give a
complete raw FEABITS-to-pin-policy mapping here; do not claim independently
proven JTAG persistence from the numeric FEABITS value alone. Confirm the
Diamond project feature settings before authorizing this sequence if needed.
JTAG_EN is externally pulled high on this board.

Activation
----------

There is no explicit LSC_REFRESH (0x79) command in this file. It clears SRAM
early, programs DONE, exits offline mode through ISC_DISABLE, then checks DONE
and the failure bit. The MachXO2 guide describes automatic reconfiguration on
leaving offline mode with SRAM cleared. This sequence therefore requests
activation, but no running-design behavior has been measured. External DONE
alone cannot establish that the expected design is running. The owner deferred
functional application and power-cycle checks.

Validation and current limits
-----------------------------

The complete offline successful path executes 2,934 scans, including 1,167
comparison scans and 189,766 TCK pulses (including interpreter start/end).
Simulated minimum delay total is 10.195660 seconds; real USB/GPIO/scheduling
overhead will add time. Busy-never-clears exhausts 900 erase-poll attempts and
returns failure before page programming. Corrupt/truncated files, wrong ID,
page mismatch, masks, transport faults, timeout and malformed lengths are tested.
These are host simulation results, not Flash verification on the FPGA.

The owner has authorized enabling Flash-write support and installing the MCU
firmware, and will run the FPGA programming command manually. The earlier
diagnostic image reported IDCODE 0x012B8043 and a passing chain test. No FPGA
erase/program has been performed by the agent. Do not silently convert this image into
configuration-only programming by deleting its feature commands.

Sources inspected
-----------------

* ``bridge-docs/FPGA-JTAG-EXAMPLE/{ivm_core.c,ispvm_ui.c,hardware.c,vmopcode.h}``.
* ``bridge-docs/FPGA-TN-02155-5-0-MachXO2-Programming-and-Configuration-User-Guide.pdf``,
  configuration flow, Feature Row sections and command table (printed pp. 70-72).
* ``bridge-docs/FIRMWARE-HANDOFF.md`` and ``BRIDGE.EDF`` for connectivity.
* ``bridge-docs/FPGA-DS-02056-4-7-MachXO2-Family-Data-Sheet.pdf``, printed p. 86:
  JTAG maximum 25 MHz, minimum high/low pulse 20 ns, setup 10 ns, hold 8 ns,
  and TDO valid within 10 ns after falling TCK (commercial/industrial).
* Installed Diamond 3.14 help under
  ``D:\lscc\diamond\3.14\docs\webhelp\eng\User Guides\Device Programming``:
  ``embedded_system_deployment_type.htm``, ``creating_a_new_deployment.htm``,
  ``Dev_Prop_DB.htm``. These distinguish Full VME, compact/compressed options,
  polling versus fixed pulse width, and USERCODE-based skipping. Export choices
  are determined from this file where possible, not assumed from UI defaults.
