Bridge ECG notification central
==============================

Build with the project-managed NCS 2.9.3, sysbuild, and target
``bridge/nrf5340/cpuapp``. CPUAPP runs the central and USB console; sysbuild
adds the Bluetooth HCI IPC controller on CPUNET. Flash both images when
switching from the ESB application.

Uses the same ``boards/senselab/bridge`` board as the ESB app. ``app.overlay``
includes ``../bridge/app_core/app.overlay`` for USB CDC ACM, LEDs, and the
5 V supply wiring, overriding its boot-on/always-on flags so the rail starts
off and is enabled only while an LED is requested. Application-specific IPC
overlays reserve the last 64 KiB of CPUAPP SRAM for both cores; the base board
and ESB app are unchanged.

The shared board also provides UARTE0 TX on EEG_TRIG (P1.00), 9600 baud,
8N1, without RX or hardware flow control, accessible as ``DT_ALIAS(eeg_uart)``.
Each valid notification sends its decoded RTC value as unsigned decimal ASCII
digits followed by one NUL byte (``0x00``), with no CR or LF. For example,
512 ticks sends ``35 31 32 00`` in hexadecimal. Values are not padded.
Logging remains on USB CDC ACM.

Behavior
--------

* Active scan for the exact complete name ``MSense4ECG-0ATEJ`` (the ECG sends
  its name in the scan response). No RSSI threshold or connection to other names.
* Scan immediately at boot, then for 10 seconds every 30 seconds if absent.
  A connection failure or disconnect triggers a retry after 30 seconds.
* Discover service ``da39c950-1d81-48e2-9c68-d0ae4bbd351f``, characteristic
  ``da39c951-1d81-48e2-9c68-d0ae4bbd351f``, and its CCC descriptor. Subscribe
  afresh on each connection. No collection-control or sensor-stream writes.
* Each valid 8-byte update pulses one LED for 200 ms: violet, pink, cyan,
  repeating. The sequence starts with violet at Bridge boot and continues
  across reconnections. The 5 V rail is enabled just before each pulse and
  disabled after the LED is turned off, including LED error cleanup.
  LEDs and their rail are off between packets. Buttons remain disabled.
* USB CDC ACM logs scan, connection, subscription, packet count, RTC ticks,
  LED color, and errors. Open the ``SenseLab Bridge Central`` serial port
  with DTR asserted (e.g. a normal serial terminal, 115200 baud). USB baud
  rate is nominal. Operation does not wait for a terminal; early logs may
  be missed if it is opened later.

ECG firmware compatibility
--------------------------

The current ``ECGv0/src/BLEService.c`` keeps the legacy ENMO/motion UUIDs,
but sends timing-only updates: eight bytes, first four zero, last four a
little-endian 32-bit RTC counter in 512 Hz ticks. Updates occur every three
seconds **while ECG collection is active**. A connected but idle ECG will
produce no LED pulses; Bridge does not start collection. Unexpected packet
lengths are logged and ignored. This app does not interpret the packet as ENMO.

The complete-name filter is device selection, not authenticated identity.
