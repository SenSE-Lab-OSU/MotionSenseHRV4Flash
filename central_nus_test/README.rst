MotionSense NUS Central Tester
==============================

This is a deliberately small, temporary tester for the MotionSense version-1
bounded sensor stream. It runs on an nRF5340 DK or nRF54L15 DK as a Bluetooth
LE Central and connects to either a PPG or ECG peripheral. It discovers
standard Nordic UART
Service (NUS), requests the fixed device-specific stream (131,072 bytes for
PPG or 131,076 bytes for ECG), validates the protocol, and relays every
received NUS notification unchanged to a second interface-MCU virtual COM port.

It is for protocol and interoperability testing, not a production Central.

The full NUS-capture and unattended BLE DFU runbook is in
``OPERATIONS.md``. It documents the VCOM mapping, binary-port hand-off,
machine-readable DFU protocol, host CLI, recovery policy, security default,
and validation procedure.

Build and flash
---------------

Build with the project wrapper and the desired application-core target.
For the nRF5340 DK:

.. code-block:: powershell

   powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
     -ApplicationRoot 'D:\MotionSenseHRV4Flash\central_nus_test' `
     -Board 'nrf5340dk/nrf5340/cpuapp' `
     -BuildDirectory 'D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340' `
     -Pristine

For the nRF54L15 DK:

.. code-block:: powershell

   powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\senselab-tools\vscode-wrapper\ncs-build.ps1 `
     -ApplicationRoot 'D:\MotionSenseHRV4Flash\central_nus_test' `
     -Board 'nrf54l15dk/nrf54l15/cpuapp' `
     -BuildDirectory 'D:\MotionSenseHRV4Flash\central_nus_test\build_nrf54l15' `
     -Pristine

The nRF5340 build includes the standard HCI IPC Bluetooth Controller image for
the network core. It produces two domain images, which must both be programmed
in the order recorded by ``build_nrf5340\domains.yaml``:

* ``D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\merged_CPUNET.hex``
  (network-core controller)
* ``D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\merged.hex``
  (application-core tester)

For example, when ``nrfutil`` is available and a single DK is attached:

.. code-block:: powershell

   nrfutil --log-output stdout device program --serial-number <serial-number> --core network --firmware D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\merged_CPUNET.hex --options verify=VERIFY_READ,chip_erase_mode=ERASE_CTRL_AP,reset=RESET_NONE
   nrfutil --log-output stdout device program --serial-number <serial-number> --core application --firmware D:\MotionSenseHRV4Flash\central_nus_test\build_nrf5340\merged.hex --options verify=VERIFY_READ,chip_erase_mode=ERASE_CTRL_AP,reset=RESET_NONE
   nrfutil --log-output stdout device reset --serial-number <serial-number> --core application --reset-kind=RESET_PIN

The nRF54L15 has an application-core-local Bluetooth controller, so its build
does not produce or require a separate ``merged_CPUNET.hex`` image.

Interface-MCU virtual COM ports
-------------------------------

Connect the DK's **interface-MCU USB connector** to the host. It exposes the two
VCOM ports used by the tester. On the nRF5340 DK, the native USB connector is
not used for tester traffic. Open the command port at **115,200 baud, 8N1**
and the binary relay port at **1,000,000 baud, 8N1**.
The host must assert DTR on each open port so the interface MCU connects its
UART pins. Standard terminal programs and a normal ``pyserial`` open do this;
set ``dtr = True`` explicitly when in doubt.

The firmware assigns the VCOMs as follows on the nRF5340 DK:

* **Command/control:** nRF UARTE1, interface-MCU Serial Port 0, pins P1.01
  (TX) and P1.00 (RX), normally marked ``VCOM0`` on the DK. It carries plain
  ASCII commands and replies at 115,200 baud.
* **Binary relay:** nRF UARTE0, interface-MCU Serial Port 1, pins P0.20 (TX)
  and P0.22 (RX), normally marked ``VCOM2`` on the DK. It uses the associated
  P0.19/P0.21 RTS/CTS pair for hardware flow control and carries only relay
  frames at 1,000,000 baud.

On the nRF54L15 DK, the assignments are:

* **Command/control:** nRF UARTE30, interface-MCU Serial Port 0, pins P0.00
  (TX) and P0.01 (RX) at 115,200 baud.
* **Binary relay:** nRF UARTE20, interface-MCU Serial Port 1, pins P1.04 (TX)
  and P1.05 (RX) at 1,000,000 baud, without RTS/CTS hardware flow control.
  The relay uses a bounded asynchronous TX operation, so a stalled VCOM cannot block BLE
  notification processing.

COM numbers and some DK silkscreen labels are not reliable identifiers. Open
both VCOMs, assert DTR, and send ``help`` followed by a newline to identify the
command port; it replies with the command list. The relay port emits no text
and carries only binary frames once a NUS notification is received. Keep the
relay VCOM open through the complete transfer. The firmware cannot observe
host DTR on these physical UARTEs. On the nRF5340, an unopened relay can hold
CTS, eventually fill the bounded relay queue, and invalidate the capture. On
the nRF54L15, flow control is intentionally disabled so an unopened relay
cannot stall BLE; the host must keep its reader open to avoid losing relay data.

RTT is diagnostic-only and never carries command or relay traffic. It reports
``UART_INIT_OK`` after both VCOM UARTEs are ready. On the nRF5340 DK, verify
the relevant UART interface solder bridges are closed (SB27--SB30 and
SB50--SB53) before treating a VCOM problem as firmware related.

Command protocol
----------------

Send one ASCII command followed by CR, LF, or CRLF on the command port.
Commands are case-sensitive:

.. code-block:: text

   help
   scan
   connect ppg
   connect ecg
   connect any
   status
   start
   start 123
   start 0x1234ABCD
   cancel
   cancel 123
   disconnect

``scan`` stops after reporting the first advertising name beginning ``MSense``.
``connect ppg`` and ``connect ecg`` retain their respective PPG/ECG name
filters; ``connect any`` accepts any ``MSense`` name, including
``MSenseBlinky``. Exact peripheral addresses and multiple simultaneous peers
are intentionally unsupported.

``start`` chooses a nonzero session ID, unless one is supplied. It is accepted
after ``NUS_READY`` and also after a prior terminal ``COMPLETE`` state on the
same connection. This permits an immediate repeat request to test the expected
``HISTORY_NOT_READY`` result. ``cancel`` without an ID names the active session;
a supplied ID is useful for deliberately testing
``WRONG_SESSION`` behavior.

Useful command-port results include:

.. code-block:: text

   ATT_MTU 498
   NUS_READY mtu=498 tx=0x.... rx=0x.... cccd=0x....
   START_SENT id=1
   START_ACK type=PPG ... records=2048+6144 bytes=131072 ...
   STREAM_OK id=1 bytes=131072 data_messages=...

``START_RESULT`` is a rejected START, for example ``NOT_RECORDING`` or
``HISTORY_NOT_READY``. ``STREAM_END`` means the accepted session did not pass
the success checks. ``PROTOCOL_ERROR`` and any relay error invalidate the test.

Throughput diagnostics
----------------------

The tester reports the negotiated link state as ``BLE_LINK`` after connection
setup and whenever the controller reports a connection-parameter, PHY, or data
length update. Its ``tx_*`` fields describe Central-to-peripheral transport;
the stream direction is represented by the ``rx_*`` fields. ``interval_ms_x100``
is the interval in hundredths of a millisecond, avoiding floating-point output.

When the first forward DATA message arrives, the tester posts a completed
history-phase summary. A successful END then posts the forward-phase summary
and the backwards-compatible total summary before ``STREAM_OK``. For example,
the PPG byte counts are:

.. code-block:: text

   THROUGHPUT_HISTORY id=1 ... sensor_bytes=32768 ...
   THROUGHPUT_FORWARD id=1 ... sensor_bytes=98304 ...
   THROUGHPUT id=1 ... sensor_bytes=131072 ...

``raw_nus_bytes`` is the complete received DATA notification, including the
MotionSense common and DATA headers but excluding ATT and relay framing.
``sensor_bytes`` contains only sensor records. The ``active_elapsed_ms`` for a
phase spans its first through last valid DATA notification; its rate fields use
that active period. For history, ``request_elapsed_ms`` separately spans from
immediately before the Central queues its START GATT write through the last
history DATA callback. It therefore includes command, peripheral, and history
availability delay rather than presenting the short history burst as the whole
request.

The total ``THROUGHPUT`` line retains its original ``elapsed_ms`` field and
spans the first valid DATA notification of either phase through the last one.
Consequently its rate and gap can include the wait between history and forward
capture. History and forward ``max_gap_ms`` values are scoped to their own
phase. Rates are shown as decimal values without floating-point support. A
single DATA notification, or notifications with the same millisecond timestamp,
has an active elapsed time of zero and reports zero rates rather than an
undefined or infinite value; its counts, byte totals, and mean size remain
valid. A ``status`` command additionally prints total ``THROUGHPUT_LIVE``
using the current time while a stream is active, followed by the most recently
reported ``BLE_LINK`` values.

Binary relay framing
--------------------

The relay stream is a sequence of frames. It is independent of UART read
boundaries; a host must accumulate bytes, locate the magic, validate the
header, and then consume exactly one frame.

All multibyte fields are little-endian:

.. list-table:: Relay frame version 1
   :header-rows: 1

   * - Offset
     - Size
     - Meaning
   * - 0
     - 4
     - ASCII magic ``MRLY`` (``4D 52 4C 59``)
   * - 4
     - 1
     - Relay version, always ``1``
   * - 5
     - 1
     - Frame type, always ``1`` = raw NUS TX notification
   * - 6
     - 2
     - Raw notification length
   * - 8
     - 4
     - Relay sequence, starting at zero after reboot
   * - 12
     - variable
     - Exact raw NUS TX notification bytes

There is no relay CRC. On the nRF5340, relay RTS/CTS prevents the interface-MCU
UART from overrunning during bursts. On the nRF54L15, each relay frame uses a
bounded asynchronous UART TX and a timeout abort prevents a stalled relay from
starving BLE processing. The enclosed NUS common header supplies a second
length/version check. For resynchronization,
discard bytes until ``MRLY`` appears, then require version 1, type 1, a length
at most 512 bytes, and a complete enclosed NUS message before accepting it.

The raw notification starts with the protocol's ``MS`` common header. See
``../NUS_SENSOR_STREAM_CENTRAL_HANDOFF.md`` and
``../shared/include/msense_sensor_stream_protocol.h`` for the NUS message
layouts. The relay includes START_ACK, DATA, END, and RESULT exactly as
received; it does not contain command-port status text or transport delimiters.

Expected workflow
-----------------

1. Start recording on the PPG or ECG device and wait for its fresh history
   window (at least 8 seconds for PPG or about 5.334 seconds for ECG).
2. Flash the DK, connect J2, open both VCOM ports with DTR asserted, and start
   a binary reader on the relay port before sending any command.
3. Send ``connect ppg`` or ``connect ecg``. Wait for ``NUS_READY`` and an ATT
   MTU of at least 128.
4. Send ``start``. Keep the relay reader running through the frame containing
   the raw NUS END message. ``STREAM_OK`` means the tester's in-device checks
   passed, but it can be printed before the relay thread has physically written
   every queued frame to the VCOM UART.
5. Independently parse the relay frames and verify the raw stream against disk
   records. The tester itself checks framing, START_ACK geometry, DATA
   sequence/index/phase/count, END counts, and exactly 131,072 PPG or 131,076
   ECG sensor bytes.

The peripheral must acquire the 96 KiB future portion before it can finish:
roughly 24 seconds for PPG and 16 seconds for ECG, plus BLE drain time.

Scope limits
------------

* One connection, one active stream, and one first-match name scan only.
* NUS capture does not add raw-record decoding or local capture persistence.
  BLE DFU reconnect/resume and bounded retry handling are documented separately
  in ``OPERATIONS.md``.
* No command-byte mutation interface for malformed or unsupported-version
  command injection, and no forced low-MTU negotiation test mode.
* If the relay queue overflows, the tester marks the session failed rather than
  silently claiming a valid capture. It does not attempt a complicated
  automatic recovery or cancellation sequence.
* It validates protocol version 1 only. Unknown messages, malformed reserved
  fields, sequence gaps, phase violations, or mismatched successful END counts
  fail the session.
