SenseLab Bridge
===============

ESB PRX application for the custom Bridge / mBridge NORA-B106 board, using
the project-managed NCS workspace ``C:/ncs/SenSEv2.9.3``.

Select ``bridge/nrf5340/cpunet`` with sysbuild. The companion
``bridge_app_core`` uses ``bridge/nrf5340/cpuapp`` and retains ownership of
all board GPIOs. It replaces Nordic's empty application-core sample, which
assigns GPIOs to the network core and powers off application RAM.
Board and binding roots are set for both sysbuild and individual images.

The receiver keeps the sample's radio settings and demonstration ACK payload.
It logs through RTT; there is no DK LED library.
The radio protocol still needs to be matched to the transmitting devices.
There is no inter-core packet transport yet.

The application-core overlay enables USB CDC ACM as the console. The 5 V LED
regulator is enabled at boot and remains on. Pink, violet, then cyan blink
repeatedly, each on for 300 ms followed by 100 ms off. The sequence does not
wait for USB enumeration or an open terminal.
CDC ACM provides a console, not an ESB packet stream or command protocol yet.
The overlay omits voltage bounds for the fixed regulator because this SDK's
driver lacks the voltage-read operation required by common initialization.
The rail remains physically fixed at 5 V; firmware controls only its enable.

The shared board routes UARTE0 TX to EEG_TRIG (P1.00), at 9600 baud, 8N1,
with RX and hardware flow control disabled. Use ``DT_ALIAS(eeg_uart)`` to
access it. UART idle high releases the trigger circuit; no application data
is transmitted yet. USB CDC remains the console.

Other board peripherals remain disabled. Application-core GPIO hogs
hold regulator MODE low and chip selects high.
FPGA shared IO/JTAG/control pins otherwise retain
their reset input state and existing external pulls. GPIO hogs apply during
driver initialization, not during reset; hardware pulls still govern reset.
Remove the corresponding hog entries when runtime drivers take ownership.

Before enabling a peripheral, enable its bus and child nodes, configure its
driver, and validate electrical/protocol assumptions. In particular:

* BQ27441 needs Bridge-specific battery design parameters before enabling.
* ADXL343 and FM25CL64B bindings describe wiring only; no driver is selected.
* FPGA SPI framing, mode, IRQ and shared-pin ownership remain to be defined.
* NanoSD is an SD-over-SPI disk, not SPI NOR. No filesystem or autoformat is set.
* USB CDC ACM uses native USB pins. No hardware debug UART is routed.
* P0.11 is reserved for SWO, not storage hardware CS; SPIM4 DCX is unused.
* The fitted LF crystal uses the 9 pF internal-load setting, without bypass.
  Confirm assembled J2 stub removal and qualify startup on hardware.
* HFXO retains the SoC default pending confirmation of NORA's module-specific
  capacitor requirements. ECG/PPG's bare-SoC capacitor setting is not copied.

Flash/SRAM use the SoC's full ranges for initial bring-up. No MCUboot, BLE HCI,
shared SRAM/IPC partition, or application-specific storage layout is imposed.
The disabled peripherals do not become usable merely by enabling their buses.

Pin verification
----------------

All GPIO assignments were checked against U1 (instance INS981) in
``bridge-docs/BRIDGE.EDF``. SHA-256:
``15916cce1406d084237a1656762014e883736687ff1ba85d6f88f17c59c3e648``.
The handoff and EDIF agree; NORA pad names were resolved to nRF port/pin names.
The EDIF names the LED supply signal ``&5V_EN``.

Devicetree preprocessing and binding validation are static checks, not evidence
of successful firmware operation. Hardware bring-up results must be recorded
separately.
