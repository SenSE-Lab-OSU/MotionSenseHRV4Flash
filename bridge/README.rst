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
It logs through RTT; there is no DK LED library or UART pin assignment.
The radio protocol still needs to be matched to the transmitting devices.
There is no inter-core packet transport yet.

Board peripherals are described but disabled. Application-core GPIO hogs
hold LED gates, LED supply enable and regulator MODE low, chip selects high,
and EEG_TRIG high/released. FPGA shared IO/JTAG/control pins otherwise retain
their reset input state and existing external pulls. GPIO hogs apply during
driver initialization, not during reset; hardware pulls still govern reset.
Remove the corresponding hog entries when runtime drivers take ownership.
Keep EEG released during that transition.

Before enabling a peripheral, enable its bus and child nodes, configure its
driver, and validate electrical/protocol assumptions. In particular:

* BQ27441 needs Bridge-specific battery design parameters before enabling.
* ADXL343 and FM25CL64B bindings describe wiring only; no driver is selected.
* FPGA SPI framing, mode, IRQ and shared-pin ownership remain to be defined.
* NanoSD is an SD-over-SPI disk, not SPI NOR. No filesystem or autoformat is set.
* USB has no class selected. No debug UART is routed.
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

This change has not been built or flashed. Devicetree preprocessing and binding
validation are static checks, not evidence of successful firmware operation.
