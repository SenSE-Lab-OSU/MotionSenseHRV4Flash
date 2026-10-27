# BRIDGE firmware handoff

Prepared 2026-10-27 for a separate firmware-development session.

## 1. Current status and scope

The owner reports that the bridge is **assembled and has passed smoke tests**. This is the starting point for firmware development. No firmware functionality, FPGA programming, RF link, filesystem, battery calibration, or EEG-device interoperability is established by that statement. Do not repeat hardware redesign or procurement work unless a functional test exposes a problem.

This is a battery-powered RF hub built around a u-blox NORA-B106-00B (nRF5340), with a small Lattice FPGA for event timestamps, persistent storage, USB, an accelerometer, buttons, indicator LEDs, and an EEG trigger connection.

The other agent has access to the EDIF. Use this handoff to accelerate development, but check the actual schematic before assigning or changing pins. No SDK release, custom-board target, firmware repository, existing FPGA image, RF protocol, or host command protocol has been selected in this handoff.

## 2. Sources and precedence

- Current schematic: `D:/bridge/CAD/BRIDGE.EDF`, exported by OrCAD Capture 22.1.
- SHA-256 verified at handoff: `15916cce1406d084237a1656762014e883736687ff1ba85d6f88f17c59c3e648`.
- Matching parsed inventory: `tmp/edif-comparison-oct20/current.json`. This was checked against the current EDIF hash, rather than assumed current.
- Approved procurement BOM: `BOM.xlsx`; original `BRIDGE.BOM` is older.
- Manufacturer references: `datasheets/`; additional NORA documents are under `tmp/module-research/`, with extracted power-review material under `tmp/nora-power-review/`.
- EEG counterparty document: `eeg-jack-tech.pdf`; interface description: https://support.mbraintrain.com/how-tos/ttl-wired-triggering .

Use the EDIF for connectivity and the approved BOM for component substitutions. Actual assembled population/rework takes precedence over either. Historical review reports are dated snapshots, not the current netlist. In particular, the earliest EEG review predates the inverter and swap links and gives an obsolete MCU trigger polarity.

Known approved BOM overrides not reflected in this EDIF:

| References | EDIF | Approved BOM |
|---|---|---|
| R1–R6 | 4.7 kΩ | 5.1 kΩ |
| R37, NanoSD clock series resistor | 120 Ω | 33 Ω |
| C11 | 470 nF | Same value, 0603, 25 V X7R, ±5%, AVX 06033C474JAT2A |
| C13/C15/C21/C23/C24/C26 | 1 µF | Same value, 0805, 25 V X7R, ±5%, KEMET C0805C105J3RECTU |

Nominal DNP population: C9, C10, R29, R44, R49, R50, U3. Confirm assembled exceptions. J2 is bare measurement pads; U2 is a Tag-Connect footprint, not a fitted IC.

## 3. Hardware and power architecture

| Ref | Part | Role |
|---|---|---|
| U1 | NORA-B106-00B | nRF5340 MCU/radio, integrated antenna |
| U6 | LCMXO2-256HC-5SG32C | Small MachXO2 FPGA, event timer and SPI slave |
| U15 | MKDN064GCL-AA | 64 Gbit (8 GB nominal) managed NanoSD storage |
| U4 | FM25CL64B-GTR | 64 Kbit (8 KiB) SPI FRAM |
| U16 | ADXL343BCCZ-RL7 | I2C accelerometer |
| U9 | BQ27441DRZT-G1A | I2C fuel gauge |
| U11 | TPS82693SIPT | 2.85 V system regulator |
| U12 | BQ25060DQCR | Battery charger/power path |
| U13 | REG71050DDCR | Enabled 5 V LED charge pump |
| U5/U7 | FDC6305N | LED and EEG low-side MOSFET switches |
| U17/U18 | SN74LVC1G04DBVR | TCXO measurement buffer / EEG inverter |

One Li-ion cell feeds the charger/power-path circuit. U11 converts `V_BAT` to `VDD = 2.85 V`. S1 directly controls U11 EN; firmware cannot keep itself running after S1 switches the regulator off.

NORA A7 and B7 (VDD) and A8 (VDDH) all connect to VDD: this is externally supplied normal-voltage operation. Do not configure VREGH to generate the 2.85 V rail. NORA B9 (VBUS) receives `VUSB` separately. USB insertion does not imply the MCU is powered while S1 is off.

`POWER_MODE` controls U11 MODE, not its enable. The default external pulldown selects the low state; confirm MODE semantics in `tps82693.pdf` before changing it. `5V_EN` controls U13 and should start low. VLED powers only the fun LEDs; they are switched through MOSFETs, not directly sunk by MCU pins.

Charging is hardware-controlled, not I2C-controlled. U12 EN is grounded and CHG is unconnected. D4 is a hardware LED fed from the charger LDO through R27; it is **not a firmware-controlled charge-status LED**. The fixed R28/R30 TS divider does not measure battery temperature. Do not expose a claim of battery-temperature protection in firmware. R26 is 6.8 kΩ; account for system load sharing the charger input-current budget.

## 4. Verified MCU signal map

GPIO names below are nRF port.pin numbers, not module pad coordinates. The full connected-pin table is appended at the end.

| Function / net | GPIO | NORA pad | Configuration / ownership |
|---|---|---|---|
| SD/FRAM SCK, SD_CLK | P0.08 | B1 | Shared SPI output; reserved for storage/SPIM4 |
| SD/FRAM MOSI, SD_CMD | P0.09 | C2 | Shared SPI output |
| SD/FRAM MISO, SD_D0 | P0.10 | C1 | Shared SPI input |
| SD chip select, SD_D3 | P0.12 | A2 | GPIO CS, idle high |
| FRAM_CS | P0.13 | D2 | GPIO CS, idle high |
| SCL | P1.02 | B4 | I2C clock |
| SDA | P1.03 | A3 | I2C data |
| FPGA SCK_MCU | P0.19 | F3 | Through R9, 33 Ω, to FPGA pin 9 |
| FPGA_MOSI | P0.20 | A5 | FPGA pin 17 |
| FPGA_MISO | P0.21 | E3 | Through R8, 33 Ω, from FPGA pin 10 |
| FPGA_CS_N | P0.22 | C8 | FPGA pin 16; idle high |
| FPGA_TCK | P1.04 | G4 | GPIO JTAG output, external pulldown |
| FPGA_TMS | P1.05 | E9 | GPIO JTAG output, external pullup |
| FPGA_TDI | P1.06 | G1 | GPIO JTAG output, external pullup |
| FPGA_TDO | P1.07 | F7 | GPIO JTAG input |
| FPGA_IRQ_N | P1.08 | G5 | Intended FPGA-to-MCU interrupt; configure input |
| FPGA_PROGRAM_N | P1.09 | G7 | Idle released/high, external pullup |
| FPGA_DONE | P1.10 | J7 | Input, external pullup |
| BTN_A | P1.11 | H9 | Shared MCU/FPGA button input |
| BTN_B | P1.12 | G8 | Shared MCU/FPGA button input |
| FPGA_IO_0 | P1.13 | G9 | Shared link to FPGA pin 11; direction must be agreed |
| FPGA_IO_1 | P1.14 | F8 | Shared link to FPGA pin 12; direction must be agreed |
| BQ_GPOUT | P1.15 | F9 | Fuel-gauge interrupt/input, 10 kΩ pullup |
| ACCEL_INT1 | P0.17 | F1 | Accelerometer interrupt input |
| ACCEL_INT2 | P0.16 | F2 | Accelerometer interrupt input |
| LED_PINK | P0.04 | D8 | High turns on MOSFET |
| LED_VIOLET | P0.05 | E8 | High turns on MOSFET |
| LED_CYAN | P0.06 | E7 | High turns on MOSFET |
| 5V_EN | P0.14 | E2 | High enables LED supply |
| POWER_MODE | P0.15 | D1 | Regulator mode control |
| EEG_TRIG | P1.00 | A4 | **High releases output; low asserts it** |
| SWO | P0.11 | B3 | Debug; do not assign storage hardware CS here |

Unused GPIOs in this EDIF include P1.01 and P0.02/.03/.07/.18/.23–.31. Some have alternate NFC/analog/peripheral functions. Unconnected is not the same as physically accessible; check the board before proposing a debug UART.

## 5. Peripheral details and initialization

### Storage SPI

NanoSD and FRAM share SCK/MOSI/MISO, with separate CS lines. Keep both CS high before enabling clocks, and serialize transactions with a shared bus lock. Use per-device SPI configurations and keep the unselected device deselected during initialization and error recovery.

SPIM4 was deliberately reserved for this bus; **do not move FPGA SPI onto it**. At high-speed SPIM4 operation the dedicated pin configuration needs explicit attention. P0.11 is SWO, not storage CS; P0.12 is SD CS, not an available DCX pin. Avoid enabling peripheral hardware CS/DCX functions that conflict with those uses. Begin at low speed and verify configuration against the chosen SDK release before increasing speed.

NanoSD is an SD-protocol device, not raw SPI NOR/NAND. Use SD-over-SPI initialization, card identification and block transfers; do not issue NOR JEDEC commands and treat it as a flash array. Start with the SD initialization clock limits, then raise clock only after stable identification/read/write. R37 only affects the NanoSD clock branch. Do not demand maximum throughput during bring-up.

FRAM is only 8192 bytes. Reserve a documented scratch region before tests; do not overwrite existing settings. Verify opcodes, address width, write-enable behavior and SPI mode in the Infineon datasheet. WP and HOLD are tied high. It could store configuration, counters and recovery metadata, but that storage allocation is a proposal, not an existing format.

Provide bounded SPI timeouts, error counts, and explicit card-busy handling. S1 can remove power abruptly: design logging for incomplete records, interrupted writes and recovery. No separate MCU-controlled NanoSD power switch is wired. FRAM access and SD activity need arbitration even if logging occurs in different tasks.

### I2C: accelerometer and gauge

P1.02/P1.03 have external 2.2 kΩ pullups to VDD. Bring up at 100 kHz and qualify 400 kHz; do not assume the bus may run at 1 MHz just because Nordic supports it.

- **ADXL343: 7-bit address 0x53**, since ALTADDRESS is grounded and CS is tied high. It is wired for I2C, not SPI. Both interrupts reach the MCU. Configure range, output rate, FIFO and interrupt polarity explicitly. Configure in standby before measurement. Choose normal versus low-power operation according to noise/power requirements, not by assumption.
- **BQ27441: 7-bit address 0x55**. Do not pass 0xAA/0xAB as 7-bit addresses. GPOUT has an external pullup. Read identity/status/voltage first; configure alert behavior intentionally. Confirm battery capacity, chemistry compatibility, terminate voltage and other parameters with the owner before initialization/calibration writes. The sense resistor is 10 mΩ. A plausible reported voltage is not proof that state of charge is calibrated.

### Buttons and LEDs

SW1/SW2 connect BTN_A/BTN_B to ground through R39/R40 (1 kΩ). They are active-low shared inputs, not MCU outputs. Configure suitable pullups and debounce; no dedicated button pullup resistor appears in the extracted netlist. Decide whether MCU or FPGA owns pull bias and whether the FPGA records raw edges or debounced events.

LED controls have external 47 kΩ gate pulldowns. Start all three low and 5V_EN low. Enable VLED before displaying a pattern. PWM can be used later if desired. Disable gates and supply for the low-power baseline. D1 violet, D2 pink, D3 cyan use 330 Ω current-limiting resistors. D1/D2/D3 cathodes are pin 1 in the EDIF; do not generalize that numbering to D4.

### USB and debugging

USB DP/DM connect through U8 to J1; there is no USB-UART bridge. USB CDC or another device class requires actual firmware configuration. Begin with SWD/RTT diagnostics if supported by the selected environment rather than repurposing occupied pins.

U2 Tag-Connect mapping: 1 VDD reference, 2 SWDIO, 3 RESET_N, 4 SWDCLK, 5 GND, 6 SWO. The intended probe is a Nordic DK using TC2030-CTX. Verify target power/reference and debug routing before attaching; do not assume the cable supplies target power.

## 6. Clocks and FPGA work

### Two independent clock systems

- MCU LFCLK: Y1 ECS-.327-9-34B-TR, 32.768 kHz, 9 pF load rating, between XL1=P0.00/C6 and XL2=P0.01/B6. U3 SiT1566 is the **DNP alternate**, not the selected clock source. The user planned to cut off the J2 stub near the crystal when using Y1; confirm this was done. Configure the supported crystal/load arrangement based on NORA/nRF documentation. Do not select external-clock bypass merely because an alternate oscillator footprint exists.
- FPGA reference: Y2 ECS-TXO-3225MV-100-TR, **10 MHz**, powered by VDD, output to U6 pin 4 (PL5C). Its enable/tristate pin is unconnected, so no MCU GPIO power-saving control is available. This 10 MHz signal is not a direct MCU clock input.

U17 buffers/inverts the TCXO for J3 SMA through R41=33 Ω. J3 is measurement-only. Its waveform is inverted relative to the FPGA clock; it is not a separate reference input. Do not assume a 50 Ω terminated measurement load is appropriate for this logic buffer.

### FPGA requirements agreed with owner

- Global counter exposed over SPI.
- Timestamp IO events and identify which event occurred.
- 100 ns resolution (one 10 MHz tick) is more than sufficient.
- Approximately four inputs are enough. The owner accepted fewer than 64 buffered events.
- Device fixed as LCMXO2-256HC-5SG32C, not a larger FPGA.
- FPGA programming is to be driven by MCU GPIO JTAG.

64-bit timestamps and a 64-entry FIFO were discussion starting points, **not guaranteed to fit**. Establish actual logic/storage resources and synthesize an achievable design. Do not assume dedicated block RAM or promise a deep FIFO without checking this exact density. Counter width, FIFO depth, event encoding, SPI command framing, CPOL/CPHA and IRQ behavior are still to be defined.

Suggested protocol requirements: identity/version/capabilities, coherent counter snapshot, FIFO count, explicit overflow flag/count, event read/pop with defined atomicity, interrupt acknowledgement, input edge masks and reset semantics. Account for partial SPI transactions, repeated reads, reset during transfers, and counter rollover.

Event candidates are BTN_A/B (U6 pins 13/14) and FPGA_IO_0/1 (pins 11/12). The latter also connect to MCU GPIOs; they are not independent external connector inputs. Define ownership and idle levels before driving them. Synchronize asynchronous inputs and SPI signals appropriately; document synchronization latency and uncertainty. Do not equate 100 ns tick size with 100 ns absolute timing accuracy.

FPGA_IRQ_N is U6 pin 20. PROGRAM_N pin 25, DONE pin 23, JTAG_EN pin 26. JTAG_EN is **hard-pulled high by R7=1 kΩ**, with no MCU control. JTAG: TCK 30, TMS 29, TDI 32, TDO 1.

The board powers FPGA VCC/VCCIO from 2.85 V. The owner deliberately accepted an exception to nominal LVCMOS25 voltage guidance. Preserve and explicitly record that engineering exception; do not claim guaranteed datasheet-compliant operation or silently redesign the rail. Review exact electrical limits and tool I/O settings when implementing the bitstream.

Start JTAG with a read-only identification/chain check. Use a documented MachXO2 programming flow, verify part/package/ID and image integrity, and check completion/status. Establish whether a working image already exists before erasing configuration. Choose persistent configuration versus volatile development programming intentionally. MCU boot must tolerate an unprogrammed or nonresponsive FPGA.

## 7. EEG trigger / possible UART output

Current chain:

`P1.00 EEG_TRIG -> U18 inverter -> EEG_TRIG_INV -> U7 gate B`.

U7 source B is ground; drain B is JACK_TIP. R48=47 kΩ pulls EEG_TRIG to VDD; R47=47 kΩ pulls the gate node to ground. Therefore:

| MCU P1.00 | Inverter / MOSFET | Jack signal |
|---|---|---|
| High / normal pulled-up idle | Gate low, MOSFET off | Released, external equipment must provide pullup |
| Low | Gate high, MOSFET on | Pulled toward board ground |

Set the output latch high before enabling output mode to avoid an unintended pulse. Restore high before disabling a UART or entering sleep. A normal idle-high UART TX is the intended polarity with the added inverter; do not automatically add software inversion. Two inversions yield the normal UART sense at the externally pulled-up jack. Baud rate, framing, accepted trigger pulse widths and command meaning remain unconfirmed with the counterparty.

J5 physical contacts: pin 1 sleeve=S, pin 2 tip=T, pin 3 ring=R.

- Default: R46=0 Ω ties tip to JACK_TIP; R45=0 Ω ties ring to ground.
- Alternate swap: R49 ties tip to ground; R50 ties ring to JACK_TIP. Both are DNP by default.
- Fit one configuration only; never fit all four links.
- R44 is DNP; sleeve is not connected to board ground.

This is a **ground-referenced open-drain output, not an isolated dry contact**. No EEG UART receive path is wired. Tip/ring polarity relative to the EEG unit was not conclusively established by the prior discussion; the swap footprints exist for this reason. Confirm actual population, jack reference and counterparty voltage before testing connected equipment. Do not infer safety or isolation from the connector or MOSFET. Use a dummy pullup/test load and scope first. The CME TRS-25 cable is a two-pack of 10 cm 2.5 mm TRS male-to-male cables; verify end-to-end contact continuity during bring-up.

## 8. Firmware organization and staged acceptance

The following is a proposed implementation plan, not an already approved wire protocol:

1. **Inventory/environment:** locate any existing firmware and FPGA project; establish the installed NCS release, custom-board definition, debugger and actual assembly variants. Record smoke-test measurements if available.
2. **Minimal boot:** SWD access, reset cause, persistent logs/RTT; establish safe GPIO states before starting threads. LEDs off, both storage CS high, EEG released, FPGA shared lines as inputs.
3. **Clock and power baseline:** verify LF crystal startup and idle current. Separate debugger-induced current from standalone measurements. Keep LEDs, radio and storage traffic inactive initially.
4. **Peripheral identity:** I2C address/identity checks, gauge voltage/status, accelerometer samples/interrupts, FRAM bounded scratch test. Missing optional parts must produce useful diagnostics rather than a boot loop.
5. **Storage:** identify NanoSD, capacity and sector access; then filesystem/logging with a versioned record format, checksums or equivalent corruption detection, and interrupted-write recovery. Do not autoformat unknown media without approval.
6. **FPGA:** identify via JTAG; program a minimal test image; verify 10 MHz counter scaling, atomic reads, timestamped edges, FIFO ordering and overflow. Compare against scope/counter observations.
7. **EEG:** verify released state at reset/boot/sleep, low pulse duration and UART waveform on a dummy load. Test the real EEG unit only after its interface is confirmed.
8. **RF and host:** implement the chosen protocol, peer discovery/reconnect, buffering and host commands. The RF protocol, node count, throughput and latency target still need agreement.
9. **Integration:** exercise simultaneous RF reception, SD writes, FPGA events, I2C and USB. Measure lost events, queue high-water marks, worst-case latency and rail behavior. Test low battery, card timeout, FPGA failure, USB reconnect and abrupt power loss.

Keep separate modules for board control, FPGA transport/protocol, storage, gauge, accelerometer, EEG output, RF transport and diagnostics. Prefer interrupt-driven notification and deferred processing; avoid SD writes or long SPI operations in ISRs. Keep timestamps in FPGA tick units with an explicit clock frequency and session identifier. A relationship between RF packet time, MCU time and FPGA time requires a defined synchronization method; it is not automatically established by reading a counter over SPI.

## 9. Questions to settle early

- Existing firmware/bitstream repository and installed SDK/toolchain version?
- Exact assembled population, battery capacity/model, and whether both prototype boards have U9/J3?
- RF protocol, participating devices, event format and required throughput/latency?
- Host interface: USB CDC, custom USB, BLE, or another transport?
- FPGA counter width, actual FIFO depth and event-input ownership?
- Logging format, filesystem, retention, and policy for low battery and storage-full conditions?
- EEG electrical reference, pulse protocol or UART baud/framing, and populated swap links?
- Whether J2's LF-crystal branch was cut as planned?

## 10. Tooling and operational constraints for the next agent

Use Nordic MCP as the primary source for Nordic/NCS/Zephyr documentation. Pin version-specific configuration to the chosen installed release; do not invent Kconfig names or copy DK pin assignments into this custom board. The application core is the natural location for board services; the network-core image depends on the selected radio stack.

Do not run a full NCS build by default. The workspace instruction requires user approval or explicit build authorization. When using the shared wrapper, read `D:/senselab-tools/vscode-wrapper/README.md` first and supply the actual BRIDGE application directory and board target. Paths for other projects in global instructions are not BRIDGE targets. Do not flash, recover, mass-erase or alter protection as an incidental test.

Nordic MCP trusted documentation retrieved for this handoff:

- https://docs.nordicsemi.com/r/bundle/ps_nrf5340/page/chapters/pin.html — dedicated high-speed SPI/TWI pins and drive requirements.
- https://docs.nordicsemi.com/r/bundle/ps_nrf5340/page/spim.html-topic — SPIM4 versus other SPI-instance capabilities.
- https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrf/app_dev/device_guides/nrf53/features_nrf53.html#dual-core-architecture — core roles. This is a rolling latest reference, not a selected project SDK version.

Deliver reproducible source, board definition, pin ownership documentation, a versioned MCU/FPGA protocol, bounded diagnostics, and a short bring-up test report separating measured results from assumptions. Keep all hardware changes explicit.


## Appendix A. Full connected NORA pin map from matching EDIF

NC entries are omitted. Leading EDIF ampersands are normalized for readability. Module reserved/NC pads are not GPIO.

| Module pad | EDIF pin function | Net |
|---|---|---|
| A1 | VSS | GND |
| A2 | P0_12_TRACECLK_DCX | SD_D3 |
| A3 | P1_03_TWI | SDA |
| A4 | P1_00 | EEG_TRIG |
| A5 | P0_20 | FPGA_MOSI |
| A7 | VDD | VDD |
| A8 | VDDH | VDD |
| B1 | P0_08_TRACEDATA3_SCK | SD_CLK |
| B4 | P1_02_TWI | SCL |
| B3 | P0_11_TRACEDATA0_CSN | SWO |
| B6 | P0_01_XL2 | XL2 |
| B9 | VBUS | VUSB |
| C1 | P0_10_TRACEDATA1_MISO | SD_D0 |
| C2 | P0_09_TRACEDATA2_MOSI | SD_CMD |
| C6 | P0_00_XL1 | XL1 |
| C8 | P0_22 | FPGA_CS_N |
| C9 | USB_DP | USB_DP |
| D1 | P0_15_IO2 | POWER_MODE |
| D2 | P0_13_IO0 | FRAM_CS |
| D8 | P0_04_AIN0 | LED_PINK |
| D9 | USB_DM | USB_DN |
| E2 | P0_14_IO1 | 5V_EN |
| E3 | P0_21 | FPGA_MISO |
| E8 | P0_05_AIN1 | LED_VIOLET |
| E7 | P0_06_AIN2 | LED_CYAN |
| E9 | P1_05 | FPGA_TMS |
| F1 | P0_17_SCK | ACCEL_INT1 |
| F2 | P0_16_IO3 | ACCEL_INT2 |
| F3 | P0_19 | FPGA_SCK_MCU |
| F8 | P1_14 | FPGA_IO_1 |
| F7 | P1_07 | FPGA_TDO |
| F9 | P1_15 | BQ_GPOUT |
| G1 | P1_06 | FPGA_TDI |
| G4 | P1_04 | FPGA_TCK |
| G5 | P1_08 | FPGA_IRQ_N |
| G7 | P1_09 | FPGA_PROGRAM_N |
| G8 | P1_12 | BTN_B |
| G9 | P1_13 | FPGA_IO_0 |
| H2 | SWDIO | SWDIO |
| H9 | P1_11 | BTN_A |
| J2 | SWDCLK | SWDCLK |
| J3 | R_E_S_E_T_ | RESET |
| J7 | P1_10 | FPGA_DONE |
| B7 | VDD_0 | VDD |
| A9 | VSS_0 | GND |
| B2 | VSS_1 | GND |
| B8 | VSS_2 | GND |
| E4 | VSS_3 | GND |
| E5 | VSS_4 | GND |
| F4 | VSS_5 | GND |
| F5 | VSS_6 | GND |
| K2 | VSS_7 | GND |
| K3 | VSS_8 | GND |
| K5 | VSS_9 | GND |
| K7 | VSS_10 | GND |
| K8 | VSS_11 | GND |
| L1 | VSS_12 | GND |
| L9 | VSS_13 | GND |
| M1 | VSS_14 | GND |
| M2 | VSS_15 | GND |
| M8 | VSS_16 | GND |
| M9 | VSS_17 | GND |
