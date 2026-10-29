Bridge FPGA bring-up
===================

Use target ``bridge/nrf5340/cpuapp`` with the project-managed NCS 2.9.3.
Board definitions and bindings come from the repository root. This application
enables SPI3 and the FPGA node; the shared board keeps them disabled by default.
The USB CDC echo sample is retained for now. No FPGA programmer is implemented yet.

Pin assignments below were checked against ``bridge-docs/BRIDGE.EDF``:

* SPI3: SCK P0.19, MOSI P0.20, MISO P0.21, active-low CS P0.22.
  The 1 MHz limit is provisional; confirm mode and protocol against the FPGA
  image before sending commands. Storage remains on disabled SPI4.
* GPIO JTAG: TCK P1.04 (output, idle low), TMS P1.05 and TDI P1.06
  (outputs, idle high), TDO P1.07 (input). JTAG_EN is externally pulled high.
* PROGRAM_N P1.09: active-low open-drain output, initially released using
  ``GPIO_OUTPUT_INACTIVE``. DONE P1.10 and active-low IRQ P1.08 are inputs.
* Shared IO0 P1.13 and IO1 P1.14 must remain inputs until image ownership is
  defined. Buttons P1.11/P1.12 remain disabled.

The programmer should obtain GPIO specs from ``DT_ALIAS(fpga)``. Enabling that
node does not initialize its GPIOs: firmware must configure directions and idle
levels explicitly. Do not use ``DEVICE_DT_GET`` on the FPGA node; it has no
Zephyr device driver. SPI can use ``SPI_DT_SPEC_GET`` on the same node.

The application disables only the FPGA CS hog so SPI3 can own that pin.
LED gates and the 5 V rail retain their low boot holds. JTAG programming does
not require the LED rail. Begin programmer bring-up with a read-only JTAG ID
check; image format and volatile versus persistent programming remain to be set.
