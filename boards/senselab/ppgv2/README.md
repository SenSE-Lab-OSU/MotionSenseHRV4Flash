# SenseLab PPGv2 board

This NCS v2.9.3 Hardware Model v2 board supports
`ppgv2/nrf5340/cpuapp` and `ppgv2/nrf5340/cpunet`.

The application core has a MAX86141 on SPI3, an ICM-20948 on SPI2, a
BQ27441 gauge for the 300 mAh / BQ25060 design, one SPI NOR device, and four
MT29 NAND devices. The PPGv2 PCB has no user button. Its 32.768 kHz source is
a crystal with the retained 9 pF LFXO capacitor configuration; LFXO bypass is
disabled. Both cores retain the common HFXO capacitor setting and use LDO mode
for VREGMAIN and VREGRADIO while VREGH is disabled.

The `icm20948` node is intentionally descriptive. PPGv2 keeps its existing
direct SPI IMU implementation, so the board does not enable a competing Zephyr
ICM-20948 driver.
