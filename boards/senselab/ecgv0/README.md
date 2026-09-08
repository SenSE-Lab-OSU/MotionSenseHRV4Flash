# SenseLab ECGv0 board

This NCS v2.9.3 Hardware Model v2 board supports
`ecgv0/nrf5340/cpuapp` and `ecgv0/nrf5340/cpunet`.

The application core has a MAX30001 ECG AFE on SPI3, with INTB and INTB2,
and an ICM-20948 on SPI2 with the common INT/FSYNC wiring. It has a BQ27441
gauge for the 200 mAh / BQ21040 design, one SPI NOR device, and two MT29 NAND
devices. The active-low P0.01 user button is ECGv0-only and supports its
retained ship-mode behavior.

ECGv0 receives an external 32.768 kHz clock through the LFXO bypass path. The
board defconfigs therefore require the repository's documented NCS v2.9.3
LFXO-bypass patch. Both cores retain the common HFXO capacitor setting and use
LDO mode for VREGMAIN and VREGRADIO while VREGH is disabled.
