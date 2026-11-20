# MotionSenseHRV4 


Welcome to the MotionSenseHRV4 Firmware and Data Extraction Repository! This Repo contains the firmware for MotionSense SoC, as well as a python Windows/Mac/Linux data extraction tool. To learn more about either application, please visit the desired folder.

## PPG ship mode

Stop collection and disconnect USB power, then write one byte, decimal `69` (`0x45`), to the BLE reset characteristic `da39c934-1d81-48e2-9c68-d0ae4bbd351f`. The PPG closes its files and stays powered with BLE, both status LEDs, and the PPG light supply off. Connecting a USB host or charger triggers a system reset and restores normal operation. If USB already supplies power when the write arrives, the PPG ignores it.

