# MotionSenseHRV3 Bioimpedence with Lab Streaming Layer and Bluetooth Low Energy (BLE)


Welcome to the MotionSenseHRV3 Bluetooth Data Collection Repository! This Repo contains projects for MotionSense SoC Embedded Data Collection Software(in multiple formats), as well as a python Windows/Mac/Linux data collection gui application that works as a Standalone Bluetooth Data Reciver and Lab Streaming Layer interface.

# Getting Started

To get started, either clone this repository or download it as a zip file (located at the top right of github). After extracting, you can run the bluetooth collection application by following the guidelines specified in BluetoothReciever/ReadMe.md. Note that you will need a MotionSense application that is charged and turned on in order for collection to begin.

## Build with docker

1. Clone the repository

```
git clone https://github.com/SenSE-Lab-OSU/MotionSenseHRV4Flash.git
```

2. (optional) checkout `vanilla_4nand` branch

```
git checkout vanilla_4nand
```

3. If `docker` is not installed, follow [the installation guide](https://docs.docker.com/engine/install/) to finish setup. 

4. Setup docker container

```
cd MSenseDevice
docker run -it -v ${PWD}/:/workdir/project agilemelon/msense_nrf bash
```

5. Build the docker container
```
cd project
west build -p always -b nrf5340dk_nrf5340_cpuapp
```

6. After compile is sucessfully completed

- Get the hex or DFU file at `MSenseDevice/build/zephyr/merged_domains.hex`
- Or, DFU file at `MSenseDevice/build/zephyr/dfu_application.zip`

