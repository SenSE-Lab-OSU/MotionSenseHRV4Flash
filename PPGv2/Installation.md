# PPGv2 installation/build note (Phase 5 draft)

Use `C:\ncs\SenSEv2.9.3` with
`C:\ncs\toolchains\b620d30767`; the project-managed NCS checkout contains
required patches. For reproducible builds, use the shared serial wrapper shown
in the repository README with `PPGv2` and `ppgv2/nrf5340/cpuapp`. Do not use a
DK overlay or silently select a newer NCS release. The application, MCUboot,
HCI IPC, signed application, and merged artifacts are all produced by the
sysbuild output directory selected with `-BuildDirectory`.

**Install Guide**

Welcome to the MSense4 Development guide! For those that not yet developed with Zephyr or the NrfConnect Sdk, this guide exists as a tool to help you find your ropes around this project, as our build system has become complex. This is partly due to the way that Zephyr wants software to be built, and partly because our embedded software in it of itself is complex, requiring multiple partitions, dual core functionality for bluetooth low energy, and lots of other sensors.


**Steps to install**:

1. Clone this repository. 
2. Follow the instructions to install the Nrf Connect SDK (https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation/install_ncs.html ). It should instruct you to install nRF Util, visual studio code, their corresponding vs code plugins, and SEGGER J-Link. Make sure you have all these things. The SDK version to install is 2.9.3.
3. Preserve the project-specific v2.9.3 SDK changes documented in `changelist.txt`. PPGv2 does not use LFXO bypass; the bypass patch is required by ECGv0's external-clock board.
4. Using visual studio, click on the nrf connect tab. Make sure you've selected the v2.9.3 sdk. If everything has installed properly and the SDK is chosen, you should see a button that says 'Open an Existing Application' within the 'Welcome' Tab. After pressing this, navigate to the repository folder `PPGv2` with the opened file explorer.
5. You should now see the project in the 'Applications tab, and the project files will appear in visual studio explorer. If you have not already, navigate back to the nrf connect tab, and in the 'Applications' tab, click the '+ Add build configuration'. You should see a new tab pop up in the visual studio code application.
6. In the build configuration, select `ppgv2/nrf5340/cpuapp` as the board target. Use `prj.conf` as the application configuration file; the repository board root is supplied by the application and sysbuild CMake files.

7. now, go back to the nrf connect tab, and in the 'actions' tab, click the build button. The application should now build correctly, and if you have a device connected to a JLink, you will be able to use the flash command to upload it.


The Guide for all Bluetooth Commands and how to use the device  is avalible at https://docs.google.com/document/d/1GqLLecDG5mTC0zdFGCEx2AJov6y5_ytyBaz1Vn2olMI/edit?usp=sharing
