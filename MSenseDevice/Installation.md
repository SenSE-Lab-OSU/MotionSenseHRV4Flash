**Install Guide**

Welcome to the MSense4 Development guide! For those that not yet developed with Zephyr or the NrfConnect Sdk, this guide exists as a tool to help you find your ropes around this project, as our build system has become complex. This is partly due to the way that Zephyr wants software to be built, and partly because our embedded software in it of itself is complex, requiring multiple partitions, dual core functionality for bluetooth low energy, and lots of other sensors.


**Steps to install**:

1. Clone this repository. 
2. Follow the instructions to install the Nrf Connect SDK (https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation/install_ncs.html ). It should instruct you to install nRF Util, visual studio code, their corresponding vs code plugins, and SEGGER J-Link. Make sure you have all these things. The SDK version to install is 2.5.3. 
3. Now we must apply project specific patches to the SDK. Open a command prompt / git bash / etc. Navigate to the SDK install directory, e.g., `cd C:\ncs\v2.5.3`
4. Run `git apply /path/to/this/repo/MSenseDevice/msense_ncs_v2.5.3.patch`
5. Using visual studio, click on the nrf connect tab. Make sure you've selected the v2.5.3 sdk. If everything has installed properly and the SDK is chosen, you should see a button that says 'Open an Existing Application' within the 'Welcome' Tab. After pressing this, navigate to the repository folder 'MSenseDevice' with the opened file explorer.
6. You should now see the project in the 'Applications tab, and the project files will appear in visual studio explorer. If you have not already, navigate back to the nrf connect tab, and in the 'Applications' tab, click the '+ Add build configuration'. You should see a new tab pop up in the visual studio code application.
7. In the build configuration, select the board to be nrf_5340dk_nrf_5340_cpuapp. Everything else should be fine as default, the project config should be prj.conf and the board file will be auto set.

8. now, go back to the nrf connect tab, and in the 'actions' tab, click the build button. The application should now build correctly, and if you have a device connected to a JLink, you will be able to use the flash command to upload it.



The Guide for all Bluetooth Commands and how to use the device  is avalible at https://docs.google.com/document/d/1GqLLecDG5mTC0zdFGCEx2AJov6y5_ytyBaz1Vn2olMI/edit?usp=sharing

