**Install Guide**

Welcome to the MSense4 Development guide! For those that not yet developed with Zephyr or the NrfConnect Sdk, this guide exists as a tool to help you find your ropes around this project, as our build system has become complex. This is partly due to the way that Zephyr wants software to be built, and partly because our embedded software in it of itself is complex, requiring multiple partitions, dual core functionality for bluetooth low energy, and lots of other sensors.


**Steps to install**:

1. Clone this repository. 
2. Follow the instructions to install the Nrf Connect SDK (https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation/install_ncs.html ). It should instruct you to install nRF Util, visual studio code, their corresponding vs code plugins, and SEGGER J-Link. Make sure you have all these things. The SDK version to install is 2.5.3. 
3. Using visual studio, click on the nrf connect tab. Make sure you've selected the v2.5.3 sdk. If everything has installed properly and the SDK is chosen, you should see a button that says 'Open an Existing Application' within the 'Welcome' Tab. After pressing this, navigate to the repository folder 'MSenseDevice' with the opened file explorer.
4. You should now see the project in the 'Applications tab, and the project files will appear in visual studio explorer. If you have not already, navigate back to the nrf connect tab, and in the 'Applications' tab, click the '+ Add build configuration'. You should see a new tab pop up in the visual studio code application. 
5. In the build configuration, select the board to be nrf_5340dk_nrf_5340_cpuapp. Everything else should be fine as default, the project config should be prj.conf and the board file will be auto set.
6. You are now almost ready to build, there are only 2 extra steps to make the project link correctly. First, there is a function zephyrfilesystem.c called f_expand, which zephyr turns off by default. To enable this, go to the file yourtoolchaindirectory/v2.5.3/modules/fs/fatfs/include/ffconfig.h and on line 41 set:
#define FF_USE_EXPAND 1 (should be at 0 previously)
![image](https://github.com/user-attachments/assets/9813e651-c49d-41b3-aa45-09411bf0ace5)
![image](https://github.com/user-attachments/assets/9c28736b-4162-45c1-8a61-b34aa34c8780) 
7. our nand flash has a page size of 4096, which unfortunetly in the usb module is hardcoded in to 512. To fix this, go to yourtoolchaindirectory/v2.5.3/zephyr/subsys/usb/device/class/msc.c and on line 111, change to #define BLOCK_SIZE 4096 (should be at 512 previously)

![image](https://github.com/user-attachments/assets/1b7cfb80-060e-463c-9ed1-fcc5e2e8ecaf)
![image](https://github.com/user-attachments/assets/7aef17d0-4818-4cef-a7ca-811a998a0163)
8. (optional) if you would like the MSense to record datetime stamps on the file system, go to ff.c (findable through) zephyrfilesystem.c DWORD get_fattime(void)
find all references -> zephyr_fatfs_config.h (located at {toolchain directory}\zephyr\modules\fatfs\zephyr_fatfs_config.h)
and set #define FF_FS_NORTC 0 (1 previously)
9. now, go back to the nrf connect tab, and in the 'actions' tab, click the build button. The application should now build correctly, and if you have a device connected to a JLink, you will be able to use the flash command to upload it.




The Guide for all Bluetooth Commands and how to use the device  is avalible at https://docs.google.com/document/d/1GqLLecDG5mTC0zdFGCEx2AJov6y5_ytyBaz1Vn2olMI/edit?usp=sharing
