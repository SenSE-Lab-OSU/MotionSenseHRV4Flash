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



**My Tips and Tricks**


First off: nrfconnect routes many things through two major systems: devicetree, and KConfig. Both of them can use parameters from each other, and can be used (and are supposed to be used) in C code to reference global variables and hardware abstracted code. DeviceTree specifically is very important, because it is the full code link to any hardware. It is a nodebased system that has it's own language where you can build hiearichal nodes representing the hardware components of your device. Thankfully, the nrf5r340 devkit device tree setup covers just about all the nodes we have on our board, but we did have to make some specific modifications to account for some of the customized hardware we did differently fromt the dk (see nrf5340dk_nrf5340_cpuapp.overlay).  I recommend studying both of these systems, and their interactions, very extensively, in order to understand what may be going on in the code.

if you get a linker error that looks something along the lines of "undefined reference to device__dts__ordXX, where XX is a number, this is a sign that you attempting to use a specific property or node in devicetree that, for whatever reason, has not been included when devicetree was built. To resolve this error, you'll need to first look in device tree and make sure that you are properly defining the node that you have built. If this does not work, you'll then need to make sure that you have enabled all the necessary KConfigs for that node - because sometimes KConfigs can Disable Nodes.

KConfigs are not all determined by the prj.conf file. KConfigs can also be set in device tree internally, and can also be set in underlying KConfigs that get included when you choose to include certain drivers in your project (in somewhat of a recursive manner, as the drivers you decide to include in your project are based upon KConfigs) 


In zephyr, driver code is not accesible directly, but instead avalible through an abstract generic interface called the device driver model (which works through devicetree). This is supposed to allow compatibility among different hardware that requires different drivers (as they can be easily swapped out, since if they all implement the same functions, swapping one out for the other will not cause any compile errors) Drivers often don't even have an includable header file, so if you have specific driver functionality you would like to use that is not supported by the interface, you'll have to do some custom workaround work on your own. If you want to implement your own out of tree driver, that implements an interface you need, you can take a look at our nand flash driver in this project, in which we built an interface for the disk driver interface which enables fatfs to work. As a summary, you will need to make sure you have a .yaml in dts/binding (let's devicetree recognize the driver), a zephyr.module.yml for zephyr to identify it, and CMakeList for zephyr to compile (and you also provide KConfigs for determining compilation).

There are technically 2 ways to access interfaces like spi, i2c from this standpoint. The first is to use the non driver specific libraries that are provided by nrfconnect. Alternatively, you can use the device driver that has higher level functionality, but you'll need to make sure that you've specified that you're going to use the driver in device tree, and call the hardware node that you want to use correctly 

Typically, partitions in flash are managed by device tree. However, because we have a multicore build, which is dependent on a system called partition manager, this system
is in charge of writing all the files.



