Welcome to the MSense4 Development guide! For those that not yet developed with Zephyr or the NrfConnect Sdk, this guide exists as a tool to help you find your ropes around this project, as our build system has become very complex. This is partly due to the way that Zephyr wants software to be built, and partly because our embedded software in it of itself is complex, requiring multiple partitions, dual core functionality for bluetooth low energy, and lots of other sensors.



About the Project

Overview of files: 


**My Tips and Tricks**

First off: nrfconnect routes many things through two major systems: devicetree, and KConfig. Both of them can use parameters from each other, and can be used (and are supposed to be used) in C code to reference global variables and hardware abstracted code. DeviceTree specifically is very important, because it is the full code link to any hardware. It is a nodebased system that has it's own language where you can build hiearichal nodes representing the hardware components of your device. Thankfully, the nrf5r340 devkit device tree setup covers just about all the nodes we have on our board, but we did have to make some specific modifications to account for some of the customized hardware we did differently fromt the dk (see nrf5340dk_nrf5340_cpuapp.overlay).  I recommend studying both of these systems, and their interactions, very extensively, in order to understand what may be going on in the code.

KConfigs are a way of creating settings that can be utilized within all the different systems of zephyr. They can be used anywhere in code almost like a preprocessor headings, and can also control device tree, and what gets compiled via CMakeLists. There are a bunch of KConfigs created by Nrf and Zephyr, and we additionally have our own KConfigs that we define both in the drivers folder and main src folder. KConfigs can be set mainly in the prj.conf file, though they are not always determined this way. KConfigs can also be set in device tree internally, and can also be set in underlying KConfigs that get included when you choose to include certain drivers in your project (in somewhat of a recursive manner, as the drivers you decide to include in your project are based upon KConfigs). KConfigs in the device tree or CMakeLists typically tend to override their value that's set in prj.conf, but if that happens the terminal will warn you during the configuration stage. The warning will usually look something like this: 

warning: KCONFIG_SYMBOL (defined at (first location),
other locations where the value was assigned) was assigned
the value 'VALUE you set' but got the value 'DIFFERENT VALUE'

this means that the KConfig is being set by something else, whether that be device tree, or another KConfig (yes, KConfigs can set other KConfigs in KConfig definition files). You will need to look at those definitions to determine how to properly get the value that you want.

-if you get a linker error that looks something along the lines of "undefined reference to device__dts__ordXX, where XX is a number, this is a sign that you attempting to use a specific property or node in devicetree that, for whatever reason, has not been included when devicetree was built. To resolve this error, you'll need to first look in device tree and make sure that you are properly defining the node that you have built. If this does not work, you'll then need to make sure that you have enabled all the necessary KConfigs for that node - because sometimes KConfigs can Disable Nodes. 



Typically, partitions in flash are managed by device tree. However, because we have a multicore build, which is dependent on a system called partition manager, this system
is in charge of writing all the files.


In zephyr, driver code is not accesible directly, but instead avalible through an abstract generic interface called the device driver model (which works through devicetree). This is supposed to allow compatibility among different hardware that requires different drivers (as they can be easily swapped out, since if they all implement the same functions, swapping one out for the other will not cause any compile errors) Drivers often don't even have an includable header file, so if you have specific driver functionality you would like to use that is not supported by the interface, you'll have to do some custom workaround work on your own. If you want to implement your own out of tree driver, that implements an interface you need, you can take a look at our nand flash driver in this project, in which we built an interface for the disk driver interface which enables fatfs to work. As a summary, you will need to make sure you have a .yaml in dts/binding (let's devicetree recognize the driver), a zephyr.module.yml for zephyr to identify it, and CMakeList for zephyr to compile (and you also provide KConfigs for determining compilation).

There are technically 2 ways to access interfaces like spi, i2c from this standpoint. The first is to use the non driver specific libraries that are provided by nrfconnect. Alternatively, you can use the device driver that has higher level functionality, but you'll need to make sure that you've specified that you're going to use the driver in device tree, and call the hardware node that you want to use correctly 


Due to the way zephyr sets up it's device driver model, There are 5 locations for every driver where config and code must go:

1. src/drivers/(c files): where are all the source code (the c files and header files) is stored

2. src/drivers/Kconfig: where KConfigs go for drivers, which can be used by prj.conf, any CMakeLists.txt, and any code

3. src/drivers/CMakesList.txt: file that actually includes the driver for compilation in the project

4. src/drivers/zephyr/module.yml: tells zephyr where all the above^ files are

5. dts/...: defines the device tree settings for the driver. Is used by the c file that defines the driver. Note that when using these parameters in code, dashes (-) are converted to underscores (_). So if you want to access property-1 in code, you do property_1.







