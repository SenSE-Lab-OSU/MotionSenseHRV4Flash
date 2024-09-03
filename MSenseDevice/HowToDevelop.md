Welcome to the MSense4 Development guide! For those that not yet developed with Zephyr or the NrfConnect Sdk, this guide exists as a tool to help you find your ropes around this project, as our build system has become very complex. This is partly due to the way that Zephyr wants software to be built, and partly because our embedded software in it of itself is complex, requiring multiple partitions, dual core functionality for bluetooth low energy, and lots of other sensors.



About the Project

C Project
The files are structured in such a way that each file 


My Tips and Tricks


if you get a linker error that looks something along the lines of "undefined reference to device__dts__ordXX, where XX is a number, this is a sign that you attempting to use a specific property or node in devicetree that, for whatever reason, has not been included when devicetree was built. To resolve this error, you'll need to first look in device tree and make sure that you are properly defining the node that you have built. If this does not work, you'll then need to make sure that you have enabled all the necessary KConfigs for that node - because sometimes KConfigs can Disable Nodes.

KConfigs are not all determined by the prj.conf file. KConfigs can also be set in device tree internally, and can also be set in underlying KConfigs that get included when you choose to include certain drivers in your project (in somewhat of a recursive manner, as the drivers you decide to include in your project are based upon KConfigs) 


Typically, partitions in flash are managed by device tree. However, because we have a multicore build, which is dependent on a system called partition manager, this system
is in charge of writing all the files.






