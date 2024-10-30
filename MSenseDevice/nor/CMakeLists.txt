# SPDX-License-Identifier: Apache-2.0

# This is a general purpose project created from MotionSenseHRV to be used for data collection.

# Ohio State SENSE LAB
cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../src/drivers)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(MotionSenseHrv)

set(CMAKE_OBJECT_PATH_MAX 380)
if((NOT CONFIG_DISK_DRIVER_FLASH) AND
   (NOT CONFIG_DISK_DRIVER_RAM) AND
   (NOT CONFIG_DISK_DRIVER_SDMMC))
	message(WARNING "No disk access settings detected." )
endif()
#
#we previously used GLOB_RECURSE, but that was deprecated since we want to have control over what code is included.
FILE(GLOB_RECURSE app_sources ../src/*.cc)
FILE(GLOB c_app_sources ../src/*.c)

#eventually, remove the induvidual file in target_sources, and just refactor to move zephyr filesystem out.
#OR Keep Global Recuse on.
# also move the drivers over to a seperate folder.
target_sources(app PRIVATE ${app_sources} 
${c_app_sources} )

#if(CONFIG_BUILD_WITH_TFM)
#  target_include_directories(app PRIVATE
#    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
#  )
#endif()




