# SPDX-License-Identifier: Apache-2.0

# This is a general purpose project created from MotionSenseHRV to be used for data collection.

# Ohio State SENSE LAB
cmake_minimum_required(VERSION 3.20.0)

# Preserve direct (non-sysbuild) configuration as well. Sysbuild receives the
# same repository board root from sysbuild/CMakeLists.txt before child images
# are configured.
list(APPEND BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

list(APPEND ZEPHYR_EXTRA_MODULES "${CMAKE_CURRENT_SOURCE_DIR}/../shared")


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(MotionSenseHrv)

find_package(Git QUIET)

set(MSENSE_GIT_EXECUTABLE "")
if(GIT_FOUND)
	set(MSENSE_GIT_EXECUTABLE "${GIT_EXECUTABLE}")
endif()

set(MSENSE_GIT_METADATA_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(MSENSE_GIT_METADATA_HEADER
	"${MSENSE_GIT_METADATA_DIR}/msense_git_metadata.h")

add_custom_target(msense_git_metadata ALL
	COMMAND ${CMAKE_COMMAND}
		"-DOUTPUT_FILE=${MSENSE_GIT_METADATA_HEADER}"
		"-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
		"-DGIT_EXECUTABLE=${MSENSE_GIT_EXECUTABLE}"
		-P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_git_metadata.cmake"
	BYPRODUCTS "${MSENSE_GIT_METADATA_HEADER}"
	COMMENT "Generating Git build metadata"
	VERBATIM
)

add_dependencies(app msense_git_metadata)
target_include_directories(app PRIVATE "${MSENSE_GIT_METADATA_DIR}")

include(CTest)
if(BUILD_TESTING AND GIT_FOUND)
	add_test(
		NAME msense_git_metadata
		COMMAND ${CMAKE_COMMAND}
			"-DGENERATOR_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_git_metadata.cmake"
			"-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
			"-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/tests/git_metadata"
			-P "${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/test_git_metadata.cmake"
	)
endif()

set(CMAKE_OBJECT_PATH_MAX 380)
if((NOT CONFIG_DISK_DRIVER_FLASH) AND
   (NOT CONFIG_DISK_DRIVER_RAM) AND
   (NOT CONFIG_DISK_DRIVER_SDMMC))
	message(WARNING "No disk access settings detected." )
endif()
target_compile_options(app PRIVATE -Wno-unused-function)

target_include_directories(app PRIVATE
	"${ZEPHYR_MSENSE_SHARED_MODULE_DIR}/drivers/nand"
	"${ZEPHYR_MSENSE_SHARED_MODULE_DIR}/drivers/nor"
)

target_sources(app PRIVATE
	src/BLEService.c
	src/batterymonitordt.c
	src/device_identity.c
	src/filesystemlogging.c
	src/imuSensor.c
	src/main.c
	src/ppgSensor.c
	src/zephyrfilesystem.c
)

target_sources_ifdef(CONFIG_DISK_DRIVER_RAW_NAND app PRIVATE
	src/drivers/nand/nand_disk.c
	src/drivers/nand/spi_nand.c
)

#if(CONFIG_BUILD_WITH_TFM)
#  target_include_directories(app PRIVATE
#    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
#  )
#endif()
