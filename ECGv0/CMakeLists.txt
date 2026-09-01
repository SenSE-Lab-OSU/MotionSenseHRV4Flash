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

include("${ZEPHYR_MSENSE_SHARED_MODULE_DIR}/cmake/msense_git_metadata.cmake")
msense_add_git_metadata("${CMAKE_CURRENT_SOURCE_DIR}")

include(CTest)
if(BUILD_TESTING)
	add_test(
		NAME msense_storage_geometry
		COMMAND ${CMAKE_COMMAND}
			"-DMSENSE_SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/.."
			-P "${CMAKE_CURRENT_SOURCE_DIR}/../shared/tests/cmake/test_storage_geometry.cmake"
	)
	add_test(
		NAME msense_phase4_contracts
		COMMAND ${CMAKE_COMMAND}
			"-DMSENSE_SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/.."
			"-DMSENSE_PRODUCT=ECGv0"
			"-DMSENSE_APP_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}"
			-P "${CMAKE_CURRENT_SOURCE_DIR}/../shared/tests/cmake/test_phase4_contracts.cmake"
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
	"${ZEPHYR_MSENSE_SHARED_MODULE_DIR}/include"
	"${ZEPHYR_MSENSE_SHARED_MODULE_DIR}/drivers/nand"
	"${ZEPHYR_MSENSE_SHARED_MODULE_DIR}/drivers/nor"
)

target_sources(app PRIVATE
	src/BLEService.c
	src/accelRecordFormat.c
	src/accelRecorder.c
	src/accelTimingEstimator.c
	src/batterymonitordt.c
	src/ecgRecordFormat.c
	src/ecgRecorder.c
	src/filesystemlogging_policy.c
	src/icm20948_accel.c
	src/imuFsyncTiming.c
	src/main.c
	src/zephyrfilesystem.c
	src/drivers/ecg/max30001.c
)

#if(CONFIG_BUILD_WITH_TFM)
#  target_include_directories(app PRIVATE
#    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/install/interface/include
#  )
#endif()
