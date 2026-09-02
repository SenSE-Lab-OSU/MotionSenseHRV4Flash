# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
# Sysbuild can preserve an empty DTC_OVERLAY_FILE. Default to the board's
# interface-MCU VCOM overlay while preserving an explicit caller override.
if(NOT DTC_OVERLAY_FILE)
  set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/nrf5340dk_nrf5340_cpuapp.overlay
  )
endif()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(central)

target_sources(app PRIVATE src/main.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
)
