# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
list(APPEND BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
list(APPEND DTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bridge_fpga_dev)

set(VME_FILE "${CMAKE_CURRENT_SOURCE_DIR}/bridge-hello.vme" CACHE FILEPATH "Reviewed Full VME input")
set(VME_HEADER "${CMAKE_CURRENT_BINARY_DIR}/vme_image.h")
add_custom_command(OUTPUT ${VME_HEADER}
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_vme.py
          ${VME_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/reviewed-vme.sha256 ${VME_HEADER}
  DEPENDS ${VME_FILE} reviewed-vme.sha256 tools/embed_vme.py
  VERBATIM)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_sources(app PRIVATE src/main.c src/vme.c src/jtag.c src/fpga_gpio.c ${VME_HEADER})
