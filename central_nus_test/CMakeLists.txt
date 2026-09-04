# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(central)

target_sources(app PRIVATE src/main.c)

include(CTest)
if(BUILD_TESTING)
  add_test(
    NAME msense_sensor_stream_geometry
    COMMAND ${CMAKE_COMMAND}
      "-DMSENSE_SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/.."
      -P "${CMAKE_CURRENT_SOURCE_DIR}/../shared/tests/cmake/test_sensor_stream_geometry.cmake"
  )
endif()

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
)
