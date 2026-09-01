# BOARD_ROOT is read by sysbuild before it configures the application, MCUboot,
# and the network-core HCI IPC image. Keep this at repository scope instead of
# relying on an application-only setting.
list(APPEND BOARD_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
