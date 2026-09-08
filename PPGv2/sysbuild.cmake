# Share the sysbuild mechanism while retaining the PPG-specific controller
# configuration fragment.
include("${CMAKE_CURRENT_LIST_DIR}/../shared/cmake/msense_sysbuild.cmake")
msense_add_hci_ipc_overlay(
  "${CMAKE_CURRENT_LIST_DIR}/child_image/hci_ipc.conf"
)
