# Apply the project-owned configuration fragment to the network-core HCI IPC
# image selected in sysbuild.conf.
if(SB_CONFIG_NETCORE_HCI_IPC)
  add_overlay_config(
    hci_ipc
    ${CMAKE_CURRENT_LIST_DIR}/child_image/hci_ipc.conf
  )
endif()
