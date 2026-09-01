# Sysbuild's automatic HCI IPC image does not consume the legacy
# child_image/hci_ipc.conf location by itself. Pass the product controller
# configuration explicitly to the network-core image before it is configured.
add_overlay_config(hci_ipc "${CMAKE_CURRENT_LIST_DIR}/child_image/hci_ipc.conf")
