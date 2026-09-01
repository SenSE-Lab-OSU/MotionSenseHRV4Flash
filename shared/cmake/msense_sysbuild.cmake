# Apply an app-owned HCI IPC configuration fragment only when sysbuild has
# selected the network-core HCI IPC image.  Each product retains ownership of
# the fragment because controller settings are intentionally product-specific.
function(msense_add_hci_ipc_overlay overlay_file)
  if(SB_CONFIG_NETCORE_HCI_IPC)
    add_overlay_config(hci_ipc "${overlay_file}")
  endif()
endfunction()
