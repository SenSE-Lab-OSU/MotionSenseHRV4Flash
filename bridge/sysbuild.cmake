#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

get_property(PM_DOMAINS GLOBAL PROPERTY PM_DOMAINS)

# Include app core image if enabled
if(SB_CONFIG_SOC_NRF5340_CPUNET)
  # Get application core board target
  string(REPLACE "/" ";" split_board_qualifiers "${BOARD_QUALIFIERS}")
  list(GET split_board_qualifiers 1 target_soc)
  set(board_target_appcore "${BOARD}/${target_soc}/cpuapp")
  set(target_soc)

  ExternalZephyrProject_Add(
    APPLICATION bridge_app_core
    SOURCE_DIR ${APP_DIR}/app_core
    BOARD ${board_target_appcore}
    BOARD_REVISION ${BOARD_REVISION}
  )

  if(NOT "CPUAPP" IN_LIST PM_DOMAINS)
    list(APPEND PM_DOMAINS CPUAPP)
  endif()

  set_property(GLOBAL APPEND PROPERTY PM_CPUAPP_IMAGES "bridge_app_core")
  set_property(GLOBAL PROPERTY DOMAIN_APP_CPUAPP "bridge_app_core")
  set(CPUAPP_PM_DOMAIN_DYNAMIC_PARTITION bridge_app_core CACHE INTERNAL "")
endif()

set_property(GLOBAL PROPERTY PM_DOMAINS ${PM_DOMAINS})
