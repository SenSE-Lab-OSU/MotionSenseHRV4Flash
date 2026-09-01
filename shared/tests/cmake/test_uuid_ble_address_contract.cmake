# Source-only regression guard for uuid.txt BLE identity persistence.

if(NOT DEFINED MSENSE_SOURCE_ROOT OR MSENSE_SOURCE_ROOT STREQUAL "")
	message(FATAL_ERROR "MSENSE_SOURCE_ROOT must identify the repository root")
endif()

function(require_text relative_path required_text)
	set(source_path "${MSENSE_SOURCE_ROOT}/${relative_path}")
	if(NOT EXISTS "${source_path}")
		message(FATAL_ERROR "Required source file is missing: ${relative_path}")
	endif()

	file(READ "${source_path}" source_contents)
	string(FIND "${source_contents}" "${required_text}" required_position)
	if(required_position EQUAL -1)
		message(FATAL_ERROR "${relative_path} is missing: ${required_text}")
	endif()
endfunction()

function(require_text_absent relative_path forbidden_text)
	set(source_path "${MSENSE_SOURCE_ROOT}/${relative_path}")
	file(READ "${source_path}" source_contents)
	string(FIND "${source_contents}" "${forbidden_text}" forbidden_position)
	if(NOT forbidden_position EQUAL -1)
		message(FATAL_ERROR "${relative_path} must not duplicate: ${forbidden_text}")
	endif()
endfunction()

function(require_order_in_region relative_path start_marker end_marker first_text second_text)
	file(READ "${MSENSE_SOURCE_ROOT}/${relative_path}" source_contents)
	string(FIND "${source_contents}" "${start_marker}" region_start)
	if(region_start EQUAL -1)
		message(FATAL_ERROR "${relative_path} is missing region start: ${start_marker}")
	endif()
	string(SUBSTRING "${source_contents}" ${region_start} -1 region_contents)
	string(FIND "${region_contents}" "${end_marker}" region_end)
	if(region_end EQUAL -1)
		message(FATAL_ERROR "${relative_path} is missing region end: ${end_marker}")
	endif()
	string(SUBSTRING "${region_contents}" 0 ${region_end} region_contents)
	string(FIND "${region_contents}" "${first_text}" first_position)
	string(FIND "${region_contents}" "${second_text}" second_position)
	if(first_position EQUAL -1 OR second_position EQUAL -1 OR
	   NOT first_position LESS second_position)
		message(FATAL_ERROR
			"${relative_path} must perform ${first_text} before ${second_text}")
	endif()
endfunction()

set(expected_ble_address "DA:BC:12:34:56:78 (random)")
string(LENGTH "${expected_ble_address}" expected_ble_address_length)
if(NOT expected_ble_address_length EQUAL 26)
	message(FATAL_ERROR "Historic random BLE address line must be 26 characters")
endif()
if(NOT expected_ble_address MATCHES
	"^[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F] \\(random\\)$")
	message(FATAL_ERROR "Historic BLE address format changed unexpectedly")
endif()

require_text("shared/CMakeLists.txt" "zephyr_library_sources_ifdef(CONFIG_FILE_SYSTEM uuid_file.c)")
require_text("shared/include/msense_uuid_file.h" "msense_uuid_file_ble_address_present")
require_text("shared/include/msense_uuid_file.h" "msense_uuid_file_prepend_ble_address")
require_text("shared/uuid_file.c" "#define UUID_BLE_ADDRESS_LINE_LEN 26U")
require_text("shared/uuid_file.c" "\" (random)\"")
require_text("shared/uuid_file.c" "if (uuid_has_ble_address_line(uuid_contents, uuid_contents_len))")
require_text("shared/uuid_file.c" "memmove(uuid_contents + ble_address_len + 1U")
require_text("shared/uuid_file.c" "FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC")

foreach(product IN ITEMS PPGv2 ECGv0)
	set(main_source "${product}/src/main.c")
	set(filesystem_source "${product}/src/zephyrfilesystem.c")
	set(filesystem_header "${product}/src/zephyrfilesystem.h")

	require_text("${filesystem_header}" "bool *ble_address_present")
	require_text("${filesystem_header}" "write_device_info_ble_address")
	require_text("${filesystem_source}" "#include \"msense_uuid_file.h\"")
	require_text("${filesystem_source}" "msense_uuid_file_ble_address_present")
	require_text("${filesystem_source}" "msense_uuid_file_create")
	require_text("${filesystem_source}" "msense_uuid_file_prepend_ble_address")
	require_text_absent("${filesystem_source}" "static bool uuid_has_ble_address_line")
	require_text_absent("${filesystem_source}" "static int read_uuid_contents")

	require_text("${main_source}" "bt_id_get(&identity_address, &address_count);")
	require_text("${main_source}" "identity_address.type != BT_ADDR_LE_RANDOM")
	require_text("${main_source}" "!BT_ADDR_IS_STATIC(&identity_address.a)")
	require_text("${main_source}" "bt_addr_le_to_str(&identity_address, ble_address")
	require_text("${main_source}" "uuid_ble_address_msc_deferred = true;")
	require_order_in_region("${main_source}"
		"static int complete_pending_uuid_ble_address"
		"static void le_param_updated"
		"write_device_info_ble_address(ble_address)"
		"filesystem_gate_and_drain()")
	require_order_in_region("${main_source}"
		"static int complete_pending_uuid_ble_address"
		"static void le_param_updated"
		"filesystem_gate_and_drain()"
		"enable_usb_msc_host_media()")
	require_order_in_region("${main_source}"
		"static void bt_ready"
		"static void ble_init"
		"complete_pending_uuid_ble_address()"
		"bt_le_adv_start")
endforeach()
