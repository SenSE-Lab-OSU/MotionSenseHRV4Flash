# Source-only regression guard for shared v0 bounded stream geometry.

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

function(forbid_text relative_path forbidden_text)
	set(source_path "${MSENSE_SOURCE_ROOT}/${relative_path}")
	file(READ "${source_path}" source_contents)
	string(FIND "${source_contents}" "${forbidden_text}" forbidden_position)
	if(NOT forbidden_position EQUAL -1)
		message(FATAL_ERROR "${relative_path} retains obsolete: ${forbidden_text}")
	endif()
endfunction()

math(EXPR ppg_history_bytes "2048 * 16")
math(EXPR ppg_forward_bytes "6144 * 16")
math(EXPR ppg_total_bytes "${ppg_history_bytes} + ${ppg_forward_bytes}")
math(EXPR ppg_live_bytes "4096 * 16")
math(EXPR ecg_history_bytes "8 * 4096")
math(EXPR ecg_forward_bytes "24 * 4096")
math(EXPR ecg_total_bytes "${ecg_history_bytes} + ${ecg_forward_bytes}")
math(EXPR ecg_live_bytes "16 * 4096")

if(NOT ppg_history_bytes EQUAL 32768 OR NOT ppg_forward_bytes EQUAL 98304 OR
   NOT ppg_total_bytes EQUAL 131072)
	message(FATAL_ERROR "Unexpected PPG stream geometry")
endif()

if(NOT ecg_history_bytes EQUAL 32768 OR NOT ecg_forward_bytes EQUAL 98304 OR
   NOT ecg_total_bytes EQUAL 131072)
	message(FATAL_ERROR "Unexpected ECG stream geometry")
endif()

if(NOT ppg_live_bytes EQUAL 65536 OR NOT ecg_live_bytes EQUAL 65536)
	message(FATAL_ERROR "Live queue must remain 64 KiB independently of the 96 KiB quota")
endif()

require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PPG_RECORD_SIZE 16U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PPG_HISTORY_RECORDS 2048U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS 6144U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PPG_TOTAL_SENSOR_BYTES 131072U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION_ECG MSENSE_SENSOR_STREAM_PROTOCOL_VERSION")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION_PPG MSENSE_SENSOR_STREAM_PROTOCOL_VERSION")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"MSENSE_SENSOR_STREAM_OPCODE_START_INFINITY")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE 4096U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS 8U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS 24U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_TOTAL_SENSOR_BYTES 131072U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES")
require_text("shared/include/msense_ecg_block_format.h"
	"#define MSENSE_ECG_BLOCK_BYTES 4096U")
require_text("shared/sensor_stream.c"
	"stream.live_capacity = MSENSE_SENSOR_STREAM_HISTORY_BYTES * 2U /")
require_text("shared/sensor_stream.c"
	"MSENSE_SENSOR_STREAM_HISTORY_BYTES * 4U")
require_text("shared/sensor_stream.c"
	"stream.future_quota_records = config->forward_record_count;")
forbid_text("shared/sensor_stream.c" "start_arming")
forbid_text("shared/include/msense_sensor_stream_protocol.h"
	"enum msense_sensor_stream_state")
require_text("central_nus_test/src/main.c"
	"MSENSE_SENSOR_STREAM_FINITE_BYTES")
require_text("central_nus_test/src/main.c"
	"msense_ecg_block_validate")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_PROTOCOL_VERSION 0U")
