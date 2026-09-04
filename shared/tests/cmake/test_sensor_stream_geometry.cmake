# Source-only regression guard for the version-1 bounded stream geometry.

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

math(EXPR ppg_history_bytes "2048 * 16")
math(EXPR ppg_forward_bytes "6144 * 16")
math(EXPR ppg_total_bytes "${ppg_history_bytes} + ${ppg_forward_bytes}")
math(EXPR ecg_history_bytes "2731 * 12")
math(EXPR ecg_forward_bytes "8192 * 12")
math(EXPR ecg_total_bytes "${ecg_history_bytes} + ${ecg_forward_bytes}")

if(NOT ppg_history_bytes EQUAL 32768 OR NOT ppg_forward_bytes EQUAL 98304 OR
   NOT ppg_total_bytes EQUAL 131072)
	message(FATAL_ERROR "Unexpected PPG stream geometry")
endif()

if(NOT ecg_history_bytes EQUAL 32772 OR NOT ecg_forward_bytes EQUAL 98304 OR
   NOT ecg_total_bytes EQUAL 131076)
	message(FATAL_ERROR "Unexpected ECG stream geometry")
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
	"#define MSENSE_SENSOR_STREAM_ECG_RECORD_SIZE 12U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_HISTORY_RECORDS 2731U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS 8192U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"#define MSENSE_SENSOR_STREAM_ECG_TOTAL_SENSOR_BYTES 131076U")
require_text("shared/include/msense_sensor_stream_protocol.h"
	"MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES")
require_text("shared/sensor_stream.c"
	"MSENSE_SENSOR_STREAM_PPG_FORWARD_RECORDS == 98304U")
require_text("shared/sensor_stream.c"
	"MSENSE_SENSOR_STREAM_ECG_FORWARD_RECORDS == 98304U")
require_text("shared/sensor_stream.c"
	"MSENSE_SENSOR_STREAM_CAPTURE_BUFFER_BYTES")
require_text("central_nus_test/src/main.c"
	"MSENSE_SENSOR_STREAM_PPG_TOTAL_SENSOR_BYTES")
require_text("central_nus_test/src/main.c"
	"MSENSE_SENSOR_STREAM_ECG_TOTAL_SENSOR_BYTES")
