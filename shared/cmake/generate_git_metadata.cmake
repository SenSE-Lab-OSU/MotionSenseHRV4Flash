if(NOT DEFINED OUTPUT_FILE OR OUTPUT_FILE STREQUAL "")
	message(FATAL_ERROR "OUTPUT_FILE must be provided")
endif()

if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
	message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(git_commit "unknown")
set(git_tree_state "unknown")

if(DEFINED GIT_EXECUTABLE AND NOT GIT_EXECUTABLE STREQUAL "" AND
   EXISTS "${GIT_EXECUTABLE}")
	execute_process(
		COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" rev-parse --verify HEAD
		RESULT_VARIABLE git_commit_result
		OUTPUT_VARIABLE git_commit_output
		ERROR_QUIET
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	string(LENGTH "${git_commit_output}" git_commit_length)
	string(REGEX MATCH "^[0-9a-fA-F]+$" git_commit_hex "${git_commit_output}")

	if(git_commit_result EQUAL 0 AND
	   git_commit_length EQUAL 40 AND
	   git_commit_hex STREQUAL git_commit_output)
		execute_process(
			COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" status --porcelain --untracked-files=no
			RESULT_VARIABLE git_status_result
			OUTPUT_VARIABLE git_status_output
			ERROR_QUIET
		)

		if(git_status_result EQUAL 0)
			string(TOLOWER "${git_commit_output}" git_commit)
			if(git_status_output STREQUAL "")
				set(git_tree_state "clean")
			else()
				set(git_tree_state "dirty")
			endif()
		endif()
	endif()
endif()

string(CONCAT generated_header
	"#ifndef MSENSE_GIT_METADATA_H_\n"
	"#define MSENSE_GIT_METADATA_H_\n"
	"\n"
	"#define MSENSE_GIT_COMMIT \"${git_commit}\"\n"
	"#define MSENSE_GIT_TREE_STATE \"${git_tree_state}\"\n"
	"\n"
	"#endif /* MSENSE_GIT_METADATA_H_ */\n"
)

get_filename_component(output_dir "${OUTPUT_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${output_dir}")

set(write_header TRUE)
if(EXISTS "${OUTPUT_FILE}")
	file(READ "${OUTPUT_FILE}" existing_header)
	if(existing_header STREQUAL generated_header)
		set(write_header FALSE)
	endif()
endif()

if(write_header)
	file(WRITE "${OUTPUT_FILE}" "${generated_header}")
endif()
