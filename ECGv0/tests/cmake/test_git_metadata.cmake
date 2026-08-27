if(NOT DEFINED GENERATOR_SCRIPT OR NOT EXISTS "${GENERATOR_SCRIPT}")
	message(FATAL_ERROR "GENERATOR_SCRIPT must identify generate_git_metadata.cmake")
endif()

if(NOT DEFINED GIT_EXECUTABLE OR GIT_EXECUTABLE STREQUAL "")
	message(FATAL_ERROR "GIT_EXECUTABLE must be provided")
endif()

if(NOT DEFINED TEST_BINARY_DIR OR TEST_BINARY_DIR STREQUAL "")
	message(FATAL_ERROR "TEST_BINARY_DIR must be provided")
endif()

function(run_git source_dir)
	execute_process(
		COMMAND "${GIT_EXECUTABLE}" -C "${source_dir}" ${ARGN}
		RESULT_VARIABLE command_result
		OUTPUT_VARIABLE command_output
		ERROR_VARIABLE command_error
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	if(NOT command_result EQUAL 0)
		message(FATAL_ERROR "Git command failed: ${ARGN}\n${command_error}")
	endif()
	set(RUN_GIT_OUTPUT "${command_output}" PARENT_SCOPE)
endfunction()

function(generate_metadata source_dir output_file)
	set(generator_git_executable "${GIT_EXECUTABLE}")
	if(ARGC GREATER 2)
		set(generator_git_executable "${ARGV2}")
	endif()

	execute_process(
		COMMAND "${CMAKE_COMMAND}"
			"-DOUTPUT_FILE=${output_file}"
			"-DSOURCE_DIR=${source_dir}"
			"-DGIT_EXECUTABLE=${generator_git_executable}"
			-P "${GENERATOR_SCRIPT}"
		RESULT_VARIABLE command_result
		OUTPUT_VARIABLE command_output
		ERROR_VARIABLE command_error
	)
	if(NOT command_result EQUAL 0)
		message(FATAL_ERROR "Metadata generation failed:\n${command_output}\n${command_error}")
	endif()
endfunction()

function(expect_header output_file commit tree_state)
	file(READ "${output_file}" header_contents)
	set(expected_commit "#define MSENSE_GIT_COMMIT \"${commit}\"")
	set(expected_tree "#define MSENSE_GIT_TREE_STATE \"${tree_state}\"")
	string(FIND "${header_contents}" "${expected_commit}" commit_position)
	string(FIND "${header_contents}" "${expected_tree}" tree_position)
	if(commit_position EQUAL -1 OR tree_position EQUAL -1)
		message(FATAL_ERROR
			"Unexpected metadata header for ${output_file}:\n${header_contents}")
	endif()
endfunction()

file(REMOVE_RECURSE "${TEST_BINARY_DIR}")
file(MAKE_DIRECTORY "${TEST_BINARY_DIR}")

set(test_repository "${TEST_BINARY_DIR}/repository")
set(metadata_header "${TEST_BINARY_DIR}/generated/msense_git_metadata.h")
file(MAKE_DIRECTORY "${test_repository}")

run_git("${test_repository}" init)
run_git("${test_repository}" config user.email "metadata-test@example.com")
run_git("${test_repository}" config user.name "Git Metadata Test")
file(WRITE "${test_repository}/tracked.txt" "base\n")
run_git("${test_repository}" add tracked.txt)
run_git("${test_repository}" commit -m initial)
run_git("${test_repository}" rev-parse --verify HEAD)
set(expected_commit "${RUN_GIT_OUTPUT}")

generate_metadata("${test_repository}" "${metadata_header}")
expect_header("${metadata_header}" "${expected_commit}" "clean")

file(APPEND "${test_repository}/tracked.txt" "unstaged\n")
generate_metadata("${test_repository}" "${metadata_header}")
expect_header("${metadata_header}" "${expected_commit}" "dirty")

run_git("${test_repository}" add tracked.txt)
generate_metadata("${test_repository}" "${metadata_header}")
expect_header("${metadata_header}" "${expected_commit}" "dirty")

run_git("${test_repository}" reset --hard HEAD)
file(WRITE "${test_repository}/untracked.txt" "untracked\n")
generate_metadata("${test_repository}" "${metadata_header}")
expect_header("${metadata_header}" "${expected_commit}" "clean")

string(RANDOM LENGTH 16 ALPHABET 0123456789abcdef non_repository_suffix)
set(non_repository "$ENV{TEMP}/msense-git-metadata-${non_repository_suffix}")
file(MAKE_DIRECTORY "${non_repository}")
generate_metadata("${non_repository}" "${metadata_header}")
expect_header("${metadata_header}" "unknown" "unknown")

file(REMOVE_RECURSE "${non_repository}")
file(REMOVE_RECURSE "${TEST_BINARY_DIR}")
