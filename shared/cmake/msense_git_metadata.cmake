include_guard(GLOBAL)

set(MSENSE_GIT_METADATA_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(msense_add_git_metadata source_dir)
	find_package(Git QUIET)

	set(git_executable "")
	if(GIT_FOUND)
		set(git_executable "${GIT_EXECUTABLE}")
	endif()

	set(metadata_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
	set(metadata_header "${metadata_dir}/msense_git_metadata.h")

	add_custom_target(msense_git_metadata ALL
		COMMAND ${CMAKE_COMMAND}
			"-DOUTPUT_FILE=${metadata_header}"
			"-DSOURCE_DIR=${source_dir}"
			"-DGIT_EXECUTABLE=${git_executable}"
			-P "${MSENSE_GIT_METADATA_CMAKE_DIR}/generate_git_metadata.cmake"
		BYPRODUCTS "${metadata_header}"
		COMMENT "Generating Git build metadata"
		VERBATIM
	)

	add_dependencies(app msense_git_metadata)
	target_include_directories(app PRIVATE "${metadata_dir}")

	include(CTest)
	if(BUILD_TESTING AND GIT_FOUND)
		add_test(
			NAME msense_git_metadata
			COMMAND ${CMAKE_COMMAND}
				"-DGENERATOR_SCRIPT=${MSENSE_GIT_METADATA_CMAKE_DIR}/generate_git_metadata.cmake"
				"-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
				"-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/tests/git_metadata"
				-P "${MSENSE_GIT_METADATA_CMAKE_DIR}/../tests/cmake/test_git_metadata.cmake"
		)
	endif()
endfunction()
