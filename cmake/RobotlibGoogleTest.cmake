find_package(GTest REQUIRED)
include(GoogleTest)

define_property(GLOBAL PROPERTY ALL_TEST_TARGETS
    BRIEF_DOCS "All GTest Targets"
    FULL_DOCS "All GTest Targets")

set_property(GLOBAL PROPERTY ALL_TEST_TARGETS "")

function(add_robotlib_test)
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSE_ARGV 0 "TEST" "" "TARGET" "SOURCES;LIBRARIES")
    message(STATUS "Adding Test: ${TEST_TARGET}...")

	add_executable(
		${TEST_TARGET} 
		${TEST_SOURCES}
	)

	target_link_libraries(
		${TEST_TARGET}
		GTest::gtest_main
		robotlib
	)

	# Tests use flat includes (e.g. "dummy_robot/dummy_robot_creator.hpp"), which
	# require include/robotlib; robotlib only exposes it as PRIVATE.
	target_include_directories(
		${TEST_TARGET}
		PRIVATE
		${PROJECT_SOURCE_DIR}/include/robotlib
	)

    add_test(${TEST_TARGET} ${TEST_TARGET})
    set_property(GLOBAL APPEND PROPERTY ALL_TEST_TARGETS "${TEST_TARGET}")
endfunction()

