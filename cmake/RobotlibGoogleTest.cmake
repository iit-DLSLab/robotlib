find_package(GTest REQUIRED)
include(GoogleTest)

define_property(GLOBAL PROPERTY ALL_TEST_TARGETS
    BRIEF_DOCS "All GTest Targets"
    FULL_DOCS "All GTest Targets")

set_property(GLOBAL PROPERTY ALL_TEST_TARGETS "")

function(add_robotlib_test)
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSE_ARGV 0 "TEST" "" "TARGET" "SOURCES;LIBRARIES")
    message("Adding Test: ${TEST_TARGET}...")
		add_executable(${TEST_TARGET} ${TEST_SOURCES})
		install(TARGETS ${TEST_TARGET} RUNTIME DESTINATION ${ROBOTLIB_INSTALL_RUNTIME_DIR}/${CMAKE_PROJECT_NAME} DESTINATION ${ROBOTLIB_INSTALL_RUNTIME_DIR}/${CMAKE_PROJECT_NAME})
		target_link_libraries(${TEST_TARGET} ${TEST_LIBRARIES} robotlib GTest::GTest GTest::Main ${CMAKE_DL_LIBS})
        # if you want to consider also sanitizers during testing uncomment the next line
		# target_link_libraries(${TEST_TARGET} -fsanitize=leak,address,undefined)
		gtest_discover_tests(${TEST_TARGET})
		set_property(GLOBAL APPEND PROPERTY ALL_TEST_TARGETS "${TEST_TARGET}")
endfunction()