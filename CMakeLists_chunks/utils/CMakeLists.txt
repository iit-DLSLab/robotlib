# TODO: update file names according to new test structure
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/tests)

include(robotlib_paths)
include_directories(
	SYSTEM
	${EIGEN_DIR}
    ${PROJECT_SOURCE_DIR}/include/
)

enable_testing()
add_executable(main_tests
	${CMAKE_CURRENT_SOURCE_DIR}/tests/main_tests.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tests/unit_tests/utils_tests.cpp
)

target_link_libraries(main_tests
	PRIVATE
	${CMAKE_DL_LIBS}
	gtest
)
