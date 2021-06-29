cmake_minimum_required(VERSION 3.5.1)
project(robotlib)

set(MAJOR 0)
set(MINOR 0)
set(PATCH 0)

# TODO: Understand the command below
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_compile_options(
	-std=c++14
	-fPIC
	$<$<CONFIG:Debug>:-g>
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	SYSTEM
	/usr/include/eigen3
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# GTest
enable_testing()

add_executable(main_tests
	tests/main_tests.cpp
	tests/model_test.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(robot_info
	${CMAKE_CURRENT_SOURCE_DIR}/src/robot_info.cpp
)

target_link_libraries(robot_info
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
)
