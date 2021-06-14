cmake_minimum_required(VERSION 3.5.1)
project(robotlib)

set(MAJOR 0)
set(MINOR 0)
set(PATCH 0)

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
	#tests/model_test.cpp
	tests/LegDataMap_test.cpp
	#tests/temporary_test.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	gtest
)