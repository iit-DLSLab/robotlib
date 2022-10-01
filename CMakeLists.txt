# Minimum CMake version working in Ubuntu 20.04
cmake_minimum_required(VERSION 3.7.0)
project(robotlib)
option(BUILD_TESTING "" OFF)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CTest)
include(robotlib_paths)
include(robotlib_include_dirs)

find_package(Eigen3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CSS_STANDARD_REQUIRED True)



add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g> #Those options are enabled is a Debug configuration is set
	$<$<CONFIG:Debug>:-Wall>
	$<$<CONFIG:Debug>:-Wextra>
	$<$<CONFIG:Debug>:-pedantic>
	$<$<CONFIG:Release>:-O3>
)

message("\n***********Building robotlib library***********\n")
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	SYSTEM
)





add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

if (BUILD_TESTING)
	enable_testing()
	include(RobotlibGoogleTest)
	add_compile_options(
		-fsanitize=leak,address,undefined #Enable LeakSanitizer, AddressSanitizer, UndefinedBehaviorSanitizer 
	)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)
endif()

set(CPACK_PACKAGE_NAME "robotlib")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "robotlib")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "dlslab")

include(CPack)
