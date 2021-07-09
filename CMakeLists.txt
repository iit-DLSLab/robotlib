cmake_minimum_required(VERSION 3.5.1)
project(robotlib)

set(MAJOR 0)
set(MINOR 0)
set(PATCH 0)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g>
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	SYSTEM
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(robotlib_paths)
include(robotlib_include_dirs)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)

set(CPACK_PACKAGE_NAME "robotlib")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "robotlib")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "dlslab")

include(CPack)

