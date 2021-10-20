cmake_minimum_required(VERSION 3.5.1)
project(robotlib)

set(MAJOR 0)
set(MINOR 0)
set(PATCH 0)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#used only for issues with #include <experimental/filesystem>
#TODO: delete it once dls images are updated with Ubuntu20 (and gcc version > 8)
link_libraries(stdc++fs)	

find_package(Eigen3 REQUIRED NO_MODULE)

add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g>
)

if(NOT CODE_TO_COMPILE)
    set(CODE_TO_COMPILE "ALL")
endif()

if(CODE_TO_COMPILE STREQUAL "Limb")
	message("\n***********Building only the Limb related code***********\n")
	include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists_chunks/limb/CMakeLists.txt)

elseif(CODE_TO_COMPILE STREQUAL "utils")
	message("\n***********Building only the utils related code***********\n")
	include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists_chunks/utils/CMakeLists.txt)
	
else()
	message("\n***********Building the entire robotlib library***********\n")
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
endif()
unset(CODE_TO_COMPILE CACHE)	# unset the variable stored in CMakeCache.txt 

