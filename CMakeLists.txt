cmake_minimum_required(VERSION 3.15)
project(robotlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#used only for issues with #include <experimental/filesystem>
#TODO: delete it once dls images are updated with Ubuntu20 (and gcc version > 8)
link_libraries(stdc++fs)	

find_package(Eigen3 REQUIRED NO_MODULE)

add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g>
	#-Wall -Wextra -pedantic # Enable warnings
	#-fsanitize=undefined -g -O3	# UBSAN - Undefined Behaviour Sanitizer
)
#add_link_options(-fsanitize=undefined) # UBSAN - Undefined Behaviour Sanitizer

# ==============================================================================
# Setup the repository
# ==============================================================================
# This will perfom task such as setting up the git hooks for anyone who uses
# this project
execute_process(
	COMMAND           ./setup.sh
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# ==============================================================================
# Calculate correct project version number
# ==============================================================================
# Read in the current version
file(READ version.txt PROJECT_VERSION)

# create array of version strings
string(REPLACE "." ";" VERSION_ARRAY ${PROJECT_VERSION})
# ------------------------------------------------------------------------------
# Major, Minor, Patch numbers
# ------------------------------------------------------------------------------
# set corresponding cmake version variables
list(POP_FRONT VERSION_ARRAY
	PROJECT_VERSION_MAJOR
	PROJECT_VERSION_MINOR
	PROJECT_VERSION_PATCH
)
# Remove the trailing newline from the patch number
string(REPLACE "\n" "" PROJECT_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# ------------------------------------------------------------------------------
# Full version number
# ------------------------------------------------------------------------------
# Rebuild the project version including the current commit hash
string(JOIN "." PROJECT_VERSION
	${PROJECT_VERSION_MAJOR}
	${PROJECT_VERSION_MINOR}
	${PROJECT_VERSION_PATCH}
)
unset(VERSION_ARRAY) # delete aray of version strings
message(STATUS "PID ROJECT VERSION ${PROJECT_VERSION}")

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

	include(debian_settings) # configure installation
	
endif()
unset(CODE_TO_COMPILE CACHE)	# unset the variable stored in CMakeCache.txt 

