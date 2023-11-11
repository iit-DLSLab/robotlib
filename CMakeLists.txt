cmake_minimum_required(VERSION 3.15)
project(robotlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

option(BUILD_TESTING "" OFF)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CTest)

find_package(Eigen3 REQUIRED NO_MODULE)

add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g>
	$<$<CONFIG:Release>:-O3>
	-Wall -Wextra -pedantic # Enable warnings
	#-fsanitize=undefined -g -O3	# UBSAN - Undefined Behaviour Sanitizer
)

#add_link_options(-fsanitize=undefined) # UBSAN - Undefined Behaviour Sanitizer

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
message(STATUS "ROBOTLIB ROJECT VERSION ${PROJECT_VERSION}")

# ==============================================================================
# Setup the repository
# ==============================================================================
# This will perfom task such as setting up the git hooks for anyone who uses
# this project
execute_process(
	COMMAND           ./setup.sh
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

if(NOT CODE_TO_COMPILE)
    set(CODE_TO_COMPILE "ALL")
endif()

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
	/usr/include/eigen3
	SYSTEM
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(robotlib_paths)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

if (BUILD_TESTING)
	enable_testing()
	include(RobotlibGoogleTest)
	# add_compile_options( # Uncomment these compile options to have more robust tests
	# 	-fsanitize=leak,address,undefined #Enable LeakSanitizer, AddressSanitizer, UndefinedBehaviorSanitizer 
	# )
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)	
endif()

set(CPACK_PACKAGE_VERSION_MAJOR         ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR         ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH         ${PROJECT_VERSION_PATCH})
set(CPACK_DEBIAN_PACKAGE_VERSION        ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR                "dlslab")
set(CPACK_PACKAGE_CONTACT               "dlslab")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER     "dlslab")
set(CPACK_VERBATIM_VARIABLES            ON)
set(CPACK_GENERATOR                     "DEB")
set(CPACK_DEB_COMPONENT_INSTALL         ON)
set(CPACK_PACKAGE_NAME 					"robotlib")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY 	"robotlib")


set(CPACK_DEBIAN_ROBOTLIB_PACKAGE_NAME          "dls-robotlib" CACHE INTERNAL "")
set(CPACK_DEBIAN_ROBOTLIB_FILE_NAME             "dls-robotlib-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ROBOTLIB_PACKAGE_DEPENDS       "" CACHE INTERNAL "")

set(CPACK_DEBIAN_ROBOTLIB_DEV_PACKAGE_NAME      "dls-robotlib-dev" CACHE INTERNAL "")
set(CPACK_DEBIAN_ROBOTLIB_DEV_FILE_NAME         "dls-robotlib-dev-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ROBOTLIB_DEV_PACKAGE_DEPENDS   "dls-robotlib" CACHE INTERNAL "")

get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

include(CPack)

# unset(CODE_TO_COMPILE CACHE)	# unset the variable stored in CMakeCache.txt