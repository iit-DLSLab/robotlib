find_package(urdf REQUIRED)
include_directories(SYSTEM ${roscpp_INCLUDE_DIRS} )

add_library(robotlib	
	${CMAKE_CURRENT_SOURCE_DIR}/robot.cpp
)

target_link_libraries(robotlib
	PRIVATE
	temp_common
	${roscpp_LIBRARIES}
	${urdf_LIBRARIES}
)

# TODO this
message(
	WARNING
	"This library installs also some parts to /usr/include/commons and\
	/usr/include/rbd. This is due to the fact that the original libraries were\
	very entangled. When doglib no longer needs to supply the sources of these\
	libraries, then the install of the headers should also be removed from\
	here."
)

install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/include/robotlib/
	DESTINATION
		include/robotlib
	COMPONENT
		"robotlib"
	FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
		PATTERN "*.tpp"
)
