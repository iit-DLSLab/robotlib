find_package(urdf REQUIRED)
include_directories(SYSTEM ${roscpp_INCLUDE_DIRS} )

add_library(robotlib	
	${CMAKE_CURRENT_SOURCE_DIR}/robot.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/robot_factory.cpp
)

target_link_libraries(robotlib
	PRIVATE
#	${roscpp_LIBRARIES}
	${urdf_LIBRARIES}
)

install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/include/
	DESTINATION
		include
	COMPONENT
		"robotlib"
	FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
		PATTERN "*.tpp"
)

install(
	TARGETS
		robotlib
	DESTINATION
		/usr/local/lib
	COMPONENT
		"robotlib"
)

#install(TARGETS robotlib
#  EXPORT robotlib
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib)
#install(EXPORT robotlib DESTINATION lib)
