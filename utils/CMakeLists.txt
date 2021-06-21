include_directories(SYSTEM ${roscpp_INCLUDE_DIRS} )

add_library(dummyRobot	
	${CMAKE_CURRENT_SOURCE_DIR}/dummy_robots.cpp
)

target_link_libraries(dummyRobot
	PRIVATE
	${roscpp_LIBRARIES}
)

install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/include/
	DESTINATION
		include
	COMPONENT
		"dummyRobot"
	FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
		PATTERN "*.tpp"
)

install(
	TARGETS
	dummyRobot
	DESTINATION
		/usr/local/lib
	COMPONENT
		"dummyRobot"
)