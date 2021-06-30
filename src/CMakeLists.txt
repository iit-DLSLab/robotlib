include_directories(SYSTEM ${roscpp_INCLUDE_DIRS} )

add_library(robotlib
	${CMAKE_CURRENT_SOURCE_DIR}/robot_info.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/robot_factory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/joint_state.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/joint.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/link.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/limb_base.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/pose.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/trunk.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dyn_params.cpp
)

target_link_libraries(robotlib
	PRIVATE
	${roscpp_LIBRARIES}
)

add_executable(robot_info
	${CMAKE_CURRENT_SOURCE_DIR}/robot_info.cpp
)

target_link_libraries(robot_info
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
)

target_include_directories(robotlib PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/robotlib> 
	$<INSTALL_INTERFACE:lib> 
)

install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/include/
	DESTINATION
		include/robotlib
	COMPONENT
		"robotlib"
	FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
		PATTERN "*.tpp"
)

install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/src/
	DESTINATION
		include/robotlib
	COMPONENT
		"robotlib"
	FILES_MATCHING
		PATTERN "*.tpp"
)

install(
	TARGETS 
		robotlib
	EXPORT
		robotlib
	DESTINATION
		/usr/local/lib
	COMPONENT
		"robotlib"
)

install(FILES ../robotlib-config.cmake DESTINATION /usr/lib/cmake/robotlib/)
install(EXPORT robotlib DESTINATION /usr/lib/cmake/robotlib/)


