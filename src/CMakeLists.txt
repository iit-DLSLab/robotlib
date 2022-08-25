add_library(robotlib
	robot_info.cpp
	robot_factory.cpp
	joint.cpp
	link.cpp
	limb_base.cpp
	trunk.cpp
	dyn_params.cpp
    robot_base.cpp
    joint_state.cpp
    jacobian.cpp
    data_map.cpp
)

target_link_libraries(robotlib PRIVATE)

add_executable(robot_info
	${CMAKE_CURRENT_SOURCE_DIR}/robot_info.cpp
)

target_link_libraries(robot_info
	PRIVATE
	${PROJECT_NAME}
	${CMAKE_DL_LIBS}
)

target_include_directories(robotlib PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${CMAKE_PROJECT_NAME}>
	$<INSTALL_INTERFACE:lib>
)

install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/include/
	DESTINATION
		/usr/include
	COMPONENT
		"robotlib"
	FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.hpp"
		PATTERN "*.tpp"
)

install(
	TARGETS robotlib
	EXPORT robotlib
	DESTINATION
		/usr/lib
	COMPONENT
		"robotlib"
)

# install(FILES ../robotlib-config.cmake DESTINATION ${ROBOTLIB_INSTALL_CMAKE_LIB_DIR}/${CMAKE_PROJECT_NAME})
# install(EXPORT robotlib DESTINATION ${ROBOTLIB_INSTALL_CMAKE_LIB_DIR}/${CMAKE_PROJECT_NAME})

add_subdirectory(robots)
