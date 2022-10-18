add_library(robotlib
	${CMAKE_CURRENT_SOURCE_DIR}/robot_factory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/joint.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/link.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/limb_base.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/trunk.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/dyn_params.cpp
)

target_link_libraries(robotlib PRIVATE)

add_executable(robot_info
	${CMAKE_CURRENT_SOURCE_DIR}/robot_info.cpp
)

target_link_libraries(robot_info
	PRIVATE
	${CMAKE_PROJECT_NAME}
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
		${ROBOTLIB_INSTALL_HEADERS_DIR}/${CMAKE_PROJECT_NAME}
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
		${ROBOTLIB_INSTALL_LIBRARIES_DIR}/${CMAKE_PROJECT_NAME}
	COMPONENT
		"robotlib"
)

install(FILES ../robotlib-config.cmake DESTINATION ${ROBOTLIB_INSTALL_CMAKE_LIB_DIR}/${CMAKE_PROJECT_NAME})
install(EXPORT robotlib DESTINATION ${ROBOTLIB_INSTALL_CMAKE_LIB_DIR}/${CMAKE_PROJECT_NAME})

add_subdirectory(robots)
