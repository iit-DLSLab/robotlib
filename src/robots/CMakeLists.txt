add_library(dummy-quadruped SHARED dummy_quadruped.cpp)
add_library(dummy-hexapod SHARED dummy_hexapod.cpp)

target_link_libraries(dummy-quadruped ${PROJECT_NAME})
target_link_libraries(dummy-hexapod ${PROJECT_NAME})

install(
	TARGETS 
		dummy-quadruped
		dummy-hexapod
	COMPONENT   ${PROJECT_NAME}
	DESTINATION
		${ROBOTLIB_INSTALL_ROBOTS_DIR}
)