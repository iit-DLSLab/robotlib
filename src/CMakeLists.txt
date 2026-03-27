add_library(robotlib
	SHARED
	joint.cpp
	link.cpp
	trunk.cpp
	frame.cpp
	dyn_params.cpp
	limb.cpp
    robot_base.cpp
	robot.cpp
    jacobian.cpp
	robot_factory.cpp
    utils/utils.cpp
	dummy_robot/dummy_robot_creator.cpp
)

target_link_libraries(robotlib 
		PUBLIC
		Eigen3::Eigen
		PRIVATE 
		urdfdom_model
        urdfdom_model_state
        urdfdom_world
        urdfdom_sensor)

# add_executable(robot_info
# 	${CMAKE_CURRENT_SOURCE_DIR}/robot_info.cpp
# )

# target_link_libraries(robot_info
# 	PRIVATE
# 		${PROJECT_NAME}
# 		${CMAKE_DL_LIBS}
# )

target_include_directories(robotlib 
	PUBLIC
		include
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include/${CMAKE_PROJECT_NAME}>
		$<INSTALL_INTERFACE:lib>
	PRIVATE
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/robotlib>
)

install(
	DIRECTORY 	${PROJECT_SOURCE_DIR}/include/
	DESTINATION /usr/include
	COMPONENT  	"robotlib_dev"
)

install(
	TARGETS 	robotlib
	DESTINATION	/usr/lib
	COMPONENT	"robotlib"
)
