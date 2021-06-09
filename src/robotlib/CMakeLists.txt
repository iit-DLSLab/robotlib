find_package(urdf REQUIRED)
include_directories(SYSTEM ${roscpp_INCLUDE_DIRS})

add_library(robotlib
	# Base
	${CMAKE_CURRENT_SOURCE_DIR}/base/forward_kinematics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/base/joint_id_tricks.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/base/joint_bool_map.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/base/leg_bool_map.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/base/test_fk.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/base/test_k.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/base/dog.cpp

	# HyQ
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/default_parameters_getter.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/feet_contact_forces.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/inverse_kinematics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/inertia_properties.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/forward_kinematics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/cylinder_lever_arm.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/joint_pid_control.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/inverse_dynamics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/shin_jacobians.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/feet_jacobians.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/joint_sweeper.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/robot_limits.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/body_lengths.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/transforms.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/hydraulics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/jacobians.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/internals.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/jsim.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyq/HyQ.cpp

	# HyQReal
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/hydraulics_parameters.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/feet_contact_forces.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/joints_pid_control.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/inverse_kinematics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/inertia_properties.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/hydraulic_dynamics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/inverse_dynamics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/forward_dynamics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/robot_limits.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/transforms.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/jacobians.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/internals.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/dynamics.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/HyQReal.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hyqreal/jsim.cpp

	# Factory
	${CMAKE_CURRENT_SOURCE_DIR}/factory/robot_factory.cpp
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
	very entangled. When robotlib no longer needs to supply the sources of these\
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
install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/include/commons/
	DESTINATION
		include/commons
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
