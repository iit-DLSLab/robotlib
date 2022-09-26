enable_testing()

add_executable(main_tests
	main_tests.cpp
	unit_tests/arm_unit_tests.cpp
	unit_tests/data_map_unit_tests.cpp
	unit_tests/dyn_params_unit_tests.cpp
	unit_tests/joint_unit_tests.cpp
	unit_tests/leg_unit_tests.cpp
	unit_tests/limb_unit_tests.cpp
	unit_tests/link_unit_tests.cpp
	unit_tests/robot_base_unit_tests.cpp
	unit_tests/trunk_unit_tests.cpp
	unit_tests/utils_unit_tests.cpp
	#real_time_tests/eigen_tests.cpp
	real_time_tests/jacobian_tests.cpp
	real_time_tests/robot_base_tests.cpp
	generic_tests/eigen_map.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)