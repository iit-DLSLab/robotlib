enable_testing()

add_executable(main_tests
	main_tests.cpp
	unit_tests/robot_base_tests.cpp
	unit_tests/utils_tests.cpp
	unit_tests/limb_tests.cpp	
	unit_tests/joint_tests.cpp
	unit_tests/link_tests.cpp
	unit_tests/trunk_tests.cpp
	#generic_tests/eigen_map_test.cpp
	#eigen_map_test.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)