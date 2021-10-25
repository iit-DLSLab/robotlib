enable_testing()

add_executable(main_tests
	main_tests.cpp
	unit_tests/joint.cpp
	unit_tests/limb.cpp
	unit_tests/link.cpp
	unit_tests/robot_base.cpp
	unit_tests/trunk.cpp
	unit_tests/utils.cpp
	#generic_tests/eigen_map.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)