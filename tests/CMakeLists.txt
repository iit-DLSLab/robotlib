enable_testing()

# TODO: Solve multiple dummy robots inclusion
#add_executable(main_tests
#	main_tests.cpp
#	unit_tests/arm_unit_tests.cpp
#	unit_tests/dyn_params_unit_tests.cpp
#	unit_tests/joint_unit_tests.cpp
#	unit_tests/leg_unit_tests.cpp
#	unit_tests/limb_unit_tests.cpp
#	unit_tests/link_unit_tests.cpp
#	unit_tests/robot_base_unit_tests.cpp
#	unit_tests/trunk_unit_tests.cpp
#	unit_tests/utils_unit_tests.cpp
#	#generic_tests/eigen_map.cpp
#)

#target_link_libraries(main_tests
#	PRIVATE
#	robotlib
#	${CMAKE_DL_LIBS}
#	gtest
#)

#  TODO: Go back to single executable run from main_tests
# Different executables to avoid dummy robots multiple inclusion error
add_executable(arm_unit_tests
	unit_tests/arm_unit_tests.cpp
)
target_link_libraries(arm_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(dyn_params_unit_tests
	unit_tests/dyn_params_unit_tests.cpp
)
target_link_libraries(dyn_params_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(joint_unit_tests
	unit_tests/joint_unit_tests.cpp
)
target_link_libraries(joint_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(leg_unit_tests
	unit_tests/leg_unit_tests.cpp
)
target_link_libraries(leg_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(limb_unit_tests
	unit_tests/limb_unit_tests.cpp
)
target_link_libraries(limb_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(link_unit_tests
	unit_tests/link_unit_tests.cpp
)
target_link_libraries(link_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(robot_base_unit_tests
	unit_tests/robot_base_unit_tests.cpp
)
target_link_libraries(robot_base_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(trunk_unit_tests
	unit_tests/trunk_unit_tests.cpp
)
target_link_libraries(trunk_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
add_executable(utils_unit_tests
	unit_tests/utils_unit_tests.cpp
)
target_link_libraries(utils_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)