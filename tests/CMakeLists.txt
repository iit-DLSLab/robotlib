add_robotlib_test(TARGET arm_unit_tests SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/arm_unit_tests.cpp)
# add_robotlib_test(TARGET data_map_unit_tests SOURCES unit_tests/data_map_unit_tests.cpp)
# add_robotlib_test(TARGET dyn_params_unit_tests SOURCES unit_tests/dyn_params_unit_tests.cpp)
# add_robotlib_test(TARGET joint_unit_test SOURCES unit_tests/joint_unit_tests.cpp)
# add_robotlib_test(TARGET leg_unit_tests SOURCES unit_tests/leg_unit_tests.cpp)
# add_robotlib_test(TARGET limb_unit_tests SOURCES unit_tests/limb_unit_tests.cpp)
# add_robotlib_test(TARGET link_unit_tests SOURCES unit_tests/link_unit_tests.cpp)
# add_robotlib_test(TARGET robot_base_unit_tests SOURCES unit_tests/robot_base_unit_tests.cpp)
# add_robotlib_test(TARGET trunk_unit_tests SOURCES unit_tests/trunk_unit_tests.cpp)
# add_robotlib_test(TARGET utils_unit_tests SOURCES unit_tests/utils_unit_tests.cpp)

# get_property(my_list_content GLOBAL PROPERTY ALL_TEST_TARGETS)
# add_custom_target(check COMMAND make test DEPENDS "${my_list_content}" COMMENT "Running tests...")

# message(STATUS "#################################### ${my_list_content}")
# SET(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage -pthread")
# # SET(CMAKE_C_FLAGS "-g -O0 -Wall -W -fprofile-arcs -ftest-coverage -pthread")
# SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage -pthread")

# add_executable(main_tests
# 	# main_tests.cpp
# 	unit_tests/arm_unit_tests.cpp
# 	# unit_tests/data_map_unit_tests.cpp
# 	# unit_tests/dyn_params_unit_tests.cpp
# 	# unit_tests/joint_unit_tests.cpp
# 	# unit_tests/leg_unit_tests.cpp
# 	# unit_tests/limb_unit_tests.cpp
# 	# unit_tests/link_unit_tests.cpp
# 	# unit_tests/joint_state_unit_tests.cpp
# 	# unit_tests/robot_base_unit_tests.cpp
# 	# unit_tests/trunk_unit_tests.cpp
# 	# unit_tests/utils_unit_tests.cpp
# 	#real_time_tests/eigen_tests.cpp
# 	# real_time_tests/jacobian_tests.cpp
# 	#real_time_tests/robot_base_tests.cpp
# 	# generic_tests/eigen_map.cpp
# )

# target_link_libraries(main_tests
# 	PRIVATE
# 	robotlib
# 	${CMAKE_DL_LIBS}
# 	gtest
# )