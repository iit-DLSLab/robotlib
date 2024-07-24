
add_robotlib_test(TARGET joint_data_map_unit_tests SOURCES unit_tests/joint_data_map_unit_tests.cpp)
add_robotlib_test(TARGET link_data_map_unit_tests SOURCES unit_tests/link_data_map_unit_tests.cpp)
add_robotlib_test(TARGET limb_data_map_unit_tests SOURCES unit_tests/limb_data_map_unit_tests.cpp)

add_robotlib_test(TARGET joint_state_unit_tests SOURCES unit_tests/joint_state_unit_tests.cpp)
add_robotlib_test(TARGET jocobian_unit_tests SOURCES unit_tests/jacobian_unit_tests.cpp)

add_robotlib_test(TARGET dyn_params_unit_tests SOURCES unit_tests/dyn_params_unit_tests.cpp)
add_robotlib_test(TARGET joint_unit_test SOURCES unit_tests/joint_unit_tests.cpp)
add_robotlib_test(TARGET arm_unit_tests SOURCES unit_tests/arm_unit_tests.cpp)
add_robotlib_test(TARGET leg_unit_tests SOURCES unit_tests/leg_unit_tests.cpp)
add_robotlib_test(TARGET limb_unit_tests SOURCES unit_tests/limb_unit_tests.cpp)
add_robotlib_test(TARGET link_unit_tests SOURCES unit_tests/link_unit_tests.cpp)
add_robotlib_test(TARGET robot_base_unit_tests SOURCES unit_tests/robot_base_unit_tests.cpp)
add_robotlib_test(TARGET trunk_unit_tests SOURCES unit_tests/trunk_unit_tests.cpp)
add_robotlib_test(TARGET utils_unit_tests SOURCES unit_tests/utils_unit_tests.cpp)

get_property(my_list_content GLOBAL PROPERTY ALL_TEST_TARGETS)
add_custom_target(check COMMAND make test GTEST_COLOR=1 CTEST_OUTPUT_ON_FAILURE=1 DEPENDS "${my_list_content}" COMMENT "Running tests...")