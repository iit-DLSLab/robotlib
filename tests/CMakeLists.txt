enable_testing()

add_executable(main_tests
	main_tests.cpp
	robot_base_tests.cpp
	utils_tests.cpp
	limb_tests.cpp
	#eigen_map_test.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)