enable_testing()

add_executable(main_tests
	main_tests.cpp
	robot_base_unit_tests.cpp
)

target_link_libraries(main_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)
