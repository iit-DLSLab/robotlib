enable_testing()

add_executable(tests
	#EXCLUDE_FROM_ALL
	main_tests.cpp
	modelTest.cpp
	#temporary_test.cpp
	#LegDataMapTest.cpp
)

target_link_libraries(tests
	PRIVATE
	robotlib
	gtest
)


