enable_testing()

add_executable(compile_test
	EXCLUDE_FROM_ALL
	compile_test.cpp
)

add_executable(main_tests
	EXCLUDE_FROM_ALL
	main_tests.cpp
	hyq/feet_contact_forces_test.cpp
	hyq/fk.cpp
	hyq/ik.cpp
	hyq/jacs.cpp
	#hyq/leverarm.cpp
	#hyqReal/fbl.cpp
)

add_executable(robotlib_gtests
	#EXCLUDE_FROM_ALL
	robotLib_tests/main_tests.cpp
	#robotLib_tests/temporary_test.cpp
	
	robotLib_tests/modelTest.cpp
	#robotLib_tests/LegDataMapTest.cpp
)

target_link_libraries(compile_test
	PRIVATE
	doglib
	${CMAKE_DL_LIBS}
)

target_link_libraries(main_tests
	PRIVATE
	doglib
	robotlib
	gtest
	hyqlib
	hyqReallib
	${CMAKE_DL_LIBS}
)


target_link_libraries(robotlib_gtests
	PRIVATE
	robotlib
	gtest
)


