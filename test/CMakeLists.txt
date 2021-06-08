enable_testing()

add_executable(compile_test
	EXCLUDE_FROM_ALL
	compile_test.cpp
)

target_link_libraries(compile_test
	PRIVATE
	doglib
)

