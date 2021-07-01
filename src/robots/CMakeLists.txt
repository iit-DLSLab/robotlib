add_library(dummy-biped SHARED dummy_biped.cpp)
add_library(dummy-quadruped SHARED dummy_quadruped.cpp)
add_library(dummy-hexapod SHARED dummy_hexapod.cpp)
add_library(dummy-octopod SHARED dummy_octopod.cpp)

target_link_libraries(dummy-biped robotlib)
target_link_libraries(dummy-quadruped robotlib)
target_link_libraries(dummy-hexapod robotlib)
target_link_libraries(dummy-octopod robotlib)

install(
	TARGETS 
		dummy-biped
		dummy-quadruped
		dummy-hexapod
		dummy-octopod
	DESTINATION
		/usr/lib/dls/robots/
)
