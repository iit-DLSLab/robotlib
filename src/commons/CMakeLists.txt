# This library needs to be cleaned up still. That is why it is called "temp*"
add_library(temp_common STATIC
	${CMAKE_CURRENT_SOURCE_DIR}/four_bar_linkage.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/hydraulics.cpp
)
