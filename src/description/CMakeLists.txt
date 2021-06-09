# TODO this
message(
	WARNING
	"This library installs also some parts to /usr/include/description."
)
install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/src/description/
	DESTINATION
		include/description
	COMPONENT
		"robotlib"
	FILES_MATCHING
		PATTERN "*.urdf"
)
