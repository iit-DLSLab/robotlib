set(CMAKE_INSTALL_PREFIX "/usr")

set(ROBOTLIB_INSTALL_HEADERS_DIR ${CMAKE_INSTALL_PREFIX}/include
    CACHE STRING "Directory where the robotlib headers are stored"
)

set(ROBOTLIB_INSTALL_RUNTIME_DIR ${CMAKE_INSTALL_PREFIX}/bin
	CACHE STRING "Directory where the robotlib executables are stored"
)

set(ROBOTLIB_INSTALL_LIBRARIES_DIR ${CMAKE_INSTALL_PREFIX}/lib
	CACHE STRING "Directory where the robotlib shared object libraries are stored"
)

set(ROBOTLIB_INSTALL_CMAKE_LIB_DIR ${ROBOTLIB_INSTALL_LIBRARIES_DIR}/cmake
	CACHE STRING "Directory where the robotlib shared object libraries are stored"
)

set(ROBOTLIB_INSTALL_ROBOTS_DIR ${ROBOTLIB_INSTALL_LIBRARIES_DIR}/robots
	CACHE STRING "Directory where the robotlib shared object libraries are stored"
)

set(EIGEN_DIR ${ROBOTLIB_INSTALL_HEADERS_DIR}/eigen3
	CACHE STRING "Directory where Eigen is installed"
)
# Backward-compatibility cache entry; prefer Eigen3::Eigen imported target.
