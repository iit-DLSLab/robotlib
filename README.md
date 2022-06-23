# Robotlib

## Overview

<!-- TODO -->
<!-- What is and what is the Glue layer -->
<!-- Schemes/images to show connections -->

## Installation and usage

<!-- TODO -->
<!-- How to clone and build Robotlib -->
### Dependencies
Robotlib has been developed and tested on a x86_64 version of Ubuntu 16.04 (Xenial Xerus). The dependencies for building and installing the library are the following:

**CMake** (3.1.0 is the minimum version) - You can download the chosen version and install it through:
* `wget https://cmake.org/files/v3.X/cmake-3.X.X-Linux-x86_64.tar.gz`
* `tar xf cmake-3.X.X-Linux-x86_64.tar.gz`
* `export PATH="$PATH:/home/dls_user/cmake-3.X.X-Linux-x86_64/bin"`

You just need to substitue the *X* value with the chosen CMake version.

**Eigen3**
* `sudo apt install libeigen3-dev`

**GTest**
* `sudo apt install libgtest-dev`

Once installed the *libgtest-dev* library, there could still be an error during the build of Robotlib, due to the missing *libgtest.a* library in */usr/lib/*. A well known solution is the following:
* `sudo apt install libgtest-dev`
* `cd /usr/src/gtest`
* `sudo mkdir build && cd build`
* `sudo cmake .. && make -j$(nproc)`
* `sudo cp libgtest* /usr/lib/`
* `cd .. && rm -rf build`

<!-- How to clone and build Glue layers -->
<!-- How to buid and install it -->
<!-- How to run robot_info -->

## Documentation

<!-- TODO -->
<!-- Doxygen configuration -->
<!-- Doxygen usage and generation -->
<!-- Important nomenclatures if not clear (e.g. RT / NRT) -->

## Tests

<!-- TODO -->
<!-- How to build and run tests -->

## Pipeline status

### Develop

|  **Ubuntu OS**  |  **Build status**  |   **Test coverage**  |
| :-------------: | :------------: | :--------------: |
| Focal Fossa | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/coverage.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) |
| Bionic Beaver | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/coverage.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) |
| Xenial Xerus | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/coverage.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) |

<!-- TODO -->
<!-- Add GCOV/LCOV for test coverage and parsing for gitlab CI/CD badge -->

<!-- TODO -->
<!-- Add the same for (Bionic Beaver + Melodic) and for (Xenial Xerus + Kinetic) -->

### Release
<!-- TODO -->
<!-- Add the same table for the Release branch -->

## Issues

<!-- TODO -->
<!-- You cannot have methods that return <X>DataMap or JointState etc... You need to pass these as reference -->
<!-- Clean issue tracker and make some internal developments (like Agile tasks) private. Put there only known issues for public -->
You can look for known issues, report bugs and ask for features implementation at the [issue tracker](https://gitlab.advr.iit.it/dls-lab/robotlib/-/issues).
