# Robotlib

## Overview

<!-- TODO -->
<!-- What is and what is the Glue layer -->
<!-- Schemes/images to show connections -->

## Installation and usage

<!-- TODO -->
<!-- How to clone and build Robotlib -->
### Dependencies
Robotlib has been developed and tested on a x86_64 version of Ubuntu 20.04 (Focal Fossa). The dependencies for building and installing the library are the following:

**CMake** (3.7.0 is the minimum version) - You can download the chosen version and install it through:
* `wget https://cmake.org/files/v3.X/cmake-3.X.X-Linux-x86_64.tar.gz`
* `tar xf cmake-3.X.X-Linux-x86_64.tar.gz`
* `export PATH="$PATH:/home/dls_user/cmake-3.X.X-Linux-x86_64/bin"`

You just need to substitue the *X* value with the chosen CMake version.

**Eigen3**
* `sudo apt install libeigen3-dev` 

**GTest**
* `sudo apt install libgtest-dev`

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

|  **Ubuntu OS**  |  **Build status**  |   **Test coverage - Lines**  |   **Test coverage - Functions**  |
| :-------------: | :----------------: | :--------------------------: | :------------------------------: |
| Focal Fossa | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) |
| Bionic Beaver | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) |
| Xenial Xerus | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) |

### Release
<!-- TODO -->
<!-- Add the same table for the master branch? (that is updated from focal-develop) -->
<!-- Add the same table for the Release branch -->

## Issues

<!-- TODO -->
<!-- You cannot have methods that return <X>DataMap or JointState etc... You need to pass these as reference -->
<!-- Clean issue tracker and make some internal developments (like Agile tasks) private. Put there only known issues for public -->
You can look for known issues, report bugs and ask for features implementation at the [issue tracker](https://gitlab.advr.iit.it/dls-lab/robotlib/-/issues).
