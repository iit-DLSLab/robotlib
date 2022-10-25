# Robotlib

## Overview
Robotlib is a modular and generic robot software interface that allows the same locomotion framework to run on robots with different morphologies. It was implemented within the project [ANT](https://www.dfki.de/en/web/research/projects-and-publications/project/ant/) as a generic software module to interface with robots having different morphologies: the quadruped Aliengo and the hexapod Crex. For more detail about the project, read the paper [Towards a generic navigation and locomotion control system for legged space exploration](https://az659834.vo.msecnd.net/eventsairwesteuprod/production-atpi-public/7bfd9084af194454bbc98fa9c9d7b648).

Robotlib is based on a factory design pattern that allows the creation of objects (i.e. robot objects) without exposing it to the client, leading to a modular architecture, plug-in based, exploiting polymorphisms and dlopen API. It provides a templated robot morphology that defines the hierarchical structure of limbs with associated kinematic and dynamic data. Additionally, it provides virtual and utility functions for the robot kinematics, dynamics, and jacobians. Each robot-specific library inherits the Robotlib interface to implement the particular morphology, kinematics, and dynamics of each different robot. Thanks to polymorphisms, it is possible to use the interface to access the deepest robot specific implementation of functions declared or defined in Robotlib.

However, polymorphisms per se is not enough to achieve a plug-in based architecture. Here is why dlopen API comes in handy. The dlopen API, allows to dynamically load shared libraries representing the concrete implementation of the interface, i.e. robot specific libraries. This is achieved by defining 2 class factory functions inside the interface, defined as extern "C" to avoid name mangling. One function creates a class instance, the other one allows to destroy it.

With this architecture you just need to:
- Write your controller based on Robotlib
- Implement the Glue code, that is the robot specific libraries
- Loading at runtime the robot library associated to the robot you want to control

Through the common interface, it is therefore possible to keep one single controller implementation for controlling robots with different morphologies.

In the image below you can see an overview of how Robotlib and the robot specific libraries are integrated in a controller framework. Here Aliengolib and Crexlib are the glue code respectively of Aliengo and Crex.

![Robotlib](doxygen/Robotlib.png)

Notice that the robot states are decoupled from the hierarchical robot structure. This means that a robot object does not store any robot state like joint configuration, joint velocities, stance status per leg etc. It provides instead data structures that can be used to define robot states when implementing a controller.

There are several advantages of using an architecture abstraction layer like Robotlib. For example, with Robotlib, robot-specific structures are hidden from the controllers and state estimators, making them more modular and easier to implement. The structure allows controllers and state estimators to be written only once, and then the framework can dynamically load different robots. The abstraction layer also provides an easy way to switch backend libraries that compute the kinematics and dynamics without affecting the rest of the framework.
Robotlib is written in C++17 to be fast and portable. It is compatible with the most adopted robotics libraries and is real-time safe.

## Installation
### Dependencies
Robotlib has been developed and tested on a x86_64 version of Ubuntu 16.04 (Xenial Xerus), Ubunutu 18.04 (Bionic Beaver) and Ubuntu 20.04 (Focal Fossa). The dependencies for building and installing the library are the following:

**CMake** (3.7.0 is the minimum version for Ubuntu 20.04, 3.1.0 is the one for the other versions) - You can download the chosen version and install it through

    wget https://cmake.org/files/v3.X/cmake-3.<X>.<X>-Linux-x86_64.tar.gz
    tar xf cmake-3.<X>.<X>-Linux-x86_64.tar.gz
    export PATH="$PATH:/home/dls_user/cmake-3.<X>.<X>-Linux-x86_64/bin"

You just need to substitue \<X> with the chosen CMake version.

**Eigen3**

    sudo apt install libeigen3-dev

**GTest**

    sudo apt install libgtest-dev

### Building
To build Robotlib, clone this repository
* For Ubuntu 20.04

    git clone git@gitlab.advr.iit.it:dls-lab/robotlib.git

* For Ubuntu 18.04

    git clone git@gitlab.advr.iit.it:dls-lab/robotlib.git -b bionic-develop

* For Ubuntu 16.04

    git clone git@gitlab.advr.iit.it:dls-lab/robotlib.git -b xenial-develop

Then compile the code

    mkdir build

    cd build

    cmake .. -DCMAKE_BUILD_TYPE=Release

    make install

<!-- How to clone and build Glue layers -->
<!-- How to buid and install it -->
<!-- How to run robot_info -->

## Usage
<!--TODO-->
## Documentation

<!-- TODO -->
<!-- Doxygen configuration -->
<!-- Doxygen usage and generation -->
<!-- Important nomenclatures if not clear (e.g. RT / NRT) -->

## Tests
The tests are based on GoogleTests: the Google's C++ test framework. The tests relies on glue codes associated to dummy robots generated with the only purpose of testing.

To run tests

    mkdir build

    cd build

    cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=On

    make install

    make check

Notice that you need to install Robotlib such that the tests can easily loading at run time the shared libraries associated to dummy robots.

## Pipeline status

### Develop

|  **Ubuntu OS**  |  **Build status**  |   **Test coverage - Lines**  |   **Test coverage - Functions**  |
| :-------------: | :----------------: | :--------------------------: | :------------------------------: |
| Focal Fossa | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/focal-develop/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/focal-develop) |
| Bionic Beaver | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/bionic-develop/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/bionic-develop) |
| Xenial Xerus | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/xenial-develop/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/xenial-develop) |

### Release

|  **Stable branch**  |  **Build status**  |   **Test coverage - Lines**  |   **Test coverage - Functions**  |
| :------------------: | :----------------: | :--------------------------: | :------------------------------: |
| master | [![pipeline status](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/master/pipeline.svg)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/master) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/master/coverage.svg?job=coverage-lines)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/master) | [![coverage report](https://gitlab.advr.iit.it/dls-lab/robotlib/badges/master/coverage.svg?job=coverage-functions)](https://gitlab.advr.iit.it/dls-lab/robotlib/-/commits/master) |

<!-- TODO -->
<!-- Actually, master is the release branch for Focal. We need releases for Bionic and Xenial as well -->

## Issues

<!-- TODO -->
<!-- You cannot have methods that return <X>DataMap or JointState etc... You need to pass these as reference -->
<!-- Clean issue tracker and make some internal developments (like Agile tasks) private. Put there only known issues for public -->
You can look for known issues, report bugs and ask for features implementation at the [issue tracker](https://gitlab.advr.iit.it/dls-lab/robotlib/-/issues).
