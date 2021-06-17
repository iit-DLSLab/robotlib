// stdlib
#include <stdexcept>
#include <fstream>
#include <string>
#include <cerrno>

// current class
#include "robot_factory.hpp"

using namespace dls::robot;

std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string& robotType){
	
	std::string libPath;

	// Get the library path
	if (robotType.compare("hyq")==0){
		libPath = "../../hyq-commons/hyqlib/build/src/libhyqlib.so";
	}
	else if (robotType.compare("hyqreal")==0){
		libPath = "TO DO";
	}

	// Load the robot library
	void* robot = dlopen(libPath.c_str(), RTLD_LAZY);
	if (!robot) {
		std::cerr << "Cannot load library: " << dlerror() << '\n';
	}

	// Load the create symbol
	RobotBase::createRobot_t* create_robot = (RobotBase::createRobot_t*) dlsym(robot, "createRobot_t");
	const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    }
	
	// // Load the destroy symbol
    // destroyDog_t* destroy_dog = (destroyDog_t*) dlsym(triangle, "destroyDog_t");
    // dlsym_error = dlerror();
    // if (dlsym_error) {
    //     cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
    // }

	// Create an instance of the class
	return create_robot ();
}
