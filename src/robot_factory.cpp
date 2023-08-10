/**
 * @file robot_factory.cpp
 *
 * @brief RobotFactory class and functions implementation
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "robot_factory.hpp"
#include <filesystem>

namespace robotlib
{
	std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type)
	{
		std::string library{"lib" + robot_type + ".so"}, lib_path{};

		if (std::filesystem::exists("/usr/lib/" + library))
		{
			lib_path = "/usr/lib/" + library;
		}
		else if (std::filesystem::exists("./" + library))
		{
			lib_path = "./" + library;
		}
		else if (std::filesystem::exists("/usr/lib/robots/" + library))
		{
			lib_path = "/usr/lib/robots/" + library;
		}
		else
		{
			const std::string error{"RobotFactory: " + library + " not found"};
            std::cout << "### " << error << " ###" << std::endl;
			throw std::runtime_error(error);
		}

		void *T_lib = dlopen(lib_path.c_str(), RTLD_LAZY);
        if(!T_lib)
	    {
		    std::stringstream ss;
		    ss << "Error: could not load object " << lib_path << ": " << dlerror();
		    std::cout << ss.str() << std::endl;
		    return nullptr;
	    }

        dlerror();

	    RobotBase::createRobot_t *create_robot = (RobotBase::createRobot_t *)dlsym(T_lib, "createRobot_t");

        if(!create_robot)
        {
            std::stringstream ss;
            ss	<< "Could not create robot " << robot_type << dlerror();
            // std::cout << ss.str() << std::endl;
            throw std::runtime_error(ss.str());
        }

        return create_robot();
	}

	std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type, const std::string& robot_urdf)
	{
		std::string library{"lib" + robot_type + ".so"}, lib_path{};

		if (std::filesystem::exists("/usr/lib/" + library))
		{
			lib_path = "/usr/lib/" + library;
		}

		else if (std::filesystem::exists("./" + library))
		{
			lib_path = "./" + library;
		}
		else
		{
			const std::string error{library + " not found"};
			throw error;
		}

		void *robot = dlopen(lib_path.c_str(), RTLD_LAZY);
		RobotBase::createRobotWithUrdf_t *create_robot_with_urdf = (RobotBase::createRobotWithUrdf_t *)dlsym(robot, "createRobotWithUrdf_t");

		return create_robot_with_urdf(robot_urdf);
	}
} // namespace robotlib