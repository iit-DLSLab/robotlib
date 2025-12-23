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
		std::string library{"libglue_code.so"}, lib_path{}; // load generic gluecode library

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
		else if (std::filesystem::exists("/usr/lib/dls2/robots/" + library))
		{
			lib_path = "/usr/lib/dls2/robots/" + library;
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

        return create_robot(robot_type);
	}
} // namespace robotlib