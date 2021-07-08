#include "robot_factory.hpp"

namespace dls
{
	namespace robotlib
	{
		std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type)
		{
			std::string library{"lib" + robot_type + ".so"};
			void *robot{};

			// Look for the robot library in the current directory and load it
			try
			{
				std::string lib_path{"./" + library};
				robot = dlopen(lib_path.c_str(), RTLD_LAZY);
				if (!robot)
					throw dlerror();
			}
			catch (const std::exception &e)
			{
				std::cout << e.what();
			}

			// Look for the robot library in the robots directory and load it
			try
			{
				std::string lib_path{"/usr/lib/robots/robotlib/" + library};
				robot = dlopen(lib_path.c_str(), RTLD_LAZY);
				if (!robot)
					throw dlerror();
			}
			catch (const std::exception &e)
			{
				std::cout << e.what();
			}

			// Load the create symbol
			RobotBase::createRobot_t *create_robot = (RobotBase::createRobot_t *)dlsym(robot, "createRobot_t");
			const char *dlsym_error = dlerror();
			if (dlsym_error)
			{
				std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
			}

			return create_robot();
		}
	} // namespace robotlib
} // namespace dls
