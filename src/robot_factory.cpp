#include "robot_factory.hpp"

namespace dls
{
	namespace robotlib
	{
		std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type)
		{
			std::string library{"lib" + robot_type + ".so"};
			void *robot{nullptr};

			/// Look for the robot library in the robots directory and load it
			try
			{
				std::string lib_path{"/usr/lib/robots/robotlib/" + library};

				std::cout << "Look for " << lib_path << " library" << std::endl;

				robot = dlopen(lib_path.c_str(), RTLD_LAZY);

				if (robot)
				{
					std::cout << "Found " << lib_path << std::endl;

					/// Load the create symbol
					RobotBase::createRobot_t *create_robot = (RobotBase::createRobot_t *)dlsym(robot, "createRobot_t");
					return create_robot();
				}
				else
				{
					robot = nullptr;
					const std::string error{dlerror()};
					throw error;
				}
			}
			catch (const std::string &e)
			{
				std::cout << e << std::endl;
			}

			/// Look for the robot library in the current directory and load it
			try
			{
				std::string lib_path{"./" + library};

				std::cout << "Look for " << lib_path << " library" << std::endl;

				robot = dlopen(lib_path.c_str(), RTLD_LAZY);

				if (robot)
				{
					std::cout << "Found " << lib_path << std::endl;

					/// Load the create symbol
					RobotBase::createRobot_t *create_robot = (RobotBase::createRobot_t *)dlsym(robot, "createRobot_t");
					return create_robot();
				}
				else
				{
					robot = nullptr;
					const std::string error{dlerror()};
					throw error;
				}
			}
			catch (const std::string &e)
			{
				std::cout << e << std::endl;
			}

			exit(EXIT_FAILURE);
		}
	} // namespace robotlib
} // namespace dls
