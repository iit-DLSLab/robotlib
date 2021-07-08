#include "robot_factory.hpp"

namespace dls
{
	namespace robotlib
	{
		std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type)
		{
			std::string library{"lib" + robot_type + ".so"}, lib_path{};
			std::string lib_robots_dir{"/usr/lib/robots/robotlib/" + library}, lib_current_dir{"./" + library};

			if (dlopen(lib_robots_dir.c_str(), RTLD_LAZY))
			{
				lib_path = lib_robots_dir;
			}
			else if (dlopen(lib_current_dir.c_str(), RTLD_LAZY))
			{
				lib_path = lib_current_dir;
			}
			else
			{
				const std::string error{library + " not found"};
				throw error;
			}

			void *robot = dlopen(lib_path.c_str(), RTLD_LAZY);
			RobotBase::createRobot_t *create_robot = (RobotBase::createRobot_t *)dlsym(robot, "createRobot_t");

			return create_robot();

			/// ANOTHER OPTION

			//std::string library{"lib" + robot_type + ".so"};
			//std::string lib_robots_dir{"/usr/lib/robots/robotlib/" + library}, lib_current_dir{"./" + library};
			//
			//auto lib_path = dlopen(lib_robots_dir.c_str(), RTLD_LAZY) ? lib_robots_dir : lib_current_dir;
			//
			//RobotBase::createRobot_t *create_robot{};
			//
			//if (dlopen(lib_path.c_str(), RTLD_LAZY))
			//{
			//	void *robot = dlopen(lib_path.c_str(), RTLD_LAZY);
			//	create_robot = (RobotBase::createRobot_t *)dlsym(robot, "createRobot_t");
			//}
			//else
			//{
			//	const std::string error{library + " not found"};
			//	throw error;
			//}
			//
			//return create_robot();
		}
	} // namespace robotlib
} // namespace dls
