#include "robot_factory.hpp"
#include <experimental/filesystem> /// TODO: Change it to <filesystem> once the docker image will use gcc/g++-8

namespace dls
{
	namespace robotlib
	{
		std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type)
		{
			std::string library{"lib" + robot_type + ".so"}, lib_path{};

			if (std::experimental::filesystem::exists("/usr/lib/robots/robotlib/" + library)) /// TODO: Change it to std::filesystem once the docker image will use gcc/g++-8
			{
				lib_path = "/usr/lib/robots/robotlib/" + library;
			}
			else if (std::experimental::filesystem::exists("./" + library)) /// TODO: Change it to std::filesystem once the docker image will use gcc/g++-8
			{
				lib_path = "./" + library;
			}
			else
			{
				const std::string error{library + " not found"};
				throw error;
			}

			void *robot = dlopen(lib_path.c_str(), RTLD_LAZY);
			RobotBase::createRobot_t *create_robot = (RobotBase::createRobot_t *)dlsym(robot, "createRobot_t");

			return create_robot();
		}
	} // namespace robotlib
} // namespace dls
