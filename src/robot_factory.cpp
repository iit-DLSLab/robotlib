#include "robot_factory.hpp"

// TODO: use #include<filesystem> once all dls images are updated with Ubuntu20 (and gcc version > 8)
#include <experimental/filesystem>

namespace robotlib
{
	std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type)
	{
		std::string library{"lib" + robot_type + ".so"}, lib_path{};

		if (std::experimental::filesystem::exists("/usr/lib/robots/" + library))
		{
			lib_path = "/usr/lib/robots/" + library;
		}

		else if (std::experimental::filesystem::exists("./" + library))
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

	std::shared_ptr<RobotBase> RobotFactory::openRobot(const std::string &robot_type, const std::string& robot_urdf)
	{
		std::string library{"lib" + robot_type + ".so"}, lib_path{};

		if (std::experimental::filesystem::exists("/usr/lib/robots/" + library))
		{
			lib_path = "/usr/lib/robots/" + library;
		}

		else if (std::experimental::filesystem::exists("./" + library))
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