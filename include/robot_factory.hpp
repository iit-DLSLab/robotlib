#ifndef _ROBOTLIB_ROBOT_FACTORY_HPP_
#define _ROBOTLIB_ROBOT_FACTORY_HPP_

#include "robot_base.hpp"
#include <memory>
#include <dlfcn.h>

namespace robotlib
{
	/// =============================================================================
	/// Class Description
	/// =============================================================================
	/// Factory class
	///
	/// The robot class is really convoluted inside of supervisor. Internal
	/// components depend on each other, therefore it's not simple to just construct
	/// a robot from a constructor. This factory class simplifies that process.
	class RobotFactory
	{
	public:
		static std::shared_ptr<RobotBase> openRobot(const std::string &robotType);
	};
} // namespace robot

#endif // _ROBOTLIB_ROBOT_FACTORY_HPP_