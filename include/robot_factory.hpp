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

		/**
		 * @brief Open the robot library using opendl
       	 * @param robot_type name of the robot library to be opened
		 * @return std::shared_ptr<RobotBase> : share pointer pointing to the robot object
		 */
		static std::shared_ptr<RobotBase> openRobot(const std::string &robot_type);
		
		/**
		 * @brief Open the robot library using opendl
       	 * @param robot_type name of the robot library to be opened
		 * @param robot_urdf robot urdf in string format (TODO: the robot urdf may not be used when creating the robot object)
		 * @return std::shared_ptr<RobotBase> : share pointer pointing to the robot object
		 */
		static std::shared_ptr<RobotBase> openRobot(const std::string &robot_type, const std::string &robot_urdf);
	};
} // namespace robot

#endif // _ROBOTLIB_ROBOT_FACTORY_HPP_