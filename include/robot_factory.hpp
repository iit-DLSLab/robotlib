/*!
 * @file robot_factory.hpp
 *
 * @brief RobotFactory class definition.
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_ROBOT_FACTORY_HPP_
#define _ROBOTLIB_ROBOT_FACTORY_HPP_

#include "robot_base.hpp"
#include <memory>
#include <dlfcn.h>

namespace robotlib
{
	/*!
	 * @brief RobotFactory class.
	 * @details
	 * The RobotFactory class provides functions to easily create a robot object, without taking care of the robot morphology. It uses the opendl API to dinamically load at run-time the shared library associated to the robot you want to create.
	 */
	class RobotFactory
	{
	public:
		/*!
		 * @brief Open the robot library using opendl.
       	 * @param robot_type name of the robot library to be opened.
		 * @return share pointer pointing to the robot object.
		 */
		static std::shared_ptr<RobotBase> openRobot(const std::string &robot_type);

		/*!
		 * @brief Open the robot library using opendl.
		 * @details
		 * This functions uses a robot urdf to load robot parameters.
       	 * @param robot_type name of the robot library to be opened.
		 * @param robot_urdf robot urdf in string format
		 * @return share pointer pointing to the robot object.
		 */
		static std::shared_ptr<RobotBase> openRobot(const std::string &robot_type, const std::string &robot_urdf);
	};
} // namespace robot

#endif // _ROBOTLIB_ROBOT_FACTORY_HPP_