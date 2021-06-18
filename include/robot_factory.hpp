#ifndef _ROBOTLIB_ROBOT_FACTORY_HPP_
#define _ROBOTLIB_ROBOT_FACTORY_HPP_

// =============================================================================
// Includes
// =============================================================================
#include <memory>
#include "robot_base.hpp"

// dlopen library for shared libraries
#include <dlfcn.h>

// =============================================================================
// Namespace
// =============================================================================
namespace dls
{
	namespace robotlib
	{

		// =============================================================================
		// Class Description
		// =============================================================================
		/// Factory class
		///
		/// The robot class is really convoluted inside of supervisor. Internal
		/// components depend on each other, therefore it's not simple to just construct
		/// a robot from a constructor. This factory class simplifies that process.
		class RobotFactory
		{
		public:
			static std::shared_ptr<RobotBase> openRobot(const std::string &robotType);
			//static std::shared_ptr<dls::dog::JSSweeper> buildHyQJSSweeper(double pstep);
		};
	} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_ROBOT_FACTORY_HPP_
