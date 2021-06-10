/*




#ifndef _ROBOTLIB_FACTORY_ROBOT_FACTORY_HPP_
#define _ROBOTLIB_FACTORY_ROBOT_FACTORY_HPP_
// =============================================================================
// Includes
// =============================================================================
#include <memory>
#include "robotlib/base/robot.hpp"

// =============================================================================
// Namespace
// =============================================================================
namespace dls {
namespace robot {

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
	/// Which robot to construct
	///
	enum class RobotType
	{
		HyQ,
		HyQReal
	};

	static std::shared_ptr<Robot> buildRobot(RobotType);
};
// =============================================================================
} // robot
} // dls
#endif // _ROBOTLIB_FACTORY_ROBOT_FACTORY_HPP_



*/
