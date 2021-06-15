#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

// stdlib
#include <fstream>

#include <memory>

#include "limb_base.hpp"
#include "leg.hpp"
#include "link.hpp"
#include "joint.hpp"
#include "trunk.hpp"
#include "leg_data_map.hpp"

#include "utils.hpp"

namespace dls
{
namespace robot
{

// =============================================================================
// Class Interface
// =============================================================================

class RobotBase {
public:

    virtual Iterator<const std::shared_ptr<LimbBase>> begin() {};
    virtual Iterator<const std::shared_ptr<LimbBase>> end() {}; 
    
    virtual int getNLEGS() = 0;

	// Plugin typedefs
	typedef std::shared_ptr<RobotBase> createRobot_t();
	typedef void destroyRobot_t(std::shared_ptr<RobotBase>);
    
};
} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LIMB_HPP_
