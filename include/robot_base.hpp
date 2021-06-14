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
#include "trunk.hpp"
#include "leg.hpp"
#include "utils.hpp"

//#include "forward_kinematics.hpp"

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
    
};
} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LIMB_HPP_
