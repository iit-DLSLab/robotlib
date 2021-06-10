#ifndef _ROBOTLIB_LEG_BASE_HPP_
#define _ROBOTLIB_LEG_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================

#include "limb.hpp"

namespace dls{
namespace robot {
/**
 * A leg interface for robots.
 */

template <unsigned int NJOINTS, unsigned int NLINKS>
class LegBase : public Limb<NJOINTS, NLINKS>
{
public:
    LegBase (const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :Limb<NJOINTS,NLINKS>(name, joints, links) {};
    
	~LegBase(){};

    // Functions for legs (and not for arms) TO DO

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LEG_BASE_HPP_
