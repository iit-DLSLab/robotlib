#ifndef _ROBOTLIB_ARM_BASE_HPP_
#define _ROBOTLIB_ARM_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================

#include "limb.hpp"

namespace dls{
namespace robot {
/**
 * An arm interface for robots.
 */

template <unsigned int NJOINTS, unsigned int NLINKS>
class ArmBase : public Limb<NJOINTS, NLINKS>
{
public:
    ArmBase (const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :Limb<NJOINTS,NLINKS>(name, joints, links) {};
    
	~ArmBase(){};

    // Functions for arms (and not for legs) TO DO

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_ARM_BASE_HPP_
