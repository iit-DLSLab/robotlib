#ifndef _ROBOTLIB_ARM_HPP_
#define _ROBOTLIB_ARM_HPP_

// =============================================================================
// Includes
// =============================================================================

#include "arm_base.hpp"

namespace dls{
namespace robot {
/**
 * An arm class for robots.
 */
template<unsigned int NJOINTS, unsigned int NLINKS>
class Arm : public ArmBase<NJOINTS, NLINKS>
{
public:

	Arm(const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :ArmBase<NJOINTS,NLINKS>(name, joints, links) {};

	~Arm();

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_ARM_HPP_

