#ifndef _ROBOTLIB_LEG_HPP_
#define _ROBOTLIB_LEG_HPP_

// =============================================================================
// Includes
// =============================================================================

#include "leg_base.hpp"

namespace dls{
namespace robot {
/**
 * A leg class for robots.
 */
template<unsigned int NJOINTS, unsigned int NLINKS>
class Leg : public LegBase<NJOINTS, NLINKS>
{
public:

	Leg(const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :LegBase<NJOINTS,NLINKS>(name, joints, links) {};

	~Leg(){};

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LEG_HPP_
