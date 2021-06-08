#ifndef DOGLIB_BASE_SHIN_JACOBIANS_HPP
#define DOGLIB_BASE_SHIN_JACOBIANS_HPP

#include "doglib/base/declarations.hpp"
#include "doglib/base/leg_data_map.hpp"

namespace dls {
namespace dog {

/**
 * An interface for the Jacobian of the shin of quadrupeds.
 */
class ShinJacobiansBase
{
public:
	virtual ~ShinJacobiansBase() = default;

	/**
	 * @brief getShinJacobian
	 * @param q
	 * @param contact_point the contact point along the shin. If 0, it returns
	 * the Jacobian at the knee, if equal to shin length, it returns the
	 * Jacobian at the foot
	 * @param leg
	 * @return
	 */
	virtual FootJac getShinJacobian
	(
		const JointState& q,
		const double& contact_point,
		const LegID& leg
	) = 0;

};

} // namespace dog
} // namespace dls
#endif
