/*******************************************************************************
*                                                       ,----,                 *
*                                                     .'   .' \                *
*                                                   ,----,'    |               *
*               ________  ___       ________        |    :  .  ;               *
*              |\   ___ \|\  \     |\   ____\       ;    |.'  /                *
*              \ \  \_|\ \ \  \    \ \  \___|_      `----'/  ;                 *
*               \ \  \ \\ \ \  \    \ \_____  \       /  ;  /                  *
*                \ \  \_\\ \ \  \____\|____|\  \     ;  /  /-,                 *
*                 \ \_______\ \_______\____\_\  \   /  /  /.`|                 *
*                  \|_______|\|_______|\_________\./__;      :                 *
*                                     \|_________||   :    .'                  *
*                                                 ;   | .'                     *
*                                                 `---'                        *
********************************************************************************
* Author:            Legacy Code                                               *
* Maintainer:        Hendrik de Bruin                                          *
* Maintainer email:  hendrik.debruin@iit.it                                    *
*******************************************************************************/
#ifndef _ROBOTLIB_BASE_FEET_JACOBIANS_HPP_
#define _ROBOTLIB_BASE_FEET_JACOBIANS_HPP_

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/leg_data_map.hpp"

namespace dls {
namespace dog {

/**
 * An interface for the Jacobian of the feet of quadrupeds.
 */
class FeetJacobiansBase
{
public:
	virtual ~FeetJacobiansBase() {};

	/**
	 * \name Foot Jacobian getters
	 * These functions shall return the 3x3 Jacobian that multiplied by the
	 * velocity of the leg joints yields the linear velocity of the foot,
	 * expressed in the base reference frame.
	 *
	 * There are five getters, one for each leg (LF, RF, LH, RH) and one that
	 * takes the identifier of the leg of interest.
	 */
	///@{
	virtual FootJac getFootJacobianLF(const JointState& q) = 0;
	virtual FootJac getFootJacobianRF(const JointState& q) = 0;
	virtual FootJac getFootJacobianLH(const JointState& q) = 0;
	virtual FootJac getFootJacobianRH(const JointState& q) = 0;
	virtual FootJac getFootJacobian(const JointState& q, const LegID& leg) = 0;

	/**
	 * @brief overload to get the Jacobian on a point of the foot sphere
	 * @param q
	 * @param leg
	 * @param foot_x
	 * @param foot_y
	 * @return
	 */
	virtual FootJac getFootJacobian
	(
		const dog::JointState& q,
		const dog::LegID& leg,
		const double& foot_x,
		const double& foot_y
	) = 0;
	///@}

	/**
	 * \name Foot Jacobian getters
	 * These functions shall return the 3x3 Jacobian that multiplied by the
	 * velocity of the leg joints yields the angular velocity of the foot,
	 * expressed in the base reference frame.
	 *
	 * There are five getters, one for each leg (LF, RF, LH, RH) and one that
	 * takes the identifier of the leg of interest.
	 */
	///@{
	virtual FootJac getAngularFootJacobianLF(const JointState& q) = 0;
	virtual FootJac getAngularFootJacobianRF(const JointState& q) = 0;
	virtual FootJac getAngularFootJacobianLH(const JointState& q) = 0;
	virtual FootJac getAngularFootJacobianRH(const JointState& q) = 0;
	virtual FootJac getAngularFootJacobian(const JointState& q, const LegID& leg) = 0;
	///@}
};
} //namespace dog
} //namespace dls

#endif // _ROBOTLIB_BASE_FEET_JACOBIANS_HPP_
