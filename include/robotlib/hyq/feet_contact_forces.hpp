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
#ifndef _ROBOTLIB_HYQ_FEET_CONTACT_FORCES_HPP_
#define _ROBOTLIB_HYQ_FEET_CONTACT_FORCES_HPP_

#include "robotlib/base/feet_contact_forces.hpp"
#include "robotlib/hyq/feet_jacobians.hpp"
#include "robotlib/hyq/inverse_dynamics.hpp"
#include "robotlib/hyq/jsim.hpp"

namespace dls {
namespace dog {
namespace HyQ {

/**
 * @brief The FeetContactForces class provides the functions to compute
 * the Ground Reaction Forces (GRF) at the HyQ's feet.
 */
class FeetContactForces : public dls::dog::FeetContactForcesBase {
public:
	typedef typename dog::Vector3d Vector3d;
	typedef typename Eigen::Quaterniond Quaterniond;
	typedef typename dog::FeetJacobiansBase FeetJacobians;
	typedef typename dog::JointState JointState;
	typedef typename dog::LegID LegID;

public:
	FeetContactForces
	(
		FeetJacobians &feet_jacobians,
		dog::InverseDynamicsBase &inverse_dynamics,
		dog::JSIMBase &jsim
	);

	inline Vector3d getFootGRF
	(
		const JointState& q,
		const JointState& qd,
		const JointState& tau,
		const Quaterniond& orient,
		const LegID& leg,
		const JointState& qdd = JointState::Zero(),
		const Vector3d& xd = Vector3d::Zero(),
		const Vector3d& xdd = Vector3d::Zero(),
		const Vector3d& omega = Vector3d::Zero(),
		const Vector3d& omegad = Vector3d::Zero()
	);

	bool getFootGRF
	(
		const JointState& q,
		const JointState& qd,
		const JointState& tau,
		const Quaterniond& orient,
		const LegID& leg,
		Vector3d& foot_grf,
		const JointState& qdd = JointState::Zero(),
		const Vector3d& xd = Vector3d::Zero(),
		const Vector3d& xdd = Vector3d::Zero(),
		const Vector3d& omega = Vector3d::Zero(),
		const Vector3d& omegad = Vector3d::Zero()
	);

	inline bool getFeetGRF
	(
		const JointState& q,
		const JointState& qd,
		const JointState& tau,
		const Quaterniond& orient,
		dog::LegDataMap<Vector3d>& feet_grf,
		const JointState& qdd = JointState::Zero(),
		const Vector3d& xd = Vector3d::Zero(),
		const Vector3d& xdd = Vector3d::Zero(),
		const Vector3d& omega = Vector3d::Zero(),
		const Vector3d& omegad = Vector3d::Zero()
	);

	inline dog::LegDataMap<Vector3d> getFeetGRF
	(
		const JointState& q,
		const JointState& qd,
		const JointState& tau,
		const Quaterniond& orient,
		const JointState& qdd = JointState::Zero(),
		const Vector3d& xd = Vector3d::Zero(),
		const Vector3d& xdd = Vector3d::Zero(),
		const Vector3d& omega = Vector3d::Zero(),
		const Vector3d& omegad = Vector3d::Zero()
	);

	/**
	* @brief setContactPoint sets the contact point w.r.t. the center origin of the lowerleg frame  (expressed in  the lowerleg frame)
	* @param foot_x
	* @param foot_y
	*/
	void setContactPoint(dog::LegID leg, double user_foot_x, double user_foot_y);

private:
	dog::FeetJacobiansBase& feet_jacobians_;
	dog::InverseDynamicsBase& inverse_dynamics_;
	dog::JSIMBase& jsim_;
	dog::LegDataMap<double> foot_x = 0.0;
	dog::LegDataMap<double> foot_y = 0.0;
};

} // namespace HyQ
} // namespace dog
} // namespace dls

#endif // _ROBOTLIB_HYQ_FEET_CONTACT_FORCES_HPP_
