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
#ifndef _ROBOTLIB_BASE_FEET_CONTACT_FORCES_HPP_
#define _ROBOTLIB_BASE_FEET_CONTACT_FORCES_HPP_

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/leg_data_map.hpp"

#include <Eigen/Dense>

namespace dls {
namespace robot {

/**
 * This class computes the ground reaction forces at the feet, according to the
 * actuated part of the Dynamics Equation of Motion:
 * \f[
 *  \begin{bmatrix}
 I_c & F \\
 F^T & M \\
 \end{bmatrix}
 \begin{bmatrix}
 {}_b{\ddot{\mathbf{x}}}{_b} \\
 {}_b{\dot{\boldsymbol{\omega}}}{_b} \\
 \ddot{\mathbf{q}} \\
 \end{bmatrix} +
 \begin{bmatrix} \mathbf{h}_b \\ \mathbf{h}_q \\ \end{bmatrix} =
 \begin{bmatrix} J_{cb}^T \\- J_{cq}^T \cr \end{bmatrix} \mathbf{f} +
 \begin{bmatrix} \mathbf{0} \\ \boldsymbol{\tau} \\ \end{bmatrix}
 * \f]
 */
class FeetContactForcesBase
{
public:
	typedef typename Eigen::Quaterniond Quaterniond;
	virtual ~FeetContactForcesBase() = default;

	/**/
	/**
	 * @brief getFootGRF computes the Ground Reaction Forces at the foot of a
	 * specific leg, according to the dynamic equation:
	 * \f[
	 * \mathbf{f} =\Big(J_{c,q}\Big)^{\dagger} \Big(-{\tau} +
	 * h_q(\mathbf{q}, \dot{\mathbf{q}}) + F^T(\mathbf{q})
	 * [\ddot{\mathbf{x}}\;\; \dot{{\omega}}]^T +
	 *  M(\mathbf{q})\ddot{\mathbf{q}}\Big)
	 * \f]
	 *
	 * @param[in] q Joint position, (measured by encoders)
	 * @param[in] qd Joint velocity (computed from encoder positions)
	 * @param[in] tau Joint torque (computed/measured by force/torque sensors)
	 * @param[in] orient Base orientation, expressed in world frame as Quaternion
	 * @param[in] leg The leg in which we wish to compute the GRF
	 * @param[in] qdd Joint acceleration (computed from encoder position)
	 * @param[in] xd Base linear velocity (estimated)
	 * @param[in] xdd Base absolute acceleration (computed from IMU).
	 * Note that this is NOT the proper acceleration (i.e., the IMU values)!
	 * @param[in] omega Base angular velocity (measured from IMU)
	 * @param[in] omegad Base angular acceleration (derived from IMU gyro)
	 * @return A Vector3d expressing the GRF at the foot, expressed in the base
	 * frame
	 */
	virtual Vector3d getFootGRF
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
	) = 0;

	/**
	 * @brief same as getFootGRF() but with output as parameter
	 * @param[in] q Joint position, (measured by encoders)
	 * @param[in] qd Joint velocity (computed from encoder positions)
	 * @param[in] tau Joint torque (computed/measured by force/torque sensors)
	 * @param[in] orient Base orientation, expressed in world frame as Quaternion
	 * @param[in] leg The leg in which we wish to compute the GRF
	 * @param[out] foot_grf
	 * @param[in] qdd Joint acceleration (computed from encoder position)
	 * @param[in] xd Base linear velocity (estimated)
	 * @param[in] xdd Base absolute acceleration (computed from IMU).
	 * Note that this is NOT the proper acceleration (i.e., the IMU values)!
	 * @param[in] omega Base angular velocity (measured from IMU)
	 * @param[in] omegad Base angular acceleration (derived from IMU gyro)
	 * @return a boolean indicating whether the computation was successful or
	 * not
	 */
	virtual bool getFootGRF
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
	) = 0;

	/**
	 * @brief same as getFootGRF() but for all legs
	 * @param[in] q Joint position, (measured by encoders)
	 * @param[in] qd Joint velocity (computed from encoder positions)
	 * @param[in] tau Joint torque (computed/measured by force/torque sensors)
	 * @param[in] orient Base orientation, expressed in world frame as Quaternion
	 * @param[out] feet_grf data structure which associates the force at the
	 * end effector for each leg
	 * @param[in] qdd Joint acceleration (computed from encoder position)
	 * @param[in] xd Base linear velocity (estimated)
	 * @param[in] xdd Base absolute acceleration (computed from IMU).
	 * Note that this is NOT the proper acceleration (i.e., the IMU values)!
	 * @param[in] omega Base angular velocity (measured from IMU)
	 * @param[in] omegad Base angular acceleration (derived from IMU gyro)
	 * @return a boolean indicating whether the computation was successful or
	 * not
	 */
	virtual bool getFeetGRF
	(
		const JointState& q,
		const JointState& qd,
		const JointState& tau,
		const Quaterniond& orient,
		LegDataMap<Vector3d>& feet_grf,
		const JointState& qdd = JointState::Zero(),
		const Vector3d& xd = Vector3d::Zero(),
		const Vector3d& xdd = Vector3d::Zero(),
		const Vector3d& omega = Vector3d::Zero(),
		const Vector3d& omegad = Vector3d::Zero()
	) = 0;

	/**
	 * @brief same as getFootGRF() but for all legs
	 * @param[in] q Joint position, (measured by encoders)
	 * @param[in] qd Joint velocity (computed from encoder positions)
	 * @param[in] tau Joint torque (computed/measured by force/torque sensors)
	 * @param[in] orient Base orientation, expressed in world frame as Quaternion
	 * @param[out] feet_grf data structure which associates the force at the
	 * end effector for each leg
	 * @param[in] qdd Joint acceleration (computed from encoder position)
	 * @param[in] xd Base linear velocity (estimated)
	 * @param[in] xdd Base absolute acceleration (computed from IMU).
	 * Note that this is NOT the proper acceleration (i.e., the IMU values)!
	 * @param[in] omega Base angular velocity (measured from IMU)
	 * @param[in] omegad Base angular acceleration (derived from IMU gyro)
	 * @return a data structure which associates the force at the
	 * end effector for each leg
	 */
	virtual LegDataMap<Vector3d> getFeetGRF
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
	) = 0;

	/**
	 * @brief setContactPoint sets the contact point w.r.t. the center of the ball foot (in foot frame)
	 * @param foot_x
	 * @param foot_y
	 */
	virtual void setContactPoint(robot::LegID leg, double foot_x, double foot_y) = 0;
};
} // namespace robot
} // namespace dls


#endif // _ROBOTLIB_BASE_FEET_CONTACT_FORCES_HPP_
