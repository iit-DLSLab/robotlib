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
#ifndef _ROBOTLIB_HYQ_INVERSE_KINEMATICS_HPP_
#define _ROBOTLIB_HYQ_INVERSE_KINEMATICS_HPP_

#include "robotlib/base/leg_data_map.hpp"
#include "robotlib/base/inverse_kinematics.hpp"
#include "robotlib/base/declarations.hpp"
#include "robotlib/base/joint_data_map.hpp"
#include "robotlib/base/joint_bool_map.hpp"
#include "robotlib/base/leg_bool_map.hpp"
#include "robotlib/base/joint_id_tricks.hpp"
#include "robotlib/hyq/jacobians.hpp"

#include "robotlib/hyq/body_lengths.hpp"

namespace dls {
namespace dog {
namespace HyQ {

class InverseKinematics : public dog::InverseKinematicsBase
{
public:
	typedef typename dog::Vector3d FootPosition;
	typedef typename dog::Vector3d FootVelocity;
	typedef typename dog::Vector3d FootAcceleration;

	typedef typename dog::LegDataMap<FootPosition> FootPositions;
	typedef typename dog::LegDataMap<FootVelocity> FootVelocities;
	typedef typename dog::LegDataMap<FootAcceleration> FootAccelerations;

	typedef typename dog::JointState JointState;
	typedef typename dog::LegJointState LegJointState;

public:
	InverseKinematics(dog::KinDynParamsBase &param_getter);
	~InverseKinematics() {}

	bool getJointPosition
	(
		const FootPosition &foot_position,
		const dog::LegID &leg_id,
		LegJointState &q_leg,
		bool clamp,
		dog::LegJointBool& q_violation_leg
	) const;

	bool getJointPosition
	(
		const FootPosition &foot_position,
		const dog::LegID &leg_id,
		LegJointState &q_leg,
		bool clamp = false
	) const;

	bool getJointPosition
	(
		const FootPositions &foot_positions,
		JointState &q,
		bool clamp,
		dog::JointBoolMap& q_violation
	) const;

	bool getJointPosition
	(
		const FootPositions &foot_positions,
		JointState &q,
		bool clamp = false
	) const;

	bool getJointVelocity
	(
		const FootVelocity &foot_velocity,
		const dog::LegID &leg_id,
		const LegJointState &q_leg,
		LegJointState &qd_leg
	);

	bool getJointVelocity
	(
		const FootVelocities &foot_velocities,
		const JointState &q,
		JointState &qd
	);


	bool getJointState
	(
		const FootPosition& foot_position,
		const FootVelocity& foot_velocity,
		const dog::LegID& leg_id,
		LegJointState & q_leg,
		LegJointState& qd_leg,
		bool clamp,
		dog::LegJointBool& q_violation_leg
	);


	bool getJointState
	(
		const FootPosition& foot_position,
		const FootVelocity& foot_velocity,
		const dog::LegID& leg_id,
		LegJointState & q_leg,
		LegJointState& qd_leg,
		bool clamp = false
	);

	bool getJointState
	(
		const FootPosition& foot_position,
		const FootVelocity& foot_velocity,
		const FootAcceleration& foot_acceleration,
		const dog::LegID& leg,
		LegJointState& q,
		LegJointState& qd,
		LegJointState& qdd,
		bool clamp,
		dog::LegJointBool& q_violation_leg
	);

	bool getJointState
	(
		const FootPosition& foot_position,
		const FootVelocity& foot_velocity,
		const FootAcceleration& foot_acceleration,
		const dog::LegID& leg,
		LegJointState& q,
		LegJointState& qd,
		LegJointState& qdd,
		bool clamp = false
	);

	bool getJointState
	(
		const FootPositions &foot_positions,
		const FootVelocities &foot_velocities,
		JointState &q,
		JointState &qd,
		bool clamp,
		dog::JointBoolMap& q_violation
	);


	bool getJointState
	(
		const FootPositions &foot_positions,
		const FootVelocities &foot_velocities,
		JointState &q,
		JointState &qd,
		bool clamp = false
	);

	bool getJointState
	(
		const FootPositions &foot_positions,
		const FootVelocities &foot_velocities,
		const FootAccelerations &foot_accelerations,
		JointState &q,
		JointState &qd,
		JointState &qdd,
		bool clamp,
		dog::JointBoolMap& q_violation
	);

	bool getJointState
	(
		const FootPositions &foot_positions,
		const FootVelocities &foot_velocities,
		const FootAccelerations &foot_accelerations,
		JointState &q,
		JointState &qd,
		JointState &qdd,
		bool clamp = false
	);

	void setTimePeriod(const double& dt);

	void setKinematicLimits
	(
		const JointState& q_min,
		const JointState& q_max
	);

	void setKneeConfiguration
	(
		const dog::LegBoolMap& is_knee_backward
	);

	/**
	* @brief computes the inverse kinematics (position, velocity,
	* acceleration) given foot position, foot velocity and acceleration, and
	* joint state.
	* \deprecated this function is deprecated. Use getJointState() instead.
	* @param des_foot_pos
	* @param des_foot_vel
	* @param des_foot_accel
	* @param des_joint_pos
	* @param des_joint_vel
	* @param des_joint_accel
	* @param q_
	* @return a data structure indicating whether the algorithm succeded (true)
	* or failed (false) for a specific leg
	* @author Victor Barasuol (victor.barasuol@iit.it)
	* @date August 29th, 2014
	*/
	dog::LegBoolMap calculate(const dog::LegDataMap<dls::rbd::Vector3d>& des_foot_pos,

	const dog::LegDataMap<dls::rbd::Vector3d>& des_foot_vel,
	const dog::LegDataMap<dls::rbd::Vector3d>& des_foot_accel,
	dog::LegDataMap<dls::rbd::Vector3d>& des_joint_pos,
	dog::LegDataMap<dls::rbd::Vector3d>& des_joint_vel,
	dog::LegDataMap<dls::rbd::Vector3d>& des_joint_accel,
	const dog::JointState& q_);

private:
	dog::LegDataMap<int> haa_sign_flip;
	dog::LegDataMap<int> kfe_sign_flip;
	dog::LegDataMap<int> haaXOffset_sign_flip;
	dog::LegDataMap<int> haaYOffset_sign_flip;

	mutable HyQLengths bodyLengths;
	HyQ::Jacobians jacobians_;
	double dt_ = 0.004; // default task rate is 4 ms
	dog::LegDataMap<dls::rbd::Matrix33d> old_feet_jacobians_;
	dog::LegDataMap<Eigen::Matrix<double, 6,3 >*> feet_jacobians_; // useful alias

	JointState q_min_;
	JointState q_max_;
};

} // HyQ
} // dog
} // dls
#endif // _ROBOTLIB_HYQ_INVERSE_KINEMATICS_HPP_
