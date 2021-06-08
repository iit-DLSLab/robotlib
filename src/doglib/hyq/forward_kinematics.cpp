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
// =============================================================================
// Includes
// =============================================================================
#include "doglib/hyq/forward_kinematics.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ;
using namespace dls::dog;
using namespace dls;
// using namespace iit;

// =============================================================================
// Cosntructors
// =============================================================================
ForwardKinematics::ForwardKinematics(dog::KinDynParamsBase& param_getter) :
	param_getter_(param_getter), transforms_(param_getter_)
{
	myGetters[dog::LF] = & ForwardKinematics::getFootPosLF;
	myGetters[dog::RF] = & ForwardKinematics::getFootPosRF;
	myGetters[dog::LH] = & ForwardKinematics::getFootPosLH;
	myGetters[dog::RH] = & ForwardKinematics::getFootPosRH;
}

// =============================================================================
// Implementation
// =============================================================================
dog::Vector3d ForwardKinematics::getFootPosLF(const dog::JointState& q)
{
	transforms_.updateParameters();
	return rbd::Utils::positionVector( transforms_.fr_trunk_X_LF_foot(q) );
}

dog::Vector3d ForwardKinematics::getFootPosRF(const dog::JointState& q)
{
	transforms_.updateParameters();
	return rbd::Utils::positionVector( transforms_.fr_trunk_X_RF_foot(q) );
}

dog::Vector3d ForwardKinematics::getFootPosLH(const dog::JointState& q)
{
	transforms_.updateParameters();
	return rbd::Utils::positionVector( transforms_.fr_trunk_X_LH_foot(q) );
}

dog::Vector3d ForwardKinematics::getFootPosRH(const dog::JointState& q)
{
	transforms_.updateParameters();
	return rbd::Utils::positionVector( transforms_.fr_trunk_X_RH_foot(q) );
}

dog::Vector3d ForwardKinematics::getFootPos
(
	const dog::JointState& q, const dog::LegID& leg
)
{
	return ((*this).*(myGetters[leg]))(q);
}

dog::Matrix3d ForwardKinematics::getFootOrientation
(
	const dog::JointState &q, const dog::LegID &leg
)
{
	switch(leg){
	case dog::LF:
		return rbd::Utils::rotationMx(transforms_.fr_trunk_X_LF_foot(q));
	case dog::RF:
		return rbd::Utils::rotationMx(transforms_.fr_trunk_X_RF_foot(q));
	case dog::LH:
		return rbd::Utils::rotationMx(transforms_.fr_trunk_X_LH_foot(q));
	case dog::RH:
		return rbd::Utils::rotationMx(transforms_.fr_trunk_X_RH_foot(q));
	}
	return dog::Matrix3d::Identity();
}

dog::Vector3d ForwardKinematics::getShinPos
(
	const dog::JointState& q,
	const double& contact_pos,
	const dog::LegID& leg
)
{
	switch(leg){
	case dog::LF:
		param_getter_.setValue_LF_shin(contact_pos);
		transforms_.updateParameters();
		return rbd::Utils::positionVector(transforms_.fr_trunk_X_LF_shin(q));
	case dog::RF:
		param_getter_.setValue_RF_shin(contact_pos);
		transforms_.updateParameters();
		return rbd::Utils::positionVector(transforms_.fr_trunk_X_RF_shin(q));
	case dog::LH:
		param_getter_.setValue_LH_shin(contact_pos);
		transforms_.updateParameters();
		return rbd::Utils::positionVector(transforms_.fr_trunk_X_LH_shin(q));
	case dog::RH:
		param_getter_.setValue_RH_shin(contact_pos);
		transforms_.updateParameters();
		return rbd::Utils::positionVector(transforms_.fr_trunk_X_RH_shin(q));
	}
	return dog::Vector3d::Zero();
}
