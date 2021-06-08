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
#ifndef DOGLIB_HYQ_FORWARD_KINEMATICS_HPP
#define DOGLIB_HYQ_FORWARD_KINEMATICS_HPP

#include "doglib/base/leg_data_map.hpp"
#include "doglib/base/forward_kinematics.hpp"
#include "doglib/base/declarations.hpp"
#include "doglib/hyq/transforms.hpp"

#include "doglib/rbd/utils.h"

namespace dls {
namespace dog {
namespace HyQ {

class ForwardKinematics : public dog::ForwardKinematicsBase
{
public:
	ForwardKinematics(dog::KinDynParamsBase& param_getter);
	~ForwardKinematics() = default;

	dog::Vector3d getFootPosLF(const dog::JointState& q);
	dog::Vector3d getFootPosRF(const dog::JointState& q);
	dog::Vector3d getFootPosLH(const dog::JointState& q);
	dog::Vector3d getFootPosRH(const dog::JointState& q);
	dog::Vector3d getFootPos(const dog::JointState& q, const dog::LegID& leg);
	dog::Matrix3d getFootOrientation(const dog::JointState &q, const dog::LegID &leg);
	dog::Vector3d getShinPos(const dog::JointState& q, const double& contact_pos, const dog::LegID& leg);

private:
	dog::KinDynParamsBase& param_getter_;
	HyQ::HomogeneousTransforms transforms_;

	typedef  dog::Vector3d(ForwardKinematics::*getter)(const dog::JointState&);

	dog::LegDataMap< getter > myGetters;

};


} // HyQ
} // dog
} // dls


#endif
