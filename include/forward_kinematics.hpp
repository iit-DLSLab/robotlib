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
#ifndef ROBOTLIB_FORWARD_KINEMATICS_HPP
#define ROBOTLIB_FORWARD_KINEMATICS_HPP

#include "doglib/base/declarations.hpp"
#include "doglib/base/leg_data_map.hpp"

namespace dls {
namespace dog {

/**
 * A forward kinematics interface for quadrupeds.
 */
class ForwardKinematicsBase
{
public:
	virtual ~ForwardKinematicsBase() = default;

	// pure virtual
	virtual Vector3d getFootPosLF(const JointState& q) = 0;
	virtual Vector3d getFootPosRF(const JointState& q) = 0;
	virtual Vector3d getFootPosLH(const JointState& q) = 0;
	virtual Vector3d getFootPosRH(const JointState& q) = 0;
	virtual Vector3d getFootPos(const JointState& q, const LegID& leg) = 0;
	virtual Matrix3d getFootOrientation(const JointState& q, const LegID& leg) = 0;
	virtual Vector3d getShinPos(const JointState& q, const double& contact_pos, const LegID& leg) = 0;

	// virtual
	virtual dog::LegDataMap<Vector3d> getFeetPos  (const JointState& q);
};


} // namespace dog
} // namespace dls


#endif
