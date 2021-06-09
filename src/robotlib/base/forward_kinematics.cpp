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
#include "robotlib/base/forward_kinematics.hpp"
using namespace dls::robot;

LegDataMap<Vector3d> ForwardKinematicsBase::getFeetPos(const JointState &q)
{
	robot::LegDataMap<robot::Vector3d> feetPos(Vector3d::Zero());
	feetPos[robot::LF] = getFootPosLF(q);
	feetPos[robot::RF] = getFootPosRF(q);
	feetPos[robot::LH] = getFootPosLH(q);
	feetPos[robot::RH] = getFootPosRH(q);
	return feetPos;
}
