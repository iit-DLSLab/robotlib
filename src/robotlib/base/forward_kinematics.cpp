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
using namespace dls::dog;

LegDataMap<Vector3d> ForwardKinematicsBase::getFeetPos(const JointState &q)
{
	dog::LegDataMap<dog::Vector3d> feetPos(Vector3d::Zero());
	feetPos[dog::LF] = getFootPosLF(q);
	feetPos[dog::RF] = getFootPosRF(q);
	feetPos[dog::LH] = getFootPosLH(q);
	feetPos[dog::RH] = getFootPosRH(q);
	return feetPos;
}
