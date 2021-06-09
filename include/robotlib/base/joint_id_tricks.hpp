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
#ifndef _ROBOTLIB_BASE_JOINT_ID_TRICKS_HPP_
#define _ROBOTLIB_BASE_JOINT_ID_TRICKS_HPP_


#include "robotlib/rbd/rbd.h"
#include "robotlib/base/declarations.hpp"
#include "robotlib/base/leg_data_map.hpp"

namespace dls
{
namespace robot
{

JointIdentifiers toJointID(LegID leg, LegJoints j);
LegID toLegID(JointIdentifiers j);
bool isHAA(JointIdentifiers j);
void getLegJointState(LegID leg, const JointState& jstate, dls::rbd::Vector3d& vecout);
dls::rbd::Vector3d getLegJointState(LegID leg, const JointState& jstate);
bool belongsTo(LegID leg, const JointIdentifiers jointID);

/**
 * @brief setLegJointState sets the portion of a JointState  variable with the
 * joint state of a specific leg.
 * @param[in] leg the identifier of the leg
 * @param[in] vecin the state of the joints of that leg
 * @param[out] jstate the full joint state variable to be set
 */
void setLegJointState(LegID leg, const dls::rbd::Vector3d& vecin,  JointState& jstate);

} // namespace robot
} // namespace dls
#endif // _ROBOTLIB_BASE_JOINT_ID_TRICKS_HPP_
