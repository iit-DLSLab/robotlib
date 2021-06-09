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
#include "robotlib/base/joint_id_tricks.hpp"

namespace dls {
namespace robot {
JointIdentifiers toJointID(LegID leg, LegJoints j)
{
    return orderedJointIDs[ leg * 3 + j ];
}

LegID toLegID(JointIdentifiers j) {
    static LegID legs[] = {LF, RF, LH, RH};
    return legs[ static_cast<int>(j) / 3 ];
}

bool isHAA(JointIdentifiers j) {
    return ((static_cast<int>(j) % 3) == 0);
}

//it assumes the legs are always the first joints
void getLegJointState(LegID leg, const JointState& jstate, dls::rbd::Vector3d& vecout) {
    // WARNING: assuming three joints per leg
    vecout = (jstate.block<3, 1>(leg * 3, 0));
}

dls::rbd::Vector3d getLegJointState(LegID leg, const JointState& jstate) {
    // WARNING: assuming three joints per leg
    return jstate.block<3, 1>(leg * 3, 0);
}

/**
 * @brief setLegJointState sets the portion of a JointState  variable with the
 * joint state of a specific leg.
 * @param[in] leg the identifier of the leg
 * @param[in] vecin the state of the joints of that leg
 * @param[out] jstate the full joint state variable to be set
 */
void setLegJointState(LegID leg, const dls::rbd::Vector3d& vecin,  JointState& jstate)
{
    // WARNING: assuming three joints per leg
    jstate.block<3,1>(leg*3,0) = vecin;
}

bool belongsTo(LegID leg, const JointIdentifiers jointID)
{

    if ((leg==LF) &&
            ((jointID == LF_HAA)||(jointID == LF_HFE)||(jointID == LF_KFE)))
                return true;
    if ((leg==RF) &&
            ((jointID == RF_HAA)||(jointID == RF_HFE)||(jointID == RF_KFE)))
                return true;
    if ((leg==LH) &&
            ((jointID == LH_HAA)||(jointID == LH_HFE)||(jointID == LH_KFE)))
                return true;
    if ((leg==RH) &&
            ((jointID == RH_HAA)||(jointID == RH_HFE)||(jointID == RH_KFE)))
                return true;

    return false;
}

}
}
