/*
 * PIDControl.cpp
 *
 *  Created on: May 22, 2013
 *      Author: mfrigerio
 */

#include "robotlib/hyqreal/joints_pid_control.h"
#include "robotlib/base/declarations.hpp"

using namespace dls::dog::HyQReal;
using namespace dls::dog;
using namespace dls;


void JointsPIDControl::pid(
        const References& ref,
        const States&     actual)
{
    static JointIdentifiers jid;

    for(int i=0; i<jointsCount; i++)
    {
        jid = orderedJointIDs[i];
        singleJointPID(jid, ref[jid], actual[jid]);
    }
}
