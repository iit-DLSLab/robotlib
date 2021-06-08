/*
 * PIDControl.cpp
 *
 *  Created on: May 22, 2013
 *      Author: mfrigerio
 */

#include "iit/robots/hyq/control/joints_pid_control.h"

using namespace dls::dog;
using namespace dls::dog::HyQ::ctrl;


void JointsPIDControl::pid(
        const References& ref,
        const States&     actual)
{
    static dls::dog::JointIdentifiers jid;

    for(int i=0; i<dog::jointsCount; i++)
    {
        jid = dog::orderedJointIDs[i];
        singleJointPID(jid, ref[jid], actual[jid]);

    }
}

