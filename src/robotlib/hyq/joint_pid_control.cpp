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
#include "robotlib/hyq/control/joints_pid_control.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ::ctrl;

// =============================================================================
// Constructors
// =============================================================================
JointsPIDControl::JointsPIDControl(double ts, double tf) :
	controller(ts, tf),
	errors(0.0),
	gains(0.0),
	terms(0.0)
{ }

// =============================================================================
// Implementation
// =============================================================================

double JointsPIDControl::getTs() const
{
	return controller.getTs();
}

double JointsPIDControl::getTf() const
{
	return controller.getTf();
}

void JointsPIDControl::setGains(const JointsPIDgains& g)
{
	gains = g;
}

const JointsPIDgains& JointsPIDControl::getGains() const
{
	return gains;
}

const JointsPIDerrors& JointsPIDControl::getErrors() const
{
	return errors;
}

const JointsPIDterms& JointsPIDControl::getTerms() const
{
	return terms;
}

void JointsPIDControl::resetIntegrators()
{
	static dls::dog::JointIdentifiers jid;

	for(int i=0; i<dog::jointsCount; i++)
	{
		jid = dog::orderedJointIDs[i];
		controller.resetIntegrator(terms[jid]);
	}
}


void JointsPIDControl::singleJointPID
(
	dls::dog::JointIdentifiers j,
	double ref,
	double actual
)
{
	errors[j].current = ref - actual;
	controller.pid(gains[j], errors[j], terms[j]);
	errors[j].updatePrevious();
}
