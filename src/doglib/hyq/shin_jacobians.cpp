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
// Include
// =============================================================================
#include "doglib/hyq/shin_jacobians.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ;
using namespace dls::dog;
using namespace dls;

// =============================================================================
// Constructors
// =============================================================================
ShinJacobians::ShinJacobians(dog::KinDynParamsBase& param_getter) :
	param_getter_(param_getter),
	jacobians_(param_getter_)
{ }

// =============================================================================
// Implementation
// =============================================================================
dog::FootJac ShinJacobians::getShinJacobian
(
	const JointState& q,
	const double& contact_point,
	const dog::LegID& leg
)
{
	switch(leg)
	{
		case dog::LF:
			param_getter_.setValue_LF_shin(contact_point);
			jacobians_.updateParameters();
			return jacobians_.fr_trunk_J_LF_shin(q).block<3,3>(rbd::LX,0);
		case dog::RF:
			param_getter_.setValue_RF_shin(contact_point);
			jacobians_.updateParameters();
			return jacobians_.fr_trunk_J_RF_shin(q).block<3,3>(rbd::LX,0);
		case dog::LH:
			param_getter_.setValue_LH_shin(contact_point);
			jacobians_.updateParameters();
			return jacobians_.fr_trunk_J_LH_shin(q).block<3,3>(rbd::LX,0);
		case dog::RH:
			param_getter_.setValue_RH_shin(contact_point);
			jacobians_.updateParameters();
			return jacobians_.fr_trunk_J_RH_shin(q).block<3,3>(rbd::LX,0);
	}
	return dog::FootJac::Identity();
}
