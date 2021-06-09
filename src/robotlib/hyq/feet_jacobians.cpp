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
#include "robotlib/hyq/feet_jacobians.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ;
using namespace dls;

// =============================================================================
// Constructors
// =============================================================================
FeetJacobians::FeetJacobians(HyQ::Jacobians& jacobians) : jacs(jacobians)
{
	lin_myGetters[dog::LF] = & FeetJacobians::getFootJacobianLF;
	lin_myGetters[dog::RF] = & FeetJacobians::getFootJacobianRF;
	lin_myGetters[dog::LH] = & FeetJacobians::getFootJacobianLH;
	lin_myGetters[dog::RH] = & FeetJacobians::getFootJacobianRH;
	ang_myGetters[dog::LF] = & FeetJacobians::getAngularFootJacobianLF;
	ang_myGetters[dog::RF] = & FeetJacobians::getAngularFootJacobianRF;
	ang_myGetters[dog::LH] = & FeetJacobians::getAngularFootJacobianLH;
	ang_myGetters[dog::RH] = & FeetJacobians::getAngularFootJacobianRH;
}

// =============================================================================
// Implementation
// =============================================================================
dog::FootJac FeetJacobians::getFootJacobian(const JointState& q, const dog::LegID& leg) {
	return ((*this).*(lin_myGetters[leg]))(q);
}

dog::FootJac FeetJacobians::getFootJacobianLF(const JointState& q) {
	jacs.updateParameters();
	return jacs.fr_trunk_J_LF_foot(q).block<3,3>(rbd::LX,0);
}

dog::FootJac FeetJacobians::getFootJacobianRF(const JointState& q) {
	jacs.updateParameters();
	return jacs.fr_trunk_J_RF_foot(q).block<3,3>(rbd::LX,0);
}

dog::FootJac FeetJacobians::getFootJacobianLH(const JointState& q)  {
	jacs.updateParameters();
	return jacs.fr_trunk_J_LH_foot(q).block<3,3>(rbd::LX,0);
}

dog::FootJac FeetJacobians::getFootJacobianRH(const JointState& q)  {
	jacs.updateParameters();
	return jacs.fr_trunk_J_RH_foot(q).block<3,3>(rbd::LX,0);
}


dog::FootJac FeetJacobians::getFootJacobian(const JointState &q, const dog::LegID &leg,
							const double& foot_x, const double& foot_y)
{
   jacs.updateParameters();
   return jacs.getFootJacobianXY(q, leg, foot_x, foot_y);
}


dog::FootJac FeetJacobians::getAngularFootJacobian(const JointState& q, const dog::LegID& leg) {
   return ((*this).*(ang_myGetters[leg]))(q);
}

dog::FootJac FeetJacobians::getAngularFootJacobianLF(const JointState& q) {
   jacs.updateParameters();
   return jacs.fr_trunk_J_LF_foot(q).block<3,3>(rbd::AX,0);
}

dog::FootJac FeetJacobians::getAngularFootJacobianRF(const JointState& q) {
   jacs.updateParameters();
   return jacs.fr_trunk_J_RF_foot(q).block<3,3>(rbd::AX,0);
}

dog::FootJac FeetJacobians::getAngularFootJacobianLH(const JointState& q)  {
   jacs.updateParameters();
   return jacs.fr_trunk_J_LH_foot(q).block<3,3>(rbd::AX,0);
}

dog::FootJac FeetJacobians::getAngularFootJacobianRH(const JointState& q)  {
   jacs.updateParameters();
   return jacs.fr_trunk_J_RH_foot(q).block<3,3>(rbd::AX,0);
}

