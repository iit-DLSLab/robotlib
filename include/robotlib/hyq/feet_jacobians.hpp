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
#ifndef _ROBOTLIB_HYQ_FEET_JACOBIANS_HPP_
#define _ROBOTLIB_HYQ_FEET_JACOBIANS_HPP_

// =============================================================================
// Includes
// =============================================================================
#include "robotlib/base/feet_jacobians.hpp"
#include "robotlib/base/leg_data_map.hpp"
#include "robotlib/base/declarations.hpp"
#include "robotlib/base/joint_id_tricks.hpp"
#include "robotlib/hyq/jacobians.hpp"

#include "robotlib/rbd/utils.h"

namespace dls {
namespace dog {
namespace HyQ {

// =============================================================================
// Declaration
// =============================================================================
class FeetJacobians : public dog::FeetJacobiansBase
{
typedef dog::JointState JointState;


public:
	FeetJacobians(HyQ::Jacobians& jacobians);
	dog::FootJac getFootJacobian(const JointState& q, const dog::LegID& leg);
	dog::FootJac getFootJacobianLF(const JointState& q);
	dog::FootJac getFootJacobianRF(const JointState& q);
	dog::FootJac getFootJacobianLH(const JointState& q);
	dog::FootJac getFootJacobianRH(const JointState& q);
	dog::FootJac getFootJacobian(const JointState &q, const dog::LegID &leg, const double& foot_x, const double& foot_y);
	dog::FootJac getAngularFootJacobian(const JointState& q, const dog::LegID& leg);
	dog::FootJac getAngularFootJacobianLF(const JointState& q);
	dog::FootJac getAngularFootJacobianRF(const JointState& q);
	dog::FootJac getAngularFootJacobianLH(const JointState& q);
	dog::FootJac getAngularFootJacobianRH(const JointState& q);

private:
	HyQ::Jacobians& jacs;

	typedef  dog::FootJac(FeetJacobians::*getter)(const JointState&);
	dog::LegDataMap< getter > lin_myGetters, ang_myGetters;
};


} // HyQ
} // dog
} // dls

#endif // _ROBOTLIB_HYQ_FEET_JACOBIANS_HPP_
