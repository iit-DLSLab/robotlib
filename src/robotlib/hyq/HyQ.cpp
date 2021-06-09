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
#include "robotlib/hyq/HyQ.hpp"

#include "robotlib/hyq/body_lengths.hpp"
#include "robotlib/hyq/constants.hpp"
#include "robotlib/hyq/cylinder_lever_arm.hpp"
#include "robotlib/hyq/default_parameters_getter.hpp"
#include "robotlib/hyq/dynamics_parameters.hpp"
#include "robotlib/hyq/feet_contact_forces.hpp"
#include "robotlib/hyq/feet_jacobians.hpp"
#include "robotlib/hyq/forward_dynamics.hpp"
#include "robotlib/hyq/forward_kinematics.hpp"
#include "robotlib/hyq/HyQ.hpp"
#include "robotlib/hyq/inertia_properties.hpp"
#include "robotlib/hyq/internals.hpp"
#include "robotlib/hyq/inverse_dynamics.hpp"
#include "robotlib/hyq/inverse_kinematics.hpp"
#include "robotlib/hyq/jacobians.hpp"
#include "robotlib/hyq/joint_state_sweeper.hpp"
#include "robotlib/hyq/jsim.hpp"
#include "robotlib/hyq/kinematics_parameters.hpp"
#include "robotlib/hyq/robot_limits.hpp"
#include "robotlib/hyq/shin_jacobians.hpp"
#include "robotlib/hyq/traits.hpp"
#include "robotlib/hyq/transforms.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ;

// =============================================================================
// Implementation
// =============================================================================
hyq::hyq
(
	std::shared_ptr<RobotLengthsBase>       pRobotLengths,
	std::shared_ptr<FeetContactForcesBase>  pFeetContactForces,
	std::shared_ptr<FeetJacobiansBase>      pFeetJacobians,
	std::shared_ptr<ForwardKinematicsBase>  pForwardKinematics,
	std::shared_ptr<InertiaPropertiesBase>  pInertiaProperties,
	std::shared_ptr<InverseDynamicsBase>    pInverseDynamics,
	std::shared_ptr<InverseKinematicsBase>  pInverseKinematics,
	std::shared_ptr<JSIMBase>               pJSIM,
	std::shared_ptr<KinDynParamsBase>       pKinDynParams,
	std::shared_ptr<LimitsBase>             pLimits,
	std::shared_ptr<ShinJacobiansBase>      pShinJacobians,

	// NEW
	std::shared_ptr<dls::dog::HomogeneousTransformsBase>  pHomogeneousTransforms,
	std::shared_ptr<dls::dog::MotionTransformsBase>       pMotionTransforms,
	std::shared_ptr<dls::dog::ForceTransformsBase>        pForceTransforms,
	const urdf::Model &model
) :
	Dog
	(
		pRobotLengths,
		pFeetContactForces,
		pFeetJacobians,
		pForwardKinematics,
		pInertiaProperties,
		pInverseDynamics,
		pInverseKinematics,
		pJSIM,
		pKinDynParams,
		pLimits,
		pShinJacobians,

		// NEW
		pHomogeneousTransforms,
		pMotionTransforms,
		pForceTransforms,
		model
	)
{ }

std::string hyq::getName()
{
	return "HyQ";
}
