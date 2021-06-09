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
#ifndef _ROBOTLIB_HYQREAL_HYQREAL
#define _ROBOTLIB_HYQREAL_HYQREAL

#include "robotlib/base/dog.hpp"

// #include "declarations.h"
#include "robotlib/hyqreal/default_parameters_getter.h"
#include "robotlib/hyqreal/dynamics_parameters.h"
#include "robotlib/hyqreal/feet_contact_forces.h"
#include "robotlib/hyqreal/feet_jacobians.h"
#include "robotlib/hyqreal/forward_dynamics.h"
#include "robotlib/hyqreal/forward_kinematics.h"
#include "robotlib/hyqreal/HyQReal.hpp"
#include "robotlib/hyqreal/inertia_properties.h"
#include "robotlib/hyqreal/internals.hpp"
#include "robotlib/hyqreal/inverse_dynamics.h"
#include "robotlib/hyqreal/inverse_kinematics.h"
#include "robotlib/hyqreal/jacobians.h"
#include "robotlib/hyqreal/joint_utils.h"
#include "robotlib/hyqreal/joints_pid_control.h"
#include "robotlib/hyqreal/jsim.h"
#include "robotlib/hyqreal/kinematics_parameters.h"
#include "robotlib/hyqreal/mechanical_constants.h"
#include "robotlib/hyqreal/robot_limits.h"
#include "robotlib/hyqreal/RobotLengths.h"
#include "robotlib/hyqreal/shin_jacobians.h"
#include "robotlib/hyqreal/traits.h"
#include "robotlib/hyqreal/transforms.h"

namespace dls {
namespace dog {
namespace HyQReal {
class hyqreal : public dls::dog::Dog
{
public:
	hyqreal
	(
		std::shared_ptr<dls::dog::RobotLengthsBase>       pRobotLengths,
		std::shared_ptr<dls::dog::FeetContactForcesBase>  pFeetContactForces,
		std::shared_ptr<dls::dog::FeetJacobiansBase>      pFeetJacobians,
		std::shared_ptr<dls::dog::ForwardKinematicsBase>  pForwardKinematics,
		std::shared_ptr<dls::dog::InertiaPropertiesBase>  pInertiaProperties,
		std::shared_ptr<dls::dog::InverseDynamicsBase>    pInverseDynamics,
		std::shared_ptr<dls::dog::InverseKinematicsBase>  pInverseKinematics,
		std::shared_ptr<dls::dog::JSIMBase>               pJSIM,
		std::shared_ptr<dls::dog::KinDynParamsBase>       pKinDynParams,
		std::shared_ptr<dls::dog::LimitsBase>             pLimits,
		std::shared_ptr<dls::dog::ShinJacobiansBase>      pShinJacobians,

		// NEW
		std::shared_ptr<dls::dog::HomogeneousTransformsBase>  pHomogeneousTransforms,
		std::shared_ptr<dls::dog::MotionTransformsBase>       pMotionTransforms,
		std::shared_ptr<dls::dog::ForceTransformsBase>        pForceTransforms,
		const urdf::Model &model
	);

	virtual std::string getName() override;
	virtual int ndof() override { return 12; }
};
} // HyQReal
} // dog
} // dls

#endif /* end of include guard: HYQREAL_HPP_EN0TYZCZ */
