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
#ifndef DOGLIB_HYQREAL_HYQREAL
#define DOGLIB_HYQREAL_HYQREAL

#include "doglib/base/dog.hpp"

// #include "declarations.h"
#include "doglib/hyqreal/default_parameters_getter.h"
#include "doglib/hyqreal/dynamics_parameters.h"
#include "doglib/hyqreal/feet_contact_forces.h"
#include "doglib/hyqreal/feet_jacobians.h"
#include "doglib/hyqreal/forward_dynamics.h"
#include "doglib/hyqreal/forward_kinematics.h"
#include "doglib/hyqreal/HyQReal.hpp"
#include "doglib/hyqreal/inertia_properties.h"
#include "doglib/hyqreal/internals.hpp"
#include "doglib/hyqreal/inverse_dynamics.h"
#include "doglib/hyqreal/inverse_kinematics.h"
#include "doglib/hyqreal/jacobians.h"
#include "doglib/hyqreal/joint_utils.h"
#include "doglib/hyqreal/joints_pid_control.h"
#include "doglib/hyqreal/jsim.h"
#include "doglib/hyqreal/kinematics_parameters.h"
#include "doglib/hyqreal/mechanical_constants.h"
#include "doglib/hyqreal/robot_limits.h"
#include "doglib/hyqreal/RobotLengths.h"
#include "doglib/hyqreal/shin_jacobians.h"
#include "doglib/hyqreal/traits.h"
#include "doglib/hyqreal/transforms.h"

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
