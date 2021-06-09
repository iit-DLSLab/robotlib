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
// stdlib
/*




#include <stdexcept>
#include <fstream>
#include <string>
#include <cerrno>

// current class
#include "robotlib/factory/robot_factory.hpp"

// HyQ
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

// HyQReal
#include "robotlib/hyqreal/declarations.h"
#include "robotlib/hyqreal/default_parameters_getter.h"
#include "robotlib/hyqreal/dynamics_parameters.h"
#include "robotlib/hyqreal/feet_contact_forces.h"
#include "robotlib/hyqreal/feet_jacobians.h"
#include "robotlib/hyqreal/forward_dynamics.h"
#include "robotlib/hyqreal/forward_kinematics.h"
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
#include "robotlib/hyqreal/HyQReal.hpp"
// =============================================================================
// Using declarations
// =============================================================================
using dls::robot::Robot;
using dls::robot::RobotFactory;

// =============================================================================
// Prototypes
// =============================================================================
urdf::Model          init_robot_description(std::string robot_description);
std::shared_ptr<Robot> buildHyQ();
std::shared_ptr<Robot> buildHyQReal();
std::string readFile(const char *filename);

// =============================================================================
// Globals
// =============================================================================
// Yes, this is bad practice, but this library is in the process of being
// refactored. This is a section that needs to be improved upon, but for the
// time being this is the easiest solution
// -----------------------------------------------------------------------------
// HyQ
// -----------------------------------------------------------------------------
static std::shared_ptr<dls::robot::KinDynParamsBase>       pHyQDefaultParamsGetter;
static std::shared_ptr<dls::robot::FeetContactForcesBase>  pHyQFeetContactForces;
static std::shared_ptr<dls::robot::FeetJacobiansBase>      pHyQFeetJacobians;
static std::shared_ptr<dls::robot::ForwardKinematicsBase>  pHyQForwardKinematics;
static std::shared_ptr<dls::robot::InertiaPropertiesBase>  pHyQInertiaProperties;
static std::shared_ptr<dls::robot::InverseDynamicsBase>    pHyQInverseDynamics;
static std::shared_ptr<dls::robot::InverseKinematicsBase>  pHyQInverseKinematics;
static std::shared_ptr<dls::robot::HyQ::Jacobians>         pHyQJacobians;
static std::shared_ptr<dls::robot::JSIMBase>               pHyQJSIM;
static std::shared_ptr<dls::robot::RobotLengthsBase>       pHyQLengths;
static std::shared_ptr<dls::robot::LimitsBase>             pHyQLimits;
static std::shared_ptr<dls::robot::MotionTransformsBase>   pHyQMotionTransforms;
static std::shared_ptr<dls::robot::ShinJacobiansBase>      pHyQShinJacobians;
static std::shared_ptr<dls::robot::ForceTransformsBase>    pHyQTransformForce;

// NEW
static std::shared_ptr<dls::robot::HomogeneousTransformsBase>  pHyQHomogeneousTransforms;
// -----------------------------------------------------------------------------
// HyQReal
// -----------------------------------------------------------------------------
static std::shared_ptr<dls::robot::KinDynParamsBase>       pHyQRealDefaultParamsGetter;
static std::shared_ptr<dls::robot::FeetContactForcesBase>  pHyQRealFeetContactForces;
static std::shared_ptr<dls::robot::FeetJacobiansBase>      pHyQRealFeetJacobians;
static std::shared_ptr<dls::robot::ForwardKinematicsBase>  pHyQRealForwardKinematics;
static std::shared_ptr<dls::robot::InertiaPropertiesBase>  pHyQRealInertiaProperties;
static std::shared_ptr<dls::robot::InverseDynamicsBase>    pHyQRealInverseDynamics;
static std::shared_ptr<dls::robot::InverseKinematicsBase>  pHyQRealInverseKinematics;
static std::shared_ptr<dls::robot::HyQReal::Jacobians>     pHyQRealJacobians;
static std::shared_ptr<dls::robot::JSIMBase>               pHyQRealJSIM;
static std::shared_ptr<dls::robot::RobotLengthsBase>       pHyQRealLengths;
static std::shared_ptr<dls::robot::LimitsBase>             pHyQRealLimits;
static std::shared_ptr<dls::robot::MotionTransformsBase>   pHyQRealMotionTransforms;
static std::shared_ptr<dls::robot::ShinJacobiansBase>      pHyQRealShinJacobians;
static std::shared_ptr<dls::robot::ForceTransformsBase>    pHyQRealTransformForce;

// NEW
static std::shared_ptr<dls::robot::HomogeneousTransformsBase>  pHyQRealHomogeneousTransforms;

// =============================================================================
// Implementiaton
// =============================================================================
std::shared_ptr<Robot> RobotFactory::buildRobot(RobotType robot)
{
	switch(robot)
	{
		case RobotType::HyQ:
		{
			return buildHyQ();
			break;
		}
		case RobotType::HyQReal:
		{
			return buildHyQReal();
			break;
		}
		default:
		{
			throw std::domain_error("Robot type not recognised");
		}
	}
}

std::string readFile(const char *filename){
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (in) {
		std::string contents;
		in.seekg(0,std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0,std::ios::beg);
		in.read(&contents[0],contents.size());
		in.close();
		return contents;
	}
	throw(errno);
}

urdf::Model init_robot_description(std::string robot_description)
{
	urdf::Model robot_model;
	if (!robot_model.initString(robot_description))
	{
		throw std::runtime_error("Failed to parse urdf file");
	}
	return robot_model;
}

std::shared_ptr<Robot> buildHyQ()
{
	using namespace dls::robot::HyQ;
	using namespace dls::robot::HyQ::dyn;
	std::string robot_description = readFile("../../src/description/hyq.urdf"); //TODO 
	auto model = init_robot_description(robot_description);

	pHyQDefaultParamsGetter  =  std::make_shared<DefaultParamsGetter>();
	pHyQLimits               =  std::make_shared<Limits>();
	pHyQLengths              =  std::make_shared<HyQLengths>(*pHyQDefaultParamsGetter.get());
	pHyQShinJacobians        =  std::make_shared<ShinJacobians>(*pHyQDefaultParamsGetter.get());
	pHyQForwardKinematics    =  std::make_shared<ForwardKinematics>(*pHyQDefaultParamsGetter.get());
	pHyQJacobians            =  std::make_shared<Jacobians>(*pHyQDefaultParamsGetter.get());
	pHyQMotionTransforms     =  std::make_shared<MotionTransforms>(*pHyQDefaultParamsGetter.get());
	pHyQInertiaProperties    =  std::make_shared<InertiaProperties>(*pHyQDefaultParamsGetter.get());
	pHyQInverseKinematics    =  std::make_shared<InverseKinematics>(*pHyQDefaultParamsGetter.get());
	pHyQFeetJacobians        =  std::make_shared<FeetJacobians>(*pHyQJacobians.get());
	pHyQTransformForce       =  std::make_shared<ForceTransforms>(*pHyQDefaultParamsGetter.get());

	pHyQJSIM = std::make_shared<JSIM>
	(
		static_cast<InertiaProperties&>(*pHyQInertiaProperties.get()),
		static_cast<ForceTransforms&>(*pHyQTransformForce.get())
	);
	pHyQInverseDynamics = std::make_shared<InverseDynamics>
	(
		static_cast<InertiaProperties&>(*pHyQInertiaProperties.get()),
		static_cast<MotionTransforms&>(*pHyQMotionTransforms.get())
	);
	pHyQFeetContactForces = std::make_shared<FeetContactForces>
	(
		static_cast<FeetJacobians&>(*pHyQFeetJacobians.get()),
		static_cast<InverseDynamics&>(*pHyQInverseDynamics.get()),
		*pHyQJSIM.get()
	);

	// NEW
	pHyQHomogeneousTransforms = std::make_shared<dls::robot::HyQ::HomogeneousTransforms>(*pHyQDefaultParamsGetter.get());

	return std::make_shared<hyq>
	(
		pHyQLengths,
		pHyQFeetContactForces,
		pHyQFeetJacobians,
		pHyQForwardKinematics,
		pHyQInertiaProperties,
		pHyQInverseDynamics,
		pHyQInverseKinematics,
		pHyQJSIM,
		pHyQDefaultParamsGetter,
		pHyQLimits,
		pHyQShinJacobians,

		// NEW
		pHyQHomogeneousTransforms,
		pHyQMotionTransforms,
		pHyQTransformForce,
		model
	);
}

std::shared_ptr<Robot> buildHyQReal()
{
	using namespace dls::robot::HyQReal;
	using namespace dls::robot::HyQReal::dyn;
	std::string robot_description = readFile("../../src/description/hyqreal.urdf"); //TODO
	auto model = init_robot_description(robot_description);

	pHyQRealDefaultParamsGetter  =  std::make_shared<DefaultParamsGetter>();
	pHyQRealLimits               =  std::make_shared<Limits>();
	pHyQRealLengths              =  std::make_shared<RobotLengths>();
	pHyQRealJacobians            =  std::make_shared<Jacobians>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealInertiaProperties    =  std::make_shared<InertiaProperties>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealInverseKinematics    =  std::make_shared<InverseKinematics>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealForwardKinematics    =  std::make_shared<ForwardKinematics>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealTransformForce       =  std::make_shared<ForceTransforms>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealMotionTransforms     =  std::make_shared<MotionTransforms>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealShinJacobians        =  std::make_shared<ShinJacobians>(*pHyQRealDefaultParamsGetter.get());
	pHyQRealFeetJacobians        =  std::make_shared<FeetJacobians>(*pHyQRealJacobians.get());
	pHyQRealFeetContactForces    =  std::make_shared<FeetContactForces>(*pHyQRealFeetJacobians.get(), *pHyQRealInverseDynamics.get(), *pHyQRealJSIM.get());
	pHyQRealFeetJacobians        =  std::make_shared<FeetJacobians>(static_cast<FeetJacobians&>(*pHyQRealFeetJacobians.get()));

	pHyQRealJSIM = std::make_shared<JSIM>
	(
		static_cast<InertiaProperties&>(*pHyQRealInertiaProperties.get()),
		static_cast<ForceTransforms&>(*pHyQRealTransformForce.get())
	);

	pHyQRealInverseDynamics = std::make_shared<InverseDynamics>
	(
		static_cast<InertiaProperties&>(*pHyQRealInertiaProperties.get()),
		static_cast<MotionTransforms&>(*pHyQRealMotionTransforms.get())
	);

	// NEW
	pHyQRealHomogeneousTransforms = std::make_shared<dls::robot::HyQReal::HomogeneousTransforms>(*pHyQRealDefaultParamsGetter.get());


	return std::make_shared<hyqreal>
	(
		pHyQRealLengths,
		pHyQRealFeetContactForces,
		pHyQRealFeetJacobians,
		pHyQRealForwardKinematics,
		pHyQRealInertiaProperties,
		pHyQRealInverseDynamics,
		pHyQRealInverseKinematics,
		pHyQRealJSIM,
		pHyQRealDefaultParamsGetter,
		pHyQRealLimits,
		pHyQRealShinJacobians,

		// NEW
		pHyQRealHomogeneousTransforms,
		pHyQRealMotionTransforms,
		pHyQRealTransformForce,
		model
	);
}





*/