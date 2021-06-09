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
* Author:            Hendrik de Bruin                                          *
* Maintainer:        Hendrik de Bruin                                          *
* author email:      hendrik.debruin@iit.it                                    *
*******************************************************************************/
#ifndef _ROBOTLIB_BASE_ROBOT_HPP_
#define _ROBOTLIB_BASE_ROBOT_HPP_
// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

#include <memory>
#include "robotlib/base/body_lengths.hpp"
#include "robotlib/base/declarations.hpp"
#include "robotlib/base/feet_contact_forces.hpp"
#include "robotlib/base/feet_jacobians.hpp"
#include "robotlib/base/forward_kinematics.hpp"
#include "robotlib/base/inverse_kinematics.hpp"
#include "robotlib/base/inertia_properties.hpp"
#include "robotlib/base/inverse_dynamics.hpp"
#include "robotlib/base/jsim.hpp"
#include "robotlib/base/kin_dyn_params.hpp"
#include "robotlib/base/robot_limits.hpp"
#include "robotlib/base/shin_jacobians.hpp"
#include "robotlib/base/transforms.hpp"

namespace dls
{
namespace robot
{
// =============================================================================
// Class Interface
// =============================================================================
class Robot
{
public:
	// Constructor
	Robot
	(
		std::shared_ptr<dls::robot::RobotLengthsBase>      pRobotLengths,
		std::shared_ptr<dls::robot::FeetContactForcesBase> pFeetContactForces,
		std::shared_ptr<dls::robot::FeetJacobiansBase>     pFeetJacobians,
		std::shared_ptr<dls::robot::ForwardKinematicsBase> pForwardKinematics,
		std::shared_ptr<dls::robot::InertiaPropertiesBase> pInertiaProperties,
		std::shared_ptr<dls::robot::InverseDynamicsBase>   pInverseDynamics,
		std::shared_ptr<dls::robot::InverseKinematicsBase> pInverseKinematics,
		std::shared_ptr<dls::robot::JSIMBase>              pJSIM,
		std::shared_ptr<dls::robot::KinDynParamsBase>      pKinDynParams,
		std::shared_ptr<dls::robot::LimitsBase>            pLimits,
		std::shared_ptr<dls::robot::ShinJacobiansBase>     pShinJacobians,

		// NEW
	std::shared_ptr<dls::robot::HomogeneousTransformsBase>  pHomogeneousTransforms,
	std::shared_ptr<dls::robot::MotionTransformsBase>       pMotionTransforms,
	std::shared_ptr<dls::robot::ForceTransformsBase>        pForceTransforms,
	const urdf::Model                                &model
	);

	virtual ~Robot() = default;

	std::shared_ptr<dls::robot::RobotLengthsBase>       getRobotLengths();
	std::shared_ptr<dls::robot::FeetContactForcesBase>  getFeetContactforces();
	std::shared_ptr<dls::robot::FeetJacobiansBase>      getFeetJacobians();
	std::shared_ptr<dls::robot::ForwardKinematicsBase>  getForwardKinematics();
	std::shared_ptr<dls::robot::InertiaPropertiesBase>  getInertiaProperties();
	std::shared_ptr<dls::robot::InverseDynamicsBase>    getInverseDynamics();
	std::shared_ptr<dls::robot::InverseKinematicsBase>  getInverseKinematics();
	std::shared_ptr<dls::robot::JSIMBase>               getJSIM();
	std::shared_ptr<dls::robot::KinDynParamsBase>       getKinDynParams();
	std::shared_ptr<dls::robot::LimitsBase>             getLimits();
	std::shared_ptr<dls::robot::ShinJacobiansBase>      getShinJacobians();

	virtual std::string getName()  = 0;
	virtual int ndof() = 0;
	// NEW
	std::shared_ptr<dls::robot::HomogeneousTransformsBase> getHomogeneousTransforms();
	std::shared_ptr<dls::robot::MotionTransformsBase>      getMotionTransforms();
	std::shared_ptr<dls::robot::ForceTransformsBase>       getForceTransforms();
	urdf::Model                                          getModel();

private:
	std::shared_ptr<dls::robot::RobotLengthsBase>       pRobotLengths;
	std::shared_ptr<dls::robot::FeetContactForcesBase>  pFeetContactForces;
	std::shared_ptr<dls::robot::FeetJacobiansBase>      pFeetJacobians;
	std::shared_ptr<dls::robot::ForwardKinematicsBase>  pForwardKinematics;
	std::shared_ptr<dls::robot::InertiaPropertiesBase>  pInertiaProperties;
	std::shared_ptr<dls::robot::InverseDynamicsBase>    pInverseDynamics;
	std::shared_ptr<dls::robot::InverseKinematicsBase>  pInverseKinematics;
	std::shared_ptr<dls::robot::JSIMBase>               pJSIM;
	std::shared_ptr<dls::robot::KinDynParamsBase>       pKinDynParams;
	std::shared_ptr<dls::robot::LimitsBase>             pLimits;
	std::shared_ptr<dls::robot::ShinJacobiansBase>      pShinJacobians;

	// NEW
	std::shared_ptr<dls::robot::HomogeneousTransformsBase>  pHomogeneousTransforms;
	std::shared_ptr<dls::robot::MotionTransformsBase>       pMotionTransforms;
	std::shared_ptr<dls::robot::ForceTransformsBase>        pForceTransforms;

	urdf::Model robot_model;
};
} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_BASE_ROBOT_HPP_
