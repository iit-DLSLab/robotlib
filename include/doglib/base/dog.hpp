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
#ifndef DOGLIB_BASE_DOG_HPP
#define DOGLIB_BASE_DOG_HPP
// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

#include <memory>
#include "doglib/base/body_lengths.hpp"
#include "doglib/base/declarations.hpp"
#include "doglib/base/feet_contact_forces.hpp"
#include "doglib/base/feet_jacobians.hpp"
#include "doglib/base/forward_kinematics.hpp"
#include "doglib/base/inverse_kinematics.hpp"
#include "doglib/base/inertia_properties.hpp"
#include "doglib/base/inverse_dynamics.hpp"
#include "doglib/base/jsim.hpp"
#include "doglib/base/kin_dyn_params.hpp"
#include "doglib/base/robot_limits.hpp"
#include "doglib/base/shin_jacobians.hpp"
#include "doglib/base/transforms.hpp"

namespace dls
{
namespace dog
{
// =============================================================================
// Class Interface
// =============================================================================
class Dog
{
public:
	// Constructor
	Dog
	(
		std::shared_ptr<dls::dog::RobotLengthsBase>      pRobotLengths,
		std::shared_ptr<dls::dog::FeetContactForcesBase> pFeetContactForces,
		std::shared_ptr<dls::dog::FeetJacobiansBase>     pFeetJacobians,
		std::shared_ptr<dls::dog::ForwardKinematicsBase> pForwardKinematics,
		std::shared_ptr<dls::dog::InertiaPropertiesBase> pInertiaProperties,
		std::shared_ptr<dls::dog::InverseDynamicsBase>   pInverseDynamics,
		std::shared_ptr<dls::dog::InverseKinematicsBase> pInverseKinematics,
		std::shared_ptr<dls::dog::JSIMBase>              pJSIM,
		std::shared_ptr<dls::dog::KinDynParamsBase>      pKinDynParams,
		std::shared_ptr<dls::dog::LimitsBase>            pLimits,
		std::shared_ptr<dls::dog::ShinJacobiansBase>     pShinJacobians,

		// NEW
	std::shared_ptr<dls::dog::HomogeneousTransformsBase>  pHomogeneousTransforms,
	std::shared_ptr<dls::dog::MotionTransformsBase>       pMotionTransforms,
	std::shared_ptr<dls::dog::ForceTransformsBase>        pForceTransforms,
	const urdf::Model                                &model
	);

	virtual ~Dog() = default;

	std::shared_ptr<dls::dog::RobotLengthsBase>       getRobotLengths();
	std::shared_ptr<dls::dog::FeetContactForcesBase>  getFeetContactforces();
	std::shared_ptr<dls::dog::FeetJacobiansBase>      getFeetJacobians();
	std::shared_ptr<dls::dog::ForwardKinematicsBase>  getForwardKinematics();
	std::shared_ptr<dls::dog::InertiaPropertiesBase>  getInertiaProperties();
	std::shared_ptr<dls::dog::InverseDynamicsBase>    getInverseDynamics();
	std::shared_ptr<dls::dog::InverseKinematicsBase>  getInverseKinematics();
	std::shared_ptr<dls::dog::JSIMBase>               getJSIM();
	std::shared_ptr<dls::dog::KinDynParamsBase>       getKinDynParams();
	std::shared_ptr<dls::dog::LimitsBase>             getLimits();
	std::shared_ptr<dls::dog::ShinJacobiansBase>      getShinJacobians();

	virtual std::string getName()  = 0;
	virtual int ndof() = 0;
	// NEW
	std::shared_ptr<dls::dog::HomogeneousTransformsBase> getHomogeneousTransforms();
	std::shared_ptr<dls::dog::MotionTransformsBase>      getMotionTransforms();
	std::shared_ptr<dls::dog::ForceTransformsBase>       getForceTransforms();
	urdf::Model                                          getModel();

private:
	std::shared_ptr<dls::dog::RobotLengthsBase>       pRobotLengths;
	std::shared_ptr<dls::dog::FeetContactForcesBase>  pFeetContactForces;
	std::shared_ptr<dls::dog::FeetJacobiansBase>      pFeetJacobians;
	std::shared_ptr<dls::dog::ForwardKinematicsBase>  pForwardKinematics;
	std::shared_ptr<dls::dog::InertiaPropertiesBase>  pInertiaProperties;
	std::shared_ptr<dls::dog::InverseDynamicsBase>    pInverseDynamics;
	std::shared_ptr<dls::dog::InverseKinematicsBase>  pInverseKinematics;
	std::shared_ptr<dls::dog::JSIMBase>               pJSIM;
	std::shared_ptr<dls::dog::KinDynParamsBase>       pKinDynParams;
	std::shared_ptr<dls::dog::LimitsBase>             pLimits;
	std::shared_ptr<dls::dog::ShinJacobiansBase>      pShinJacobians;

	// NEW
	std::shared_ptr<dls::dog::HomogeneousTransformsBase>  pHomogeneousTransforms;
	std::shared_ptr<dls::dog::MotionTransformsBase>       pMotionTransforms;
	std::shared_ptr<dls::dog::ForceTransformsBase>        pForceTransforms;

	urdf::Model robot_model;
};
} // namespace dog
} // namespace dls

#endif /* end of include guard: DOGLIB_DOG_HPP_PE0SMNXQ */
