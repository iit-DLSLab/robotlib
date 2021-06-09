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
// =============================================================================
// Includes
// =============================================================================
#include <memory>
#include "robotlib/base/dog.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog;

// =============================================================================
// Constructors
// =============================================================================
Dog::Dog
(
	std::shared_ptr<RobotLengthsBase>       pRobotLengths_,
	std::shared_ptr<FeetContactForcesBase>  pFeetContactForces_,
	std::shared_ptr<FeetJacobiansBase>      pFeetJacobians_,
	std::shared_ptr<ForwardKinematicsBase>  pForwardKinematics_,
	std::shared_ptr<InertiaPropertiesBase>  pInertiaProperties_,
	std::shared_ptr<InverseDynamicsBase>    pInverseDynamics_,
	std::shared_ptr<InverseKinematicsBase>  pInverseKinematics_,
	std::shared_ptr<JSIMBase>               pJSIM_,
	std::shared_ptr<KinDynParamsBase>       pKinDynParams_,
	std::shared_ptr<LimitsBase>             pLimits_,
	std::shared_ptr<ShinJacobiansBase>      pShinJacobians_,

	// NEW
	std::shared_ptr<dls::dog::HomogeneousTransformsBase>  pHomogeneousTransforms_,
	std::shared_ptr<dls::dog::MotionTransformsBase>      pMotionTransforms_,
	std::shared_ptr<dls::dog::ForceTransformsBase>       pForceTransforms_,
		const urdf::Model                                &model_
) :
	pRobotLengths       (pRobotLengths_       ),
	pFeetContactForces  (pFeetContactForces_  ),
	pFeetJacobians      (pFeetJacobians_      ),
	pForwardKinematics  (pForwardKinematics_  ),
	pInertiaProperties  (pInertiaProperties_  ),
	pInverseDynamics    (pInverseDynamics_    ),
	pInverseKinematics  (pInverseKinematics_  ),
	pJSIM               (pJSIM_               ),
	pKinDynParams       (pKinDynParams_       ),
	pLimits             (pLimits_             ),
	pShinJacobians      (pShinJacobians_      ),

	// NEW
	pHomogeneousTransforms  (pHomogeneousTransforms_  ),
	pMotionTransforms       (pMotionTransforms_       ),
	pForceTransforms        (pForceTransforms_        ),
	robot_model(model_)
{ }

// =============================================================================
// Implementation
// =============================================================================
std::shared_ptr<RobotLengthsBase>       Dog::getRobotLengths()       {  return  this->pRobotLengths;  }
std::shared_ptr<FeetContactForcesBase>  Dog::getFeetContactforces()  {  return  this->pFeetContactForces;  }
std::shared_ptr<FeetJacobiansBase>      Dog::getFeetJacobians()      {  return  this->pFeetJacobians;  }
std::shared_ptr<ForwardKinematicsBase>  Dog::getForwardKinematics()  {  return  this->pForwardKinematics;  }
std::shared_ptr<InertiaPropertiesBase>  Dog::getInertiaProperties()  {  return  this->pInertiaProperties;  }
std::shared_ptr<InverseDynamicsBase>    Dog::getInverseDynamics()    {  return  this->pInverseDynamics;  }
std::shared_ptr<InverseKinematicsBase>  Dog::getInverseKinematics()  {  return  this->pInverseKinematics;  }
std::shared_ptr<JSIMBase>               Dog::getJSIM()               {  return  this->pJSIM;  }
std::shared_ptr<KinDynParamsBase>       Dog::getKinDynParams()       {  return  this->pKinDynParams;  }
std::shared_ptr<LimitsBase>             Dog::getLimits()             {  return  this->pLimits;  }
std::shared_ptr<ShinJacobiansBase>      Dog::getShinJacobians()      {  return  this->pShinJacobians;  }

// NEW
std::shared_ptr<HomogeneousTransformsBase>  Dog::getHomogeneousTransforms()  {  return  this->pHomogeneousTransforms;  }
std::shared_ptr<MotionTransformsBase>       Dog::getMotionTransforms()       {  return  this->pMotionTransforms;       }
std::shared_ptr<ForceTransformsBase>        Dog::getForceTransforms()        {  return  this->pForceTransforms;        }
urdf::Model                                 Dog::getModel()                  {  return  this->robot_model;             }
