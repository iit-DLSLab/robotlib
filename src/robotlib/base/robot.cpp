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
#include "robotlib/base/robot.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::robot;

// =============================================================================
// Constructors
// =============================================================================
Robot::Robot
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
	std::shared_ptr<dls::robot::HomogeneousTransformsBase>  pHomogeneousTransforms_,
	std::shared_ptr<dls::robot::MotionTransformsBase>      pMotionTransforms_,
	std::shared_ptr<dls::robot::ForceTransformsBase>       pForceTransforms_,
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
std::shared_ptr<RobotLengthsBase>       Robot::getRobotLengths()       {  return  this->pRobotLengths;  }
std::shared_ptr<FeetContactForcesBase>  Robot::getFeetContactforces()  {  return  this->pFeetContactForces;  }
std::shared_ptr<FeetJacobiansBase>      Robot::getFeetJacobians()      {  return  this->pFeetJacobians;  }
std::shared_ptr<ForwardKinematicsBase>  Robot::getForwardKinematics()  {  return  this->pForwardKinematics;  }
std::shared_ptr<InertiaPropertiesBase>  Robot::getInertiaProperties()  {  return  this->pInertiaProperties;  }
std::shared_ptr<InverseDynamicsBase>    Robot::getInverseDynamics()    {  return  this->pInverseDynamics;  }
std::shared_ptr<InverseKinematicsBase>  Robot::getInverseKinematics()  {  return  this->pInverseKinematics;  }
std::shared_ptr<JSIMBase>               Robot::getJSIM()               {  return  this->pJSIM;  }
std::shared_ptr<KinDynParamsBase>       Robot::getKinDynParams()       {  return  this->pKinDynParams;  }
std::shared_ptr<LimitsBase>             Robot::getLimits()             {  return  this->pLimits;  }
std::shared_ptr<ShinJacobiansBase>      Robot::getShinJacobians()      {  return  this->pShinJacobians;  }

// NEW
std::shared_ptr<HomogeneousTransformsBase>  Robot::getHomogeneousTransforms()  {  return  this->pHomogeneousTransforms;  }
std::shared_ptr<MotionTransformsBase>       Robot::getMotionTransforms()       {  return  this->pMotionTransforms;       }
std::shared_ptr<ForceTransformsBase>        Robot::getForceTransforms()        {  return  this->pForceTransforms;        }
urdf::Model                                 Robot::getModel()                  {  return  this->robot_model;             }
