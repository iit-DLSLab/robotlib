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
#ifndef _ROBOTLIB_HYQ_HYQ_HPP_
#define _ROBOTLIB_HYQ_HYQ_HPP_

// =============================================================================
// Includes
// =============================================================================
#include "robotlib/base/dog.hpp"

namespace dls
{
namespace dog
{
namespace HyQ {
// =============================================================================
// Class Definition
// =============================================================================
class hyq : public dls::dog::Dog
{
public:
	hyq
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
	);

	virtual std::string getName() override;
	virtual int ndof() override { return 12; }
};
} // namespace HyQ
} // namespace dog
} // namespace dls

#endif //_ROBOTLIB_HYQ_HYQ_HPP_