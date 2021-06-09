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
#ifndef _ROBOTLIB_HYQ_SHIN_JACOBIANS_HPP_
#define _ROBOTLIB_HYQ_SHIN_JACOBIANS_HPP_

#include "robotlib/base/shin_jacobians.hpp"
#include "robotlib/base/kin_dyn_params.hpp"
#include "robotlib/base/leg_data_map.hpp"
#include "robotlib/base/declarations.hpp"
#include "robotlib/hyq/jacobians.hpp"

#include "robotlib/rbd/utils.h"


namespace dls {
namespace dog {
namespace HyQ {

class ShinJacobians : public dls::dog::ShinJacobiansBase
{
typedef dog::JointState JointState;


public:
	ShinJacobians(dls::dog::KinDynParamsBase& param_getter);
	~ShinJacobians() = default;

	dog::FootJac getShinJacobian
	(
		const JointState& q,
		const double& contact_point,
		const dog::LegID& leg
	);

private:
	dog::KinDynParamsBase& param_getter_;
	HyQ::Jacobians jacobians_;
};


} // HyQ
} // dog
} // dls


#endif // _ROBOTLIB_HYQ_SHIN_JACOBIANS_HPP_
