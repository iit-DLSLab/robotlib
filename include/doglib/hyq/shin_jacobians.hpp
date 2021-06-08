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
#ifndef DOGLIB_SHIN_JACOBIANS_HPP
#define DOGLIB_SHIN_JACOBIANS_HPP

#include "doglib/base/shin_jacobians.hpp"
#include "doglib/base/kin_dyn_params.hpp"
#include "doglib/base/leg_data_map.hpp"
#include "doglib/base/declarations.hpp"
#include "doglib/hyq/jacobians.hpp"

#include "doglib/rbd/utils.h"


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


#endif
