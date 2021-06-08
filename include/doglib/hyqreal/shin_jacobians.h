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
#ifndef DOGLIB_HYQREAL_SHIN_JACOBIANS_HPP
#define DOGLIB_HYQREAL_SHIN_JACOBIANS_HPP

#include "doglib/base/shin_jacobians.hpp"
#include "doglib/base/leg_data_map.hpp"
#include "doglib/base/declarations.hpp"
#include "doglib/hyqreal/jacobians.h"
#include "doglib/hyqreal/feet_jacobians.h"
#include "doglib/rbd/utils.h"

namespace dls {
namespace dog {
namespace HyQReal {
class ShinJacobians : public dog::ShinJacobiansBase
{
typedef dog::JointState JointState;


public:
    ShinJacobians(const dog::KinDynParamsBase & param_getter) :
        param_getter_(param_getter), jacobians_(param_getter_),
        feet_jacobians_(jacobians_)
    {
    }
    ~ShinJacobians() {}

    dog::FootJac getShinJacobian(const JointState& q,
                                  const double& contact_point,
                                  const dog::LegID& leg) {


        // TODO
        // the default param getter has no shins for now,
        // returning the foot jacobians always
        //
        /*switch(leg){
        case dog::LF:
            param_getter_setValue_LF_shin(contact_point);
            return jacobians_.fr_trunk_J_LF_shin(q);
        case dog::RF:
            param_getter_setValue_RF_shin(contact_point);
            return jacobians_.fr_trunk_J_RF_shin(q);
        case dog::LH:
            param_getter_setValue_LH_shin(contact_point);
            return jacobians_.fr_trunk_J_LH_shin(q);
        case dog::RH:
            param_getter_setValue_RH_shin(contact_point);
            return jacobians_.fr_trunk_J_RH_shin(q);
        }
        return dog::FootJac::Identity();*/
        return feet_jacobians_.getFootJacobian(q,leg);
    }

private:
    const dog::KinDynParamsBase& param_getter_;
    HyQReal::Jacobians jacobians_;
    HyQReal::FeetJacobians feet_jacobians_;
};


} // HyQReal
} // dog
} // dls


#endif
