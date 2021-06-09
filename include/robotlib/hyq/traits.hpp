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
#ifndef _ROBOTLIB_HYQ_TRAITS_HPP_
#define _ROBOTLIB_HYQ_TRAITS_HPP_

#include "robotlib/hyq/transforms.hpp"
#include "robotlib/hyq/inverse_dynamics.hpp"
#include "robotlib/hyq/forward_dynamics.hpp"
#include "robotlib/hyq/jsim.hpp"

namespace dls {
namespace dog {
namespace HyQ {

struct Traits
{
    typedef typename HyQ::HomogeneousTransforms HomogeneousTransforms;
    typedef typename HyQ::MotionTransforms MotionTransforms;
    typedef typename HyQ::ForceTransforms ForceTransforms;

    typedef typename HyQ::dyn::ForwardDynamics FwdDynEngine;
    typedef typename HyQ::dyn::InverseDynamics InvDynEngine;
    typedef typename HyQ::dyn::JSIM JSIM;

    static const bool floating_base = true;
};

} // HyQ
} // dog
} // dls

#endif // _ROBOTLIB_HYQ_TRAITS_HPP_
