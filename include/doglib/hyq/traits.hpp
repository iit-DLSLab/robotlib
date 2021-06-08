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
#ifndef DOGLIB_HYQ_TRAITS_HPP
#define DOGLIB_HYQ_TRAITS_HPP

#include "doglib/hyq/transforms.hpp"
#include "doglib/hyq/inverse_dynamics.hpp"
#include "doglib/hyq/forward_dynamics.hpp"
#include "doglib/hyq/jsim.hpp"

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

#endif
