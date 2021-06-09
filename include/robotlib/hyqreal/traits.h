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
#ifndef _ROBOTLIB_HYQREAL_TRAITS_HPP_
#define _ROBOTLIB_HYQREAL_TRAITS_HPP_

#include "robotlib/hyqreal/declarations.h"
#include "robotlib/hyqreal/transforms.h"
#include "robotlib/hyqreal/inverse_dynamics.h"
#include "robotlib/hyqreal/forward_dynamics.h"
#include "robotlib/hyqreal/jsim.h"
#include "robotlib/hyqreal/inertia_properties.h"

namespace dls {
namespace dog {
namespace HyQReal {
struct Traits {
    typedef typename HyQReal::JointState JointState;

    typedef typename HyQReal::JointIdentifiers JointID;
    typedef typename HyQReal::LinkIdentifiers  LinkID;

    typedef typename HyQReal::HomogeneousTransforms HomogeneousTransforms;
    typedef typename HyQReal::MotionTransforms MotionTransforms;
    typedef typename HyQReal::ForceTransforms ForceTransforms;

    typedef typename HyQReal::dyn::InertiaProperties InertiaProperties;
    typedef typename HyQReal::dyn::ForwardDynamics FwdDynEngine;
    typedef typename HyQReal::dyn::InverseDynamics InvDynEngine;
    typedef typename HyQReal::dyn::JSIM JSIM;

    static const int joints_count = HyQReal::jointsCount;
    static const int links_count  = HyQReal::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return HyQReal::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return HyQReal::orderedLinkIDs;
}

} // HyQReal
} // dog
} // dls

#endif
