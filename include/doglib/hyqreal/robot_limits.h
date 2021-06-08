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
#ifndef DOGLIB_HYQREAL_ROBOT_LIMTS_HPP
#define DOGLIB_HYQREAL_ROBOT_LIMTS_HPP

#include "doglib/base/robot_limits.hpp"
#include "commons/four_bar_linkage.h"
#include <memory>

namespace dls {
namespace dog {
namespace HyQReal {

class Limits : public dog::LimitsBase
{
public:
    Limits();
    virtual ~Limits();

    void setMaxEffort(const dog::JointState & max_effort);

    dog::JointState getTorqueLimits(const dog::JointState & q);
private:
   dog::JointState  max_actuator_effort_;
   dls::commons::FBLConsts fblConsts;
   std::shared_ptr<dls::commons::FBLStatus> fblSts;
};


} // HyQReal
} // dog
} // dls

#endif
