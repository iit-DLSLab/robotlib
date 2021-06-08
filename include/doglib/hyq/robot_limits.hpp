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
#ifndef DOGLIB_ROBOT_LIMITS_HPP
#define DOGLIB_ROBOT_LIMITS_HPP

#include "doglib/base/robot_limits.hpp"
#include "doglib/hyq/cylinder_lever_arm.hpp"

namespace dls {
namespace dog {
namespace HyQ {

class Limits : public dog::LimitsBase
{
public:
	Limits();
	virtual ~Limits();
	void setMaxEffort(const dog::JointState & max_effort);
	dog::JointState getTorqueLimits(const dog::JointState & q);
private:

   dog::JointState  max_actuator_effort_;
};

} // HyQ
} // dog
} // dls
#endif
