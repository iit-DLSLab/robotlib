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
#ifndef _ROBOTLIB_HYQ_ROBOT_LIMITS_HPP_
#define _ROBOTLIB_HYQ_ROBOT_LIMITS_HPP_

#include "robotlib/base/robot_limits.hpp"
#include "robotlib/hyq/cylinder_lever_arm.hpp"

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
#endif // _ROBOTLIB_HYQ_ROBOT_LIMITS_HPP_