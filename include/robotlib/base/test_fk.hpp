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
#ifndef _ROBOTLIB_BASE_TEST_FK_HPP_
#define _ROBOTLIB_BASE_TEST_FK_HPP_

#include <iostream>
#include "robotlib/rbd/rbd.h"

#include "robotlib/base/forward_kinematics.hpp"
#include "robotlib/base/leg_data_map.hpp"

namespace dls {
namespace robot {

class TestFK
{
public:

    TestFK(ForwardKinematicsBase& fwdkin);
    ~TestFK() = default;

    void printPositions(const JointState& q);

private:
    ForwardKinematicsBase& fk;
};


} // namespace robot
} // namespace dls
#endif // _ROBOTLIB_BASE_TEST_FK_HPP_