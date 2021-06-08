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
#ifndef DOGLIB_BASE_TEST_FK_HPP
#define DOGLIB_BASE_TEST_FK_HPP

#include <iostream>
#include "doglib/rbd/rbd.h"

#include "doglib/base/forward_kinematics.hpp"
#include "doglib/base/leg_data_map.hpp"

namespace dls {
namespace dog {

class TestFK
{
public:

    TestFK(ForwardKinematicsBase& fwdkin);
    ~TestFK() = default;

    void printPositions(const JointState& q);

private:
    ForwardKinematicsBase& fk;
};


} // namespace dog
} // namespace dls
#endif
