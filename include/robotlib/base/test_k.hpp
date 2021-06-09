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
#ifndef _ROBOTLIB_BASE_TEST_K_HPP_
#define _ROBOTLIB_BASE_TEST_K_HPP_

#include <iostream>
#include "robotlib/rbd/rbd.h"
#include "robotlib/rbd/utils.h"

#include "robotlib/base/forward_kinematics.hpp"
#include "robotlib/base/inverse_kinematics.hpp"
#include "robotlib/base/leg_data_map.hpp"

namespace dls {
namespace dog {


class TestK
{
public:
	struct Result
	{
		JointState q;
		JointState q_ik;
		bool equal;
		LegDataMap<Vector3d> feet;
	};
public:

	TestK(ForwardKinematicsBase& fwdkin, InverseKinematicsBase& invkin);
	~TestK() = default;

	void testFK_IK(const JointState& q, Result& res);

private:
	ForwardKinematicsBase& fk;
	InverseKinematicsBase& ik;
};


} // namespace dog
} // namespace dls

#endif // _ROBOTLIB_BASE_TEST_K_HPP_
