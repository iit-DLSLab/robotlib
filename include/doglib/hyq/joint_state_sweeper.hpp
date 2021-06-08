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
#ifndef DOGLIB_HYQ_JOINT_STATE_SWEEPER_HPP
#define DOGLIB_HYQ_JOINT_STATE_SWEEPER_HPP

#include "doglib/base/declarations.hpp"

namespace dls {
namespace dog {
namespace HyQ {

struct JointLim {
	double min;
	double max;
};

static struct JointLim jointLimits[] = {
	{-1.5700,  0.5236}, // LF_HAA
	{-0.8727,  1.2217}, // LF_HFE
	{-2.4435, -0.3491}, // LF_KFE

	{-1.5700,  0.5236}, // RF_HAA
	{-0.8727,  1.2217}, // RF_HFE
	{-2.4435, -0.3491}, // RF_KFE

	{-1.5700,  0.5236}, // LH_HAA
	{-1.2217,  0.8727}, // LH_HFE
	{ 0.3491,  2.4435}, // LH_KFE

	{-1.5700,  0.5236}, // RH_HAA
	{-1.2217,  0.8727}, // RH_HFE
	{ 0.3491,  2.4435}, // RH_KFE
};

class JointStateSweeper
{
public:
	JointStateSweeper(double pstep);
	const JointState& next();
	const JointState& current() const;
private:
	double step;
	dog::JointState current_state;
};


} // HyQ
} // dog
} // dls

#endif
