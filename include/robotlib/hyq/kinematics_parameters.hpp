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
#ifndef _ROBOTLIB_HYQ_KINEMATICS_PARAMETERS_HPP_
#define _ROBOTLIB_HYQ_KINEMATICS_PARAMETERS_HPP_

namespace dls {
namespace dog {
namespace HyQ {

struct Params_lengths
{
	double foot_x;
	double LF_shin;
	double RF_shin;
	double LH_shin;
	double RH_shin;
};

} // HyQ
} // dog
} // dls
#endif // _ROBOTLIB_HYQ_KINEMATICS_PARAMETERS_HPP_
