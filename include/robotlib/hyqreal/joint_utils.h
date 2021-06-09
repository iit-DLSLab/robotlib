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
/*
 * joint_ids_tricks.h
 *
 *  Created on: Nov 17, 2014
 *      Author: mfrigerio
 */

#ifndef _ROBOTLIB_HYQREAL_JOINT_UTILS_HPP_
#define _ROBOTLIB_HYQREAL_JOINT_UTILS_HPP_

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/leg_data_map.hpp"

namespace dls {
namespace dog {
namespace HyQReal {

inline bool hasRotaryActuator(dog::JointIdentifiers j)
{
    return ! (j==dog::LF_KFE || j==dog::RF_KFE || j==dog::LH_KFE || j==dog::RH_KFE);
}

} // HyQReal
} // dog
} // dls

#endif
