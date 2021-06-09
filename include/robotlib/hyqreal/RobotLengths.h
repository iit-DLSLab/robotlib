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
 * RobotLenghts.h
 *
 *  Created on: May 10, 2017
 *      Author: Victor Barasuol
 *
 * THIS SHOULDNT EXIST
 */

#ifndef _ROBOTLIB_HYQREAL_ROBOT_LENGTHS_HPP_
#define _ROBOTLIB_HYQREAL_ROBOT_LENGTHS_HPP_

#include "robotlib/base/body_lengths.hpp"

namespace dls {
namespace dog {
namespace HyQReal {

/**
 * @brief The RobotLengths class contains the relevant robot lenghts required
 * by kinematics objects (e.g., inverse kinematics), such as the distance
 * between the robot center and the HAA axis.
 * @deprecated This class is deprecated, and included for backcompatibility
 * only. It will be removed soon in the next software release.
 * Please use iit::HyQReal::DefaultParamsGetter instead.
 * @sa iit::HyQReal::DefaultParamsGetter
 */
class RobotLengths : public dls::dog::RobotLengthsBase {
public:
    RobotLengths() {};
    ~RobotLengths() {};
    virtual double getHAA_x()  override{
        return 0.4435;
    }
    virtual double getHAA_y()  override{
        return 0.139;
    }
    virtual double getHAA_z()  override{
        return 0.0;
    }
    virtual double getDist_HAA_HFE()  override{
        return 0.117;
    }
    virtual double getDist_HFE_KFE()  override{
        return 0.360;
    }
    double get_lowleg_length()  {
        return 0.380;
    }
    virtual double getFoot_x()  override{
        return 0.3730;
    }
    virtual double getFoot_y()  override {
        return 0.0725;
    }
    virtual double getFoot_z()  override {
        return 0;
    }
};

} // HyQReal
} // dog
} // dls

#endif /* HYQREAL_IK_ROBOTLENGTHS_IK_H_ */
