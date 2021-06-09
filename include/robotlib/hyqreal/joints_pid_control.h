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
 * PIDControl.h
 *
 *  Created on: May 22, 2013
 *      Author: mfrigerio
 */

#ifndef _ROBOTLIB_HYQREAL_JOINTS_PID_CONTROL_HPP_
#define _ROBOTLIB_HYQREAL_JOINTS_PID_CONTROL_HPP_

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/joint_data_map.hpp"
#include "robotlib/base/leg_data_map.hpp"
#include "commons/control.h"

namespace dls {
namespace dog {
namespace HyQReal {

typedef dls::commons::ctrl::pid_errors pid_errors; // to cope with a possible change in iit:: ...
typedef dls::commons::ctrl::pid_gains  pid_gains;
typedef dls::commons::ctrl::pid_terms  pid_terms;

typedef dls::dog::JointDataMap<pid_errors> JointsPIDerrors;
typedef dls::dog::JointDataMap<pid_gains>  JointsPIDgains;
typedef dls::dog::JointDataMap<pid_terms>  JointsPIDterms;
// typedef dls::dog::JointIdentifiers JointIdentifiers;

class JointsPIDControl
{
public:
    typedef dog::JointDataMap<double>  References;
    typedef dog::JointDataMap<double>  States;
public:
    JointsPIDControl(double ts, double tf);

    double getTs() const;
    double getTf() const;

    void setGains(const JointsPIDgains& gains);
    const JointsPIDgains& getGains() const;

    const JointsPIDerrors& getErrors() const;
    const JointsPIDterms&  getTerms()  const;

    void pid(
            const References& ref,
            const States&     actual);

private:
    void singleJointPID(dls::dog::JointIdentifiers j, double ref, double actual);
private:
    dls::commons::ctrl::PIDControl controller;
    JointsPIDerrors errors;
    JointsPIDgains  gains;
    JointsPIDterms  terms;
};


inline JointsPIDControl::JointsPIDControl(double ts, double tf)
    : controller(ts, tf), errors(0.0), gains(0.0), terms(0.0)
{
}


inline double JointsPIDControl::getTs() const {
    return controller.getTs();
}
inline double JointsPIDControl::getTf() const {
    return controller.getTf();
}

inline void JointsPIDControl::setGains(const JointsPIDgains& g) {
    gains = g;
}

inline const JointsPIDgains& JointsPIDControl::getGains() const {
    return gains;
}

inline const JointsPIDerrors& JointsPIDControl::getErrors() const {
    return errors;
}

inline const JointsPIDterms& JointsPIDControl::getTerms() const {
    return terms;
}

inline void JointsPIDControl::singleJointPID(
        dls::dog::JointIdentifiers j,
        double ref,
        double actual)
{
    errors[j].current = ref - actual;
    controller.pid(gains[j], errors[j], terms[j]);
    errors[j].updatePrevious();
}

} // HyQReal
} // dog
} // dls


#endif
