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
#ifndef DOGLIB_HYQ_CONTROL_JOINT_PID_CONTROL_HPP
#define DOGLIB_HYQ_CONTROL_JOINT_PID_CONTROL_HPP

#include "commons/control.h"
#include "doglib/base/declarations.hpp"
#include "doglib/base/joint_data_map.hpp"
#include "doglib/base/leg_data_map.hpp"

namespace dls {
namespace dog {
namespace HyQ {
namespace ctrl {

typedef dls::commons::ctrl::pid_errors pid_errors; // to cope with a possible change in dls:: ...
typedef dls::commons::ctrl::pid_gains  pid_gains;
typedef dls::commons::ctrl::pid_terms  pid_terms;

typedef dls::dog::JointDataMap<pid_errors> JointsPIDerrors;
typedef dls::dog::JointDataMap<pid_gains>  JointsPIDgains;
typedef dls::dog::JointDataMap<pid_terms>  JointsPIDterms;

typedef dls::dog::JointDataMap<double>  References;
typedef dls::dog::JointDataMap<double>  States;

typedef dls::dog::LegDataMap<pid_errors> HAAsPIDerrors;
typedef dls::dog::LegDataMap<pid_gains>  HAAsPIDgains;
typedef dls::dog::LegDataMap<pid_terms>  HAAsPIDterms;


class JointsPIDControl {
public:
	JointsPIDControl();
	JointsPIDControl(double ts, double tf);

	double getTs() const;
	double getTf() const;

	void resetIntegrators();

	void setGains(const JointsPIDgains& gains);
	const JointsPIDgains& getGains() const;

	const JointsPIDerrors& getErrors() const;
	const JointsPIDterms&  getTerms()  const;

	void pid(const References& ref, const States& actual);

private:
	void singleJointPID(dls::dog::JointIdentifiers j, double ref, double actual);
private:
	dls::commons::ctrl::PIDControl controller;
	JointsPIDerrors errors;
	JointsPIDgains  gains;
	JointsPIDterms  terms;
};
} // ctrl
} // HyQ
} // dog
} // dls


#endif
