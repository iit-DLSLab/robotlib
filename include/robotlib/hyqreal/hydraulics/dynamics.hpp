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
 * hydraulics.h
 *
 *  Created on: Apr 29, 2013
 *      Author: phd
 */

#ifndef _ROBOTLIB_HYQREAL_HYDRAULICS_DYNAMICS_HPP_
#define _ROBOTLIB_HYQREAL_HYDRAULICS_DYNAMICS_HPP_

#include "commons/hydraulics.h"
#include "commons/four_bar_linkage.h"

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/joint_data_map.hpp"
#include "robotlib/base/leg_data_map.hpp"
#include "robotlib/base/joint_id_tricks.hpp"
#include "robotlib/hyqreal/hydraulics/params.hpp"
#include "robotlib/hyqreal/internals.hpp"

namespace dls {
namespace dog {
namespace HyQReal {

typedef dls::hydr::ValvePort Valve;


template<class T>
using JointDataMap = dog::JointDataMap<T>;

typedef dog::JointIdentifiers JointIdentifiers;
typedef dog::JointState JointState;

typedef JointDataMap<dls::hydr::Pressures>          Pressures;
typedef JointDataMap<dls::hydr::HydDynTerms>        DynamicsTerms;
typedef JointDataMap<dls::hydr::LinearizationTerms> LinearizationTerms;
typedef JointDataMap<dls::hydr::LinearizationGains> LinearizationGains;
typedef JointDataMap<Valve::Port>				   ValvePorts;
typedef JointDataMap<dls::hydr::PortToVoltageSign>  ValvePortToVoltageMap;
typedef JointDataMap<dls::hydr::ForceSignToPort>	ForceToValvePortMap;

class HydraulicsDynamics
{

public:
	HydraulicsDynamics();

	HydraulicsDynamics(
		const HydraulicsParameters&,
		const ValvePortToVoltageMap&,
		const ForceToValvePortMap&);

	void setHydraulicsParameters(const HydraulicsParameters&);
	void configure(const ValvePortToVoltageMap&, const ForceToValvePortMap&);

	void setActiveValvePort(JointIdentifiers, Valve::Port);

	void estimateCylinderChamberPressures(
			JointIdentifiers j,
			double force,
			dls::hydr::Pressures& press) const;

	void estimateMotorChamberPressures(
			JointIdentifiers j,
			double tau_force,
			dls::hydr::Pressures& press) const;

	void computeDynamicsTerms(
			const JointState& q,
			const JointState& qd,
			const Pressures& pressures);

	void computeLinearizationTerms(
			const LinearizationGains& linGains);

	void applyLinearizationTerms(
			const JointDataMap<double>& force_ctrl_commands,
			JointDataMap<double>& output_commands) const;

	double jointStateToCylinderState(JointIdentifiers, const dls::commons::FBLStatus&) const;
	double jointStateToMotorState(JointIdentifiers, const double&) const;

	const DynamicsTerms& getDynamicsTerms() const;
	const LinearizationTerms& getLinearizationTerms() const;

	HyQReal::JointDataMap<dls::hydr::ActuatorState> debugActuator;


private:
	void motor_dynamics_terms(
			const JointIdentifiers& haa,
			const double& q, const double& qd,
			const dls::hydr::Pressures& pressures);

	void cylinder_dynamics_terms(
			const JointIdentifiers& sagittalj,
			const double& q, const double& qd,
			const dls::hydr::Pressures& pressures);
private:
	static const JointIdentifiers haa_joints_id[4];
	HydraulicsParameters   hydrParams;
	DynamicsTerms          dynamicsTerms;
	LinearizationTerms     linearizationTerms;
	ValvePorts             activeValvePorts;
	ValvePortToVoltageMap  valvePortToVoltage;
	ForceToValvePortMap    forceToValvePort;
};

} // HyQReal
} // dog
} // dls
#endif
