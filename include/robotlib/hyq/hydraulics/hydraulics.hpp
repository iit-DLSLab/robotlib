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
#ifndef _ROBOTLIB_HYQ_HYDRAULICS_HPP_
#define _ROBOTLIB_HYQ_HYDRAULICS_HPP_


#include "robotlib/base/declarations.hpp"
#include "robotlib/hyq/cylinder_lever_arm.hpp"
#include "robotlib/hyq/internals.hpp"
#include "robotlib/base/joint_id_tricks.hpp"
#include "robotlib/base/joint_data_map.hpp"

#include "commons/hydraulics.h"


namespace dls {
namespace dog {
namespace HyQ {
namespace hydr {

typedef dls::hydr::ValvePort Valve;

struct HydraulicsParameters
{
	dls::hydr::PipelineProperties pipeline;
	dls::hydr::MotorProperties	HAA_motor;
	dls::hydr::CylinderProperties cylinder;
	dog::JointDataMap<dls::hydr::ValveProperties>	valveParams;
	HydraulicsParameters();
};


typedef dog::JointDataMap<dls::hydr::Pressures>          Pressures;
typedef dog::JointDataMap<dls::hydr::HydDynTerms>        DynamicsTerms;
typedef dog::JointDataMap<dls::hydr::LinearizationTerms> LinearizationTerms;
typedef dog::JointDataMap<dls::hydr::LinearizationGains> LinearizationGains;
typedef dog::JointDataMap<Valve::Port>                   ValvePorts;
typedef dog::JointDataMap<dls::hydr::PortToVoltageSign>  ValvePortToVoltageMap;
typedef dog::JointDataMap<dls::hydr::ForceSignToPort>    ForceToValvePortMap;

class HydraulicsDynamics
{

public:
	HydraulicsDynamics();

	HydraulicsDynamics
	(
		const HydraulicsParameters&,
		const ValvePortToVoltageMap&,
		const ForceToValvePortMap&
	);

	void setHydraulicsParameters(const HydraulicsParameters&);
	void configure(const ValvePortToVoltageMap&, const ForceToValvePortMap&);

	void setActiveValvePort(dog::JointIdentifiers, Valve::Port);
	//void setGains(const JointDataMap<LinearizationGains>& gains);

	void estimateChamberPressures
	(
		dog::JointIdentifiers j,
		double tau_force,
		dls::hydr::Pressures& press
	) const;

	void computeDynamicsTerms
	(
		const dog::JointState& q,
		const dog::JointState& qd,
		const Pressures& pressures
	);

	void computeLinearizationTerms
	(
		const LinearizationGains& linGains
	);

	void applyLinearizationTerms
	(
		const dog::JointDataMap<double>& force_ctrl_commands,
		dog::JointDataMap<double>& output_commands
	) const;

	void applyLinearizationTermsAndUpdateValveGains
	(
		const dog::JointDataMap<double>& force_ctrl_commands,
		dog::JointDataMap<double>& output_commands
	);

	const DynamicsTerms& getDynamicsTerms() const;
	const LinearizationTerms& getLinearizationTerms() const;

	double jointStateToMotorActuatorState(const dog::JointIdentifiers& j, const double & q) const;

	double jointStateToCylinderActuatorState(const dog::JointIdentifiers& j,const double & q, const double & cylinder_length) const;

	dog::JointDataMap<dls::hydr::ActuatorState> debugActuator;


	void estimateHAAChamberPressures
	(
		dog::JointIdentifiers j,
		double tau,
		dls::hydr::Pressures& press
	) const;

	void estimateCylinderChamberPressures
	(
		dog::JointIdentifiers j,
		double force,
		dls::hydr::Pressures& press
	) const;

	void setValveGain
	(
		dog::JointIdentifiers j,
		double valveGain
	);

	void setValveDoubleGain
	(
		dog::JointIdentifiers j,
		double gain_low,
		double gain_high,
		double transition_point,
		double dt
	);

	double getValveGain
	(
		dog::JointIdentifiers j
	) const;

private:

	void motor_dynamics_terms
	(
		const dog::JointIdentifiers& haa,
		const double& q, const double& qd,
		const dls::hydr::Pressures& pressures
	);

	void cylinder_dynamics_terms
	(
		const dog::JointIdentifiers& sagittalj,
		const double& q, const double& qd,
		const dls::hydr::Pressures& pressures
	);

private:
	static const dog::JointIdentifiers haa_joints_id[4];
	HydraulicsParameters hydrParams;
	DynamicsTerms		dynamicsTerms;
	LinearizationTerms   linearizationTerms;
	ValvePorts		   activeValvePorts;
	ValvePortToVoltageMap valvePortToVoltage;
	ForceToValvePortMap  forceToValvePort;
};


} // hydr
} // HyQ
} // dog
} // dls

#endif
