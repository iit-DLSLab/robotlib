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
// =============================================================================
// Includes
// =============================================================================
#include "doglib/hyq/hydraulics/hydraulics.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ::hydr;
// =============================================================================
// Constructors
// =============================================================================
// =============================================================================
// Implementation
// =============================================================================

HydraulicsDynamics::HydraulicsDynamics() :
	hydrParams(),
	dynamicsTerms(0.0),
	linearizationTerms(0.0)
{ }

void HydraulicsDynamics::setHydraulicsParameters
(
	const HydraulicsParameters& params)
{
	hydrParams = params;
}

void HydraulicsDynamics::configure
(
	const ValvePortToVoltageMap& port2Volt,
	const ForceToValvePortMap& force2Port
)
{
	valvePortToVoltage = port2Volt;
	forceToValvePort   = force2Port;
}

void HydraulicsDynamics::setActiveValvePort
(
	dog::JointIdentifiers j,
	Valve::Port p
)
{
	activeValvePorts[j] = p;
}

void HydraulicsDynamics::estimateHAAChamberPressures
(
	dog::JointIdentifiers j,
	double tau,
	dls::hydr::Pressures& pressures
) const
{
	dls::hydr::motorChamberPressures(
			hydrParams.HAA_motor,
			forceToValvePort[j].getPositiveForcePort(),
			forceToValvePort[j].getNegativeForcePort(),
			tau, pressures);
}

void HydraulicsDynamics::estimateCylinderChamberPressures
(
	dog::JointIdentifiers j,
	double force,
	dls::hydr::Pressures& pressures
) const
{
	dls::hydr::cylinderChamberPressures
	(
		hydrParams.cylinder,
		forceToValvePort[j].getPositiveForcePort(),
		forceToValvePort[j].getNegativeForcePort(),
		force, pressures
	);
}

void HydraulicsDynamics::estimateChamberPressures
(
		dog::JointIdentifiers j,
		double tau_force,
		dls::hydr::Pressures& pressures
) const
{
	if( ! dog::isHAA(j) )
	{
		// Compute the pressures in the chambers of the cylinders
		estimateCylinderChamberPressures(j, tau_force, pressures);
	}
	else
	{
		// Compute the pressures in the chambers of the HAA motors
		estimateHAAChamberPressures(j, tau_force, pressures);
	}
}

const DynamicsTerms& HydraulicsDynamics::getDynamicsTerms() const
{
	return dynamicsTerms;
}

const LinearizationTerms& HydraulicsDynamics::getLinearizationTerms() const
{
	return linearizationTerms;
}


/**
 * Returns the position state of the hydraulic motor on the HAA joint.
 * Such a state refers to the position of the internal vane of the motor;
 * by convention, such a state should be zero when the vane is touching
 * one of the internal end-stops (no oil in one chamber).
 * \param q_haa the angle of the HAA joint, in radians, according to the
 *		robot conventions
 */
double HydraulicsDynamics::jointStateToMotorActuatorState
(
	const dog::JointIdentifiers& j,
	const double & q
) const
{
	if( ! isHAA(j) )
	{
		//TODO print warning / log error / exception!!
		return 0;
	}
	return q - HyQ::internal::misc_cfg.HAAJointToMotorOffset[ dog::toLegID(j) ];
}

double HydraulicsDynamics::jointStateToCylinderActuatorState
(
	const dog::JointIdentifiers& j,
	const double & q,
	const double & cylinder_length
) const
{

	double xp;

	if (forceToValvePort[j].getPositiveForcePort() == dls::hydr::ValvePort::A)
		xp = cylinder_length - HyQ::retractedCylinderLength;
	else
		xp = hydrParams.cylinder.stroke - (cylinder_length - HyQ::retractedCylinderLength);

	return xp;
}

void HydraulicsDynamics::setValveGain
(
	dog::JointIdentifiers j,
	double valveGain
)
{
	hydrParams.valveParams[j].gain = valveGain;
	hydrParams.valveParams[j].gain_f = hydrParams.valveParams[j].gain;
}

void HydraulicsDynamics::setValveDoubleGain
(
	dog::JointIdentifiers j,
	double gain_low,
	double gain_high,
	double transition_point,
	double dt
)
{
	hydrParams.valveParams[j].gain_low = gain_low;
	hydrParams.valveParams[j].gain_high = gain_high;
	hydrParams.valveParams[j].transition_point = transition_point;
	hydrParams.valveParams[j].dt = dt;
	hydrParams.valveParams[j].gain_f = hydrParams.valveParams[j].gain;
}

double HydraulicsDynamics::getValveGain
(
	dog::JointIdentifiers j) const
{
	return hydrParams.valveParams[j].gain;
}

using namespace dls::dog;

// using namespace dls::commons;



HyQ::hydr::HydraulicsParameters::HydraulicsParameters() {
	pipeline.oilBulkMod = 1.8181E9;
	pipeline.volume = 8.17E-6; // tube 0.65 long, internal diameter 0.004 (everything in meters); // TODO this comes from hyl2
	// TODO check the following
	HAA_motor.volumetricDisp = 6.87E-6;
	HAA_motor.totVolume	  = 12.0E-6; //range of motion (100°) times vol displacement
	HAA_motor.unusedVolume[dls::hydr::ValvePort::A] = 0.0; // TODO need to update iit commons
	HAA_motor.unusedVolume[dls::hydr::ValvePort::B] = 0.0; // TODO need to update iit commons
	//cylinder
	cylinder.pistonArea[dls::hydr::ValvePort::A] = 2.0106 * 1E-04; //area chamber A [m²]
	cylinder.pistonArea[dls::hydr::ValvePort::B] =  0.6094*cylinder.pistonArea[dls::hydr::ValvePort::A];
	cylinder.stroke = 0.08; //m
	dls::hydr::valveProperties defaultValveProperties;
	defaultValveProperties.gain = 6.12E-8;
	defaultValveProperties.gain_f =  defaultValveProperties.gain;
	defaultValveProperties.gain_high = 6.12E-8;
	defaultValveProperties.gain_low = 6.12E-8;
	defaultValveProperties.transition_point = 0.0;
	//TODO Kv = (1.8*2/60000)/(1.6)/sqrt(68.9476*10^5/2)*10 Kv =	2.019699178641170e-07
	valveParams = defaultValveProperties;
}


HyQ::hydr::HydraulicsDynamics::HydraulicsDynamics(
	const HydraulicsParameters& params,
	const ValvePortToVoltageMap& port2voltage,
   	const ForceToValvePortMap&  force2port)

	:
	hydrParams(params),
	dynamicsTerms(0.0),
	linearizationTerms(0.0),
	valvePortToVoltage(port2voltage),
	forceToValvePort(force2port)
{

}


void HyQ::hydr::HydraulicsDynamics::computeDynamicsTerms(
		const dog::JointState& q,
		const dog::JointState& qd,
		const Pressures& pressures)
{
	dog::JointIdentifiers j;
	for (int i = 0; i<dog::jointsCount; i++)
	{
		j = dog::orderedJointIDs[i];
		if( ! dog::isHAA(j) ) {
			cylinder_dynamics_terms(j, q(j), qd(j), pressures[j]);
		}else {
			motor_dynamics_terms(j, q(j), qd(j), pressures[j]);
		}
	}
}


void HyQ::hydr::HydraulicsDynamics::computeLinearizationTerms(const LinearizationGains& gains)
{
	dog::JointIdentifiers j;
	for (int i = 0; i<dog::jointsCount; i++)
	{
		j = dog::orderedJointIDs[i];
		dls::hydr::computeLinearizationTerms(dynamicsTerms[j], gains[j], linearizationTerms[j]);

	}
}

void HyQ::hydr::HydraulicsDynamics::applyLinearizationTerms(
		const dog::JointDataMap<double>& input_commands,
		dog::JointDataMap<double>& output_commands) const
{
	dog::JointIdentifiers j;
	for (int i = 0; i<dog::jointsCount; i++)
	{
		j = dog::orderedJointIDs[i];
		output_commands[j] = dls::hydr::applyLinearizationTerms(linearizationTerms[j], input_commands[j]);
	}
}

void HyQ::hydr::HydraulicsDynamics::applyLinearizationTermsAndUpdateValveGains(
		const dog::JointDataMap<double>& input_commands,
		dog::JointDataMap<double>& output_commands)
{
	dog::JointIdentifiers j;
	for (int i = 0; i<dog::jointsCount; i++)
	{
		j = dog::orderedJointIDs[i];
		output_commands[j] = dls::hydr::applyLinearizationTerms(hydrParams.valveParams[j], linearizationTerms[j], input_commands[j]);
	}
}


void HyQ::hydr::HydraulicsDynamics::motor_dynamics_terms(
		const dog::JointIdentifiers& j,
		const double& q, const double& qd,
		const dls::hydr::Pressures& pressures)
{
	dls::hydr::ActuatorState haa_motorState;

	haa_motorState.q  = jointStateToMotorActuatorState(j,q);
	haa_motorState.qd = qd;
	haa_motorState.activePort = activeValvePorts[j];

	debugActuator[j].qd =  haa_motorState.qd;
	debugActuator[j].q =  haa_motorState.q;

	dls::hydr::feedback_linearization_motor
	(
			hydrParams.pipeline,
			hydrParams.HAA_motor,
			hydrParams.valveParams[j],
			haa_motorState,
			pressures,
			forceToValvePort[j].getPositiveForcePort(),
			valvePortToVoltage[j],
			dynamicsTerms[j].f,
			dynamicsTerms[j].g
		);
}


void HyQ::hydr::HydraulicsDynamics::cylinder_dynamics_terms(
		const dog::JointIdentifiers& j,
		const double& q, const double& qd,
		const dls::hydr::Pressures& pressures)
{
	dls::hydr::ActuatorState cylinderState;

	//actuator state are piston position and velocitys
	cylinderState.q  = jointStateToCylinderActuatorState(j, q, getCylinderLength(j,q)); //piston position
	cylinderState.qd =  qd * getCylinderLeverArm( j, getCylinderLength(j, q) ); //piston velocity is the joint velocity multiplied by lever arm



	cylinderState.activePort = activeValvePorts[j];

	debugActuator[j].qd =  cylinderState.qd;
	debugActuator[j].q =  cylinderState.q;

//	dls::hydr::ValveProperties modifiedValveProp  =   hydrParams.valveParams;
//	if ((j == HyQ::LH_HFE) || (j == HyQ::LH_KFE)){
//		modifiedValveProp.gain = 6.12E-8;
//	}

	dls::hydr::feedback_linearization_cylinder(
			hydrParams.pipeline,
			hydrParams.cylinder,
			hydrParams.valveParams[j],
			cylinderState,
			pressures,
			forceToValvePort[j].getPositiveForcePort(),
			valvePortToVoltage[j],
			dynamicsTerms[j].f, dynamicsTerms[j].g);
}




