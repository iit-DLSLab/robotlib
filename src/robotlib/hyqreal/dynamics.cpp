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
#include "robotlib/hyqreal/hydraulics/dynamics.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQReal;

// =============================================================================
// Implementation
// =============================================================================
HydraulicsDynamics::HydraulicsDynamics() :
        hydrParams(), dynamicsTerms(0.0), linearizationTerms(0.0)
{ }

void HydraulicsDynamics::setHydraulicsParameters(
        const HydraulicsParameters& params)
{
    hydrParams = params;
}

void HydraulicsDynamics::configure(
        const ValvePortToVoltageMap& port2Volt, const ForceToValvePortMap& force2Port)
{
    valvePortToVoltage = port2Volt;
    forceToValvePort   = force2Port;
}


void HydraulicsDynamics::setActiveValvePort(
        JointIdentifiers j, Valve::Port p)
{
    activeValvePorts[j] = p;
}


void HydraulicsDynamics::estimateCylinderChamberPressures(
        JointIdentifiers j,
        double force,
        dls::hydr::Pressures& pressures) const
{
    dls::hydr::cylinderChamberPressures(
            hydrParams.KFE_cylinder,
            forceToValvePort[j].getPositiveForcePort(),
            forceToValvePort[j].getNegativeForcePort(),
            force, pressures);
}


double HydraulicsDynamics::jointStateToCylinderState(
        JointIdentifiers j, const dls::commons::FBLStatus& fbl) const
{
    //double extension = fbl.cylinder - fbl::cyl_retracted;
    return
    (forceToValvePort[j].getPositiveForcePort() == dls::hydr::ValvePort::A) ?
            fbl.extension  :  hydrParams.KFE_cylinder.stroke - fbl.extension;
}

double HydraulicsDynamics::jointStateToMotorState(JointIdentifiers j, const double& q) const
{
    return
    isHAA(j) ? q - internal::misc_cfg.HAAJointToMotorOffset[ toLegID(j) ]
             : q - internal::misc_cfg.HFEJointToMotorOffset[ toLegID(j) ];
}


const DynamicsTerms& HydraulicsDynamics::getDynamicsTerms() const {
    return dynamicsTerms;
}
const LinearizationTerms& HydraulicsDynamics::getLinearizationTerms() const {
    return linearizationTerms;
}
