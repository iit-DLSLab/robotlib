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
#include "doglib/hyq/inertia_properties.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ::dyn;
using namespace std;
using namespace dls::rbd;

// =============================================================================
// Constructors
// =============================================================================

// =============================================================================
// Implementation
// =============================================================================
 InertiaProperties::~InertiaProperties() {}

 const InertiaMatrix& InertiaProperties::getTensor_trunk()
{
	updateParameters();
	return this->tensor_trunk;
}

const InertiaMatrix& InertiaProperties::getTensor_LF_hipassembly() const
{
	return this->tensor_LF_hipassembly;
}

const InertiaMatrix& InertiaProperties::getTensor_LF_upperleg() const
{
	return this->tensor_LF_upperleg;
}

const InertiaMatrix& InertiaProperties::getTensor_LF_lowerleg() const
{
	return this->tensor_LF_lowerleg;
}

const InertiaMatrix& InertiaProperties::getTensor_RF_hipassembly() const
{
	return this->tensor_RF_hipassembly;
}

const InertiaMatrix& InertiaProperties::getTensor_RF_upperleg() const
{
	return this->tensor_RF_upperleg;
}

const InertiaMatrix& InertiaProperties::getTensor_RF_lowerleg() const
{
	return this->tensor_RF_lowerleg;
}

const InertiaMatrix& InertiaProperties::getTensor_LH_hipassembly() const
{
	return this->tensor_LH_hipassembly;
}

const InertiaMatrix& InertiaProperties::getTensor_LH_upperleg() const
{
	return this->tensor_LH_upperleg;
}

const InertiaMatrix& InertiaProperties::getTensor_LH_lowerleg() const
{
	return this->tensor_LH_lowerleg;
}

const InertiaMatrix& InertiaProperties::getTensor_RH_hipassembly() const
{
	return this->tensor_RH_hipassembly;
}

const InertiaMatrix& InertiaProperties::getTensor_RH_upperleg() const
{
	return this->tensor_RH_upperleg;
}

const InertiaMatrix& InertiaProperties::getTensor_RH_lowerleg() const
{
	return this->tensor_RH_lowerleg;
}

double InertiaProperties::getMass_trunk() const
{
	return this->tensor_trunk.getMass();
}

double InertiaProperties::getMass_LF_hipassembly() const
{
	return this->tensor_LF_hipassembly.getMass();
}

double InertiaProperties::getMass_LF_upperleg() const
{
	return this->tensor_LF_upperleg.getMass();
}

double InertiaProperties::getMass_LF_lowerleg() const
{
	return this->tensor_LF_lowerleg.getMass();
}

double InertiaProperties::getMass_RF_hipassembly() const
{
	return this->tensor_RF_hipassembly.getMass();
}

double InertiaProperties::getMass_RF_upperleg() const
{
	return this->tensor_RF_upperleg.getMass();
}

double InertiaProperties::getMass_RF_lowerleg() const
{
	return this->tensor_RF_lowerleg.getMass();
}

double InertiaProperties::getMass_LH_hipassembly() const
{
	return this->tensor_LH_hipassembly.getMass();
}

double InertiaProperties::getMass_LH_upperleg() const
{
	return this->tensor_LH_upperleg.getMass();
}

double InertiaProperties::getMass_LH_lowerleg() const
{
	return this->tensor_LH_lowerleg.getMass();
}

double InertiaProperties::getMass_RH_hipassembly() const
{
	return this->tensor_RH_hipassembly.getMass();
}

double InertiaProperties::getMass_RH_upperleg() const
{
	return this->tensor_RH_upperleg.getMass();
}

double InertiaProperties::getMass_RH_lowerleg() const
{
	return this->tensor_RH_lowerleg.getMass();
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_trunk() const
{
	return this->com_trunk;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_LF_hipassembly() const
{
	return this->com_LF_hipassembly;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_LF_upperleg() const
{
	return this->com_LF_upperleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_LF_lowerleg() const
{
	return this->com_LF_lowerleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_RF_hipassembly() const
{
	return this->com_RF_hipassembly;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_RF_upperleg() const
{
	return this->com_RF_upperleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_RF_lowerleg() const
{
	return this->com_RF_lowerleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_LH_hipassembly() const
{
	return this->com_LH_hipassembly;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_LH_upperleg() const
{
	return this->com_LH_upperleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_LH_lowerleg() const
{
	return this->com_LH_lowerleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_RH_hipassembly() const
{
	return this->com_RH_hipassembly;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_RH_upperleg() const
{
	return this->com_RH_upperleg;
}

const dls::rbd::Vector3d& InertiaProperties::getCOM_RH_lowerleg() const
{
	return this->com_RH_lowerleg;
}

 double InertiaProperties::getTotalMass() const
{
	return paramsGetter.getValue_trunk_mass() +
	 2.93 +
	 2.638 +
	 0.881 +
	 2.93 +
	 2.638 +
	 0.881 +
	 2.93 +
	 2.638 +
	 0.881 +
	 2.93 +
	 2.638 +
	 0.881;
}


dls::dog::HyQ::dyn::InertiaProperties::InertiaProperties(const dls::dog::KinDynParamsBase& pGetter)
    : paramsGetter(pGetter), homogeneus_transforms(pGetter)
{
    com_trunk = dls::rbd::Vector3d(paramsGetter.getValue_trunk_com_x(),paramsGetter.getValue_trunk_com_y(),paramsGetter.getValue_trunk_com_z());
    tensor_trunk.fill(
        paramsGetter.getValue_trunk_mass(),
        com_trunk,
        Utils::buildInertiaTensor(
                paramsGetter.getValue_trunk_Ix(),
                paramsGetter.getValue_trunk_Iy(),
                paramsGetter.getValue_trunk_Iz(),
                paramsGetter.getValue_trunk_Ixy(),
                paramsGetter.getValue_trunk_Ixz(),
                paramsGetter.getValue_trunk_Iyz()) );

    com_LF_hipassembly = dls::rbd::Vector3d(0.04263,0.0,0.16931);
    tensor_LF_hipassembly.fill(
        2.93,
        com_LF_hipassembly,
        Utils::buildInertiaTensor(
                0.134705,
                0.144171,
                0.011033,
                3.6E-5,
                0.022734,
                5.1E-5) );

    com_LF_upperleg = dls::rbd::Vector3d(0.15074,-0.02625,-0.0);
    tensor_LF_upperleg.fill(
        2.638,
        com_LF_upperleg,
        Utils::buildInertiaTensor(
                0.005495,
                0.087136,
                0.089871,
                -0.007418,
                -1.02E-4,
                -2.1E-5) );

    com_LF_lowerleg = dls::rbd::Vector3d(0.1254,5.0E-4,-1.0E-4);
    tensor_LF_lowerleg.fill(
        0.881,
        com_LF_lowerleg,
        Utils::buildInertiaTensor(
                4.68E-4,
                0.026409,
                0.026181,
                0.0,
                0.0,
                0.0) );

    com_RF_hipassembly = dls::rbd::Vector3d(0.04263,-0.0,-0.16931);
    tensor_RF_hipassembly.fill(
        2.93,
        com_RF_hipassembly,
        Utils::buildInertiaTensor(
                0.134705,
                0.144171,
                0.011033,
                -3.6E-5,
                -0.022734,
                5.1E-5) );

    com_RF_upperleg = dls::rbd::Vector3d(0.15074,-0.02625,-0.0);
    tensor_RF_upperleg.fill(
        2.638,
        com_RF_upperleg,
        Utils::buildInertiaTensor(
                0.005495,
                0.087136,
                0.089871,
                -0.007418,
                -1.02E-4,
                -2.1E-5) );

    com_RF_lowerleg = dls::rbd::Vector3d(0.1254,5.0E-4,-1.0E-4);
    tensor_RF_lowerleg.fill(
        0.881,
        com_RF_lowerleg,
        Utils::buildInertiaTensor(
                4.68E-4,
                0.026409,
                0.026181,
                0.0,
                0.0,
                0.0) );

    com_LH_hipassembly = dls::rbd::Vector3d(0.04263,-0.0,-0.16931);
    tensor_LH_hipassembly.fill(
        2.93,
        com_LH_hipassembly,
        Utils::buildInertiaTensor(
                0.134705,
                0.144171,
                0.011033,
                -3.6E-5,
                -0.022734,
                5.1E-5) );

    com_LH_upperleg = dls::rbd::Vector3d(0.15074,0.02625,0.0);
    tensor_LH_upperleg.fill(
        2.638,
        com_LH_upperleg,
        Utils::buildInertiaTensor(
                0.005495,
                0.087136,
                0.089871,
                0.007418,
                1.02E-4,
                -2.1E-5) );

    com_LH_lowerleg = dls::rbd::Vector3d(0.1254,-5.0E-4,1.0E-4);
    tensor_LH_lowerleg.fill(
        0.881,
        com_LH_lowerleg,
        Utils::buildInertiaTensor(
                4.68E-4,
                0.026409,
                0.026181,
                0.0,
                0.0,
                0.0) );

    com_RH_hipassembly = dls::rbd::Vector3d(0.04263,0.0,0.16931);
    tensor_RH_hipassembly.fill(
        2.93,
        com_RH_hipassembly,
        Utils::buildInertiaTensor(
                0.134705,
                0.144171,
                0.011033,
                3.6E-5,
                0.022734,
                5.1E-5) );

    com_RH_upperleg = dls::rbd::Vector3d(0.15074,0.02625,0.0);
    tensor_RH_upperleg.fill(
        2.638,
        com_RH_upperleg,
        Utils::buildInertiaTensor(
                0.005495,
                0.087136,
                0.089871,
                0.007418,
                1.02E-4,
                -2.1E-5) );

    com_RH_lowerleg = dls::rbd::Vector3d(0.1254,-5.0E-4,1.0E-4);
    tensor_RH_lowerleg.fill(
        0.881,
        com_RH_lowerleg,
        Utils::buildInertiaTensor(
                4.68E-4,
                0.026409,
                0.026181,
                0.0,
                0.0,
                0.0) );

}

void dls::dog::HyQ::dyn::InertiaProperties::updateParameters() {
    com_trunk = dls::rbd::Vector3d(paramsGetter.getValue_trunk_com_x(),
                                   paramsGetter.getValue_trunk_com_y(),
                                   paramsGetter.getValue_trunk_com_z());
        tensor_trunk.fill(
            paramsGetter.getValue_trunk_mass(),
            com_trunk,
            Utils::buildInertiaTensor(
                    paramsGetter.getValue_trunk_Ix(),
                    paramsGetter.getValue_trunk_Iy(),
                    paramsGetter.getValue_trunk_Iz(),
                    paramsGetter.getValue_trunk_Ixy(),
                    paramsGetter.getValue_trunk_Ixz(),
                    paramsGetter.getValue_trunk_Iyz()));
}




 const dls::rbd::Vector3d& dls::dog::HyQ::dyn::InertiaProperties::getTrunkCOM() {
     updateParameters();
     return this->getCOM_trunk();
 }
 double dls::dog::HyQ::dyn::InertiaProperties::getTrunkMass() {
     updateParameters();
     return this->getMass_trunk();
 }

 double dls::dog::HyQ::dyn::InertiaProperties::getHipAssemblyMass(const dog::LegID& leg) const{
     switch(leg){
     case dog::LF:
         return getMass_LF_hipassembly();
     case dog::RF:
         return getMass_RF_hipassembly();
     case dog::LH:
         return getMass_LH_hipassembly();
     case dog::RH:
         return getMass_RH_hipassembly();
     }
 }

 double dls::dog::HyQ::dyn::InertiaProperties::getUpperLegMass(const dog::LegID& leg) const{
     switch(leg){
     case dog::LF:
         return getMass_LF_upperleg();
     case dog::RF:
         return getMass_RF_upperleg();
     case dog::LH:
         return getMass_LH_upperleg();
     case dog::RH:
         return getMass_RH_upperleg();
     }
 }
 double dls::dog::HyQ::dyn::InertiaProperties::getLowerLegMass(const dog::LegID& leg) const{
     switch(leg){
     case dog::LF:
         return getMass_LF_lowerleg();
     case dog::RF:
         return getMass_RF_lowerleg();
     case dog::LH:
         return getMass_LH_lowerleg();
     case dog::RH:
         return getMass_RH_lowerleg();
     }
 }

 dls::rbd::Vector3d dls::dog::HyQ::dyn::InertiaProperties::getHipAssemblyCOM(const dog::LegID& leg) const{
     switch(leg){
     case dog::LF:
         return getCOM_LF_hipassembly();
     case dog::RF:
         return getCOM_RF_hipassembly();
     case dog::LH:
         return getCOM_LH_hipassembly();
     case dog::RH:
         return getCOM_RH_hipassembly();
     }
 }
 dls::rbd::Vector3d dls::dog::HyQ::dyn::InertiaProperties::getUpperLegCOM(const dog::LegID& leg) const{
     switch(leg){
     case dog::LF:
         return getCOM_LF_upperleg();
     case dog::RF:
         return getCOM_RF_upperleg();
     case dog::LH:
         return getCOM_LH_upperleg();
     case dog::RH:
         return getCOM_RH_upperleg();
     }
 }
 dls::rbd::Vector3d dls::dog::HyQ::dyn::InertiaProperties::getLowerLegCOM(const dog::LegID& leg) const{
     switch(leg){
     case dog::LF:
         return getCOM_LF_lowerleg();
     case dog::RF:
         return getCOM_RF_lowerleg();
     case dog::LH:
         return getCOM_LH_lowerleg();
     case dog::RH:
         return getCOM_RH_lowerleg();
     }
 }

 dls::rbd::Vector3d dls::dog::HyQ::dyn::InertiaProperties::getWholeBodyCOM(const dog::JointState &q){
     homogeneus_transforms.fr_trunk_X_fr_LF_hipassembly(q);
     homogeneus_transforms.fr_trunk_X_fr_RF_hipassembly(q);
     homogeneus_transforms.fr_trunk_X_fr_LH_hipassembly(q);
     homogeneus_transforms.fr_trunk_X_fr_RH_hipassembly(q);
     homogeneus_transforms.fr_LF_hipassembly_X_fr_LF_upperleg(q);
     homogeneus_transforms.fr_LF_upperleg_X_fr_LF_lowerleg(q);
     homogeneus_transforms.fr_RF_hipassembly_X_fr_RF_upperleg(q);
     homogeneus_transforms.fr_RF_upperleg_X_fr_RF_lowerleg(q);
     homogeneus_transforms.fr_LH_hipassembly_X_fr_LH_upperleg(q);
     homogeneus_transforms.fr_LH_upperleg_X_fr_LH_lowerleg(q);
     homogeneus_transforms.fr_RH_hipassembly_X_fr_RH_upperleg(q);
     homogeneus_transforms.fr_RH_upperleg_X_fr_RH_lowerleg(q);

     dls::rbd::Vector3d tmpSum(dls::rbd::Vector3d::Zero());

     tmpSum += this->getTrunkCOM() * this->getTrunkMass();

     static HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
     HomogeneousTransforms::MatrixType base_X_LF_HAA_chain;
     HomogeneousTransforms::MatrixType base_X_RF_HAA_chain;
     HomogeneousTransforms::MatrixType base_X_LH_HAA_chain;
     HomogeneousTransforms::MatrixType base_X_RH_HAA_chain;


     base_X_LF_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_LF_hipassembly;
     tmpSum += this->getMass_LF_hipassembly() *
             ( dls::rbd::Utils::transform(base_X_LF_HAA_chain, this->getCOM_LF_hipassembly()));

     base_X_LF_HAA_chain = base_X_LF_HAA_chain * homogeneus_transforms.fr_LF_hipassembly_X_fr_LF_upperleg;
     tmpSum += this->getMass_LF_upperleg() *
             ( dls::rbd::Utils::transform(base_X_LF_HAA_chain, this->getCOM_LF_upperleg()));

     base_X_LF_HAA_chain = base_X_LF_HAA_chain * homogeneus_transforms.fr_LF_upperleg_X_fr_LF_lowerleg;
     tmpSum += this->getMass_LF_lowerleg() *
             ( dls::rbd::Utils::transform(base_X_LF_HAA_chain, this->getCOM_LF_lowerleg()));

     base_X_RF_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_RF_hipassembly;
     tmpSum += this->getMass_RF_hipassembly() *
             ( dls::rbd::Utils::transform(base_X_RF_HAA_chain, this->getCOM_RF_hipassembly()));

     base_X_RF_HAA_chain = base_X_RF_HAA_chain * homogeneus_transforms.fr_RF_hipassembly_X_fr_RF_upperleg;
     tmpSum += this->getMass_RF_upperleg() *
             ( dls::rbd::Utils::transform(base_X_RF_HAA_chain, this->getCOM_RF_upperleg()));

     base_X_RF_HAA_chain = base_X_RF_HAA_chain * homogeneus_transforms.fr_RF_upperleg_X_fr_RF_lowerleg;
     tmpSum += this->getMass_RF_lowerleg() *
             ( dls::rbd::Utils::transform(base_X_RF_HAA_chain, this->getCOM_RF_lowerleg()));

     base_X_LH_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_LH_hipassembly;
     tmpSum += this->getMass_LH_hipassembly() *
             ( dls::rbd::Utils::transform(base_X_LH_HAA_chain, this->getCOM_LH_hipassembly()));

     base_X_LH_HAA_chain = base_X_LH_HAA_chain * homogeneus_transforms.fr_LH_hipassembly_X_fr_LH_upperleg;
     tmpSum += this->getMass_LH_upperleg() *
             ( dls::rbd::Utils::transform(base_X_LH_HAA_chain, this->getCOM_LH_upperleg()));

     base_X_LH_HAA_chain = base_X_LH_HAA_chain * homogeneus_transforms.fr_LH_upperleg_X_fr_LH_lowerleg;
     tmpSum += this->getMass_LH_lowerleg() *
             ( dls::rbd::Utils::transform(base_X_LH_HAA_chain, this->getCOM_LH_lowerleg()));

     base_X_RH_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_RH_hipassembly;
     tmpSum += this->getMass_RH_hipassembly() *
             ( dls::rbd::Utils::transform(base_X_RH_HAA_chain, this->getCOM_RH_hipassembly()));

     base_X_RH_HAA_chain = base_X_RH_HAA_chain * homogeneus_transforms.fr_RH_hipassembly_X_fr_RH_upperleg;
     tmpSum += this->getMass_RH_upperleg() *
             ( dls::rbd::Utils::transform(base_X_RH_HAA_chain, this->getCOM_RH_upperleg()));

     base_X_RH_HAA_chain = base_X_RH_HAA_chain * homogeneus_transforms.fr_RH_upperleg_X_fr_RH_lowerleg;
     tmpSum += this->getMass_RH_lowerleg() *
             ( dls::rbd::Utils::transform(base_X_RH_HAA_chain, this->getCOM_RH_lowerleg()));


     return tmpSum / this->getTotalMass();
 }
