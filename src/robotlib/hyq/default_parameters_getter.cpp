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
#include "robotlib/hyq/default_parameters_getter.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ;

// =============================================================================
// Constructors
// =============================================================================
DefaultParamsGetter::DefaultParamsGetter()
{
	resetDefaults();
}

// =============================================================================
// Implementation
// =============================================================================
void DefaultParamsGetter::resetDefaults()
{
	value.foot_x = 0.341;	  // [m]
	inertias.trunk_mass = 60.96;
	inertias.trunk_com_x = 0.056;
	inertias.trunk_com_y = 0.0215;
	inertias.trunk_com_z = 0.00358;
	value.LF_shin = 0.341;	  // [m]
	value.RF_shin = 0.341;	  // [m]
	value.LH_shin = 0.341;	  // [m]
	value.RH_shin = 0.341;	  // [m]

	inertias.trunk_Ix  =   1.6882;
	inertias.trunk_Iy  =   8.6653;
	inertias.trunk_Iz  =   9.2436;
	inertias.trunk_Ixy =   0.0288;
	inertias.trunk_Ixz =  -0.2777;
	inertias.trunk_Iyz =  -0.0038;
}

double DefaultParamsGetter::getValue_trunk_mass() const {
	return inertias.trunk_mass;
}
double DefaultParamsGetter::getValue_trunk_com_x() const {
	return inertias.trunk_com_x;
}
double DefaultParamsGetter::getValue_trunk_com_y() const {
	return inertias.trunk_com_y;
}
double DefaultParamsGetter::getValue_trunk_com_z() const {
	return inertias.trunk_com_z;
}
double DefaultParamsGetter::getValue_trunk_Ix() const {
	return inertias.trunk_Ix;
}
double DefaultParamsGetter::getValue_trunk_Iy() const {
	return inertias.trunk_Iy;
}
double DefaultParamsGetter::getValue_trunk_Iz() const {
	return inertias.trunk_Iz;
}
double DefaultParamsGetter::getValue_trunk_Ixy() const {
	return inertias.trunk_Ixy;
}
double DefaultParamsGetter::getValue_trunk_Ixz() const {
	return inertias.trunk_Ixz;
}
double DefaultParamsGetter::getValue_trunk_Iyz() const {
	return inertias.trunk_Iyz;
}
//setters
// the getters are not virtual (the setters are!) so
// if you inherit this class, to avoid the use of the default
// implementation we need to set the functions as virtual!)
void DefaultParamsGetter::setValue_trunk_mass(double val)  {
	inertias.trunk_mass =val;
}
void DefaultParamsGetter::setValue_trunk_com_x(double val)  {
	inertias.trunk_com_x =val;
}
void DefaultParamsGetter::setValue_trunk_com_y(double val)  {
	inertias.trunk_com_y =val;
}
void DefaultParamsGetter::setValue_trunk_com_z(double val)  {
	inertias.trunk_com_z =val;
}
void DefaultParamsGetter::setValue_trunk_Ix(double val)  {
	inertias.trunk_Ix =val;
}
void DefaultParamsGetter::setValue_trunk_Iy(double val)  {
	inertias.trunk_Iy =val;
}
void DefaultParamsGetter::setValue_trunk_Iz(double val)  {
	inertias.trunk_Iz =val;
}
void DefaultParamsGetter::setValue_trunk_Ixy(double val)  {
	inertias.trunk_Ixy =val;
}
void DefaultParamsGetter::setValue_trunk_Ixz(double val)  {
	inertias.trunk_Ixz =val;
}
void DefaultParamsGetter::setValue_trunk_Iyz(double val)  {
	inertias.trunk_Iyz =val;
}
double DefaultParamsGetter::getValue_foot_x() const
{
	return value.foot_x;
}

void DefaultParamsGetter::setValue_foot_x(double val)
{
	value.foot_x = val;
}

double DefaultParamsGetter::getValue_LF_shin() const {
	return value.LF_shin;
}
double DefaultParamsGetter::getValue_RF_shin() const {
	return value.RF_shin;
}
double DefaultParamsGetter::getValue_LH_shin() const {
	return value.LH_shin;
}
double DefaultParamsGetter::getValue_RH_shin() const {
	return value.RH_shin;
}

void DefaultParamsGetter::setValue_LF_shin(double val)
{
	 value.LF_shin = val;
}
void DefaultParamsGetter::setValue_RF_shin(double val)
{
	 value.RF_shin = val;
}
void DefaultParamsGetter::setValue_LH_shin(double val)
{
	 value.LH_shin = val;
}
void DefaultParamsGetter::setValue_RH_shin(double val)
{
	 value.RH_shin =  val;
}

// TODO
// these are not used in the actual implementation
// the generated code for HyQ is not yet parametric
// for these parameters
double DefaultParamsGetter::getValue_haa_x() const {
	return 0;
}
double DefaultParamsGetter::getValue_haa_y() const {
	return 0;
}
double DefaultParamsGetter::getValue_haa_z() const {
	return 0;
}
double DefaultParamsGetter::getValue_haa_hfe() const {
	return 0;
}
double DefaultParamsGetter::getValue_upper_leg() const {
	return 0;
}
double DefaultParamsGetter::getValue_lower_leg() const {
	return 0;
}

void DefaultParamsGetter::setValue_haa_x(double val) {}
void DefaultParamsGetter::setValue_haa_y(double val) {}
void DefaultParamsGetter::setValue_haa_z(double val) {}
void DefaultParamsGetter::setValue_haa_hfe(double val) {}
void DefaultParamsGetter::setValue_upper_leg(double val) {}
void DefaultParamsGetter::setValue_lower_leg(double val) {}

/**
 * @return the whole set of the current inertias of the inertia parameters
 */
const dyn::RuntimeInertiaParams& DefaultParamsGetter::getInertiaParams() const {
	return inertias;
}
/**
 * Sets new inertias for all the inertia parameters
 */
void DefaultParamsGetter::setInertiaParams(const dyn::RuntimeInertiaParams& newp) {
	inertias = newp;
}
