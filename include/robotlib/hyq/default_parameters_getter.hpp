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
#ifndef _ROBOTLIB_HYQ_DEFAULT_PARAMETERS_GETTER_HPP_
#define _ROBOTLIB_HYQ_DEFAULT_PARAMETERS_GETTER_HPP_

#include "robotlib/base/kin_dyn_params.hpp"
#include "robotlib/hyq/kinematics_parameters.hpp"
#include "robotlib/hyq/dynamics_parameters.hpp"

namespace dls {
namespace dog {
namespace HyQ {


class DefaultParamsGetter : public KinDynParamsBase
{
public:
	DefaultParamsGetter();

	virtual void resetDefaults();

	//getters
	double getValue_trunk_mass() const;
	double getValue_trunk_com_x() const;
	double getValue_trunk_com_y() const;
	double getValue_trunk_com_z() const;
	double getValue_trunk_Ix() const;
	double getValue_trunk_Iy() const;
	double getValue_trunk_Iz() const;
	double getValue_trunk_Ixy() const;
	double getValue_trunk_Ixz() const;
	double getValue_trunk_Iyz() const;
	//setters
	// the getters are not virtual (the setters are!) so
	// if you inherit this class, to avoid the use of the default
	// implementation we need to set the functions as virtual!)
	virtual void setValue_trunk_mass(double val);
	virtual void setValue_trunk_com_x(double val);
	virtual void setValue_trunk_com_y(double val);
	virtual void setValue_trunk_com_z(double val);
	virtual void setValue_trunk_Ix(double val);
	virtual void setValue_trunk_Iy(double val);
	virtual void setValue_trunk_Iz(double val);
	virtual void setValue_trunk_Ixy(double val);
	virtual void setValue_trunk_Ixz(double val);
	virtual void setValue_trunk_Iyz(double val);
	double getValue_foot_x() const;

	virtual void setValue_foot_x(double val);

	double getValue_LF_shin() const;
	double getValue_RF_shin() const;
	double getValue_LH_shin() const;
	double getValue_RH_shin() const;

	virtual void setValue_LF_shin(double val);
	virtual void setValue_RF_shin(double val);
	virtual void setValue_LH_shin(double val);
	virtual void setValue_RH_shin(double val);

	// TODO
	// these are not used in the actual implementation
	// the generated code for HyQ is not yet parametric
	// for these parameters
	virtual double getValue_haa_x() const;
	virtual double getValue_haa_y() const;
	virtual double getValue_haa_z() const;
	virtual double getValue_haa_hfe() const;
	virtual double getValue_upper_leg() const;
	virtual double getValue_lower_leg() const;

	virtual void setValue_haa_x(double val);
	virtual void setValue_haa_y(double val);
	virtual void setValue_haa_z(double val);
	virtual void setValue_haa_hfe(double val);
	virtual void setValue_upper_leg(double val);
	virtual void setValue_lower_leg(double val);

	/**
	 * @return the whole set of the current inertias of the inertia parameters
	 */
	const dyn::RuntimeInertiaParams& getInertiaParams() const;
	/**
	 * Sets new inertias for all the inertia parameters
	 */
	virtual void setInertiaParams(const dyn::RuntimeInertiaParams& newp);


private:
	Params_lengths value;
	dyn::RuntimeInertiaParams inertias;
};


} // HyQ
} // dog
} // dls
#endif // _ROBOTLIB_HYQ_DEFAULT_PARAMETERS_GETTER_HPP_
