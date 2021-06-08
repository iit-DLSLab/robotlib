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
#ifndef DOGLIB_HYQREAL_DEFAULT_PARAMETERS_GETTER_HPP
#define DOGLIB_HYQREAL_DEFAULT_PARAMETERS_GETTER_HPP

#include "doglib/base/kin_dyn_params.hpp"
#include "doglib/hyqreal/kinematics_parameters.h"
#include "doglib/hyqreal/dynamics_parameters.h"

namespace dls {
namespace dog {
namespace HyQReal {

/**
 * @brief The DefaultParamsGetter class
 * @date October 9th, 2013
 * @author Andreea Radulescu (andreea.radulescu@iit.it)
 * @author Marco Camurri (marco.camurri@iit.it)
 */
class DefaultParamsGetter : public dog::KinDynParamsBase
{
    public:
        DefaultParamsGetter();
        virtual ~DefaultParamsGetter() {}
    public:
        void resetDefaults() {
            value.upper_leg = 0.36; // TODO auto-generated -- fix me
            value.foot_x = 0.38; // TODO auto-generated -- fix me

            inertias.trunk_mass = 84.991171; // TODO auto-generated -- fix me
            inertias.trunk_com_x = 0.047250; // TODO auto-generated -- fix me
            inertias.trunk_com_y = 0.000069; // TODO auto-generated -- fix me
            inertias.trunk_com_z =-0.000034; // TODO auto-generated -- fix me
            inertias.trunk_Ix = 1.600773; // TODO auto-generated -- fix me
            inertias.trunk_Iy = 9.784177; // TODO auto-generated -- fix me
            inertias.trunk_Iz = 9.202111; // TODO auto-generated -- fix me
            inertias.trunk_Ixy =-0.000018; // TODO auto-generated -- fix me
            inertias.trunk_Ixz =-0.328286; // TODO auto-generated -- fix me
            inertias.trunk_Iyz =-0.007369; // TODO auto-generated -- fix me

            value.haa_y = 0.139*100/100; // Why the 100 / 100 ? by mcamurri
            value.haa_x = 0.4435*100/100;
            value.haa_hfe = 0.117;
            value.upper_leg = 0.36;

        }
        double getValue_foot_x() const {
            return value.foot_x;
        }

        //getters
        double getValue_trunk_mass() const {
            return inertias.trunk_mass;
        }
        double getValue_trunk_com_x() const {
            return inertias.trunk_com_x;
        }
        double getValue_trunk_com_y() const {
            return inertias.trunk_com_y;
        }
        double getValue_trunk_com_z() const {
            return inertias.trunk_com_z;
        }
        double getValue_trunk_Ix() const {
            return inertias.trunk_Ix;
        }
        double getValue_trunk_Iy() const {
            return inertias.trunk_Iy;
        }
        double getValue_trunk_Iz() const {
            return inertias.trunk_Iz;
        }
        double getValue_trunk_Ixy() const {
            return inertias.trunk_Ixy;
        }
        double getValue_trunk_Ixz() const {
            return inertias.trunk_Ixz;
        }
        double getValue_trunk_Iyz() const {
            return inertias.trunk_Iyz;
        }
        //setters
        virtual void setValue_trunk_mass(double val)  {
            inertias.trunk_mass =val;
        }
        virtual void setValue_trunk_com_x(double val)  {
            inertias.trunk_com_x =val;
        }
        virtual void setValue_trunk_com_y(double val)  {
            inertias.trunk_com_y =val;
        }
        virtual void setValue_trunk_com_z(double val)  {
            inertias.trunk_com_z =val;
        }
        virtual void setValue_trunk_Ix(double val)  {
            inertias.trunk_Ix =val;
        }
        virtual void setValue_trunk_Iy(double val)  {
            inertias.trunk_Iy =val;
        }
        virtual void setValue_trunk_Iz(double val)  {
            inertias.trunk_Iz =val;
        }
        virtual void setValue_trunk_Ixy(double val)  {
            inertias.trunk_Ixy =val;
        }
        virtual void setValue_trunk_Ixz(double val)  {
            inertias.trunk_Ixz =val;
        }
        virtual void setValue_trunk_Iyz(double val)  {
            inertias.trunk_Iyz =val;
        }

        inline void setValue_foot_x(double val);


        inline double getValue_haa_x() const;


        inline void setValue_haa_x(double val);


        inline double getValue_haa_y() const;

        inline double getValue_haa_z() const;


        inline void setValue_haa_y(double val);


        inline double getValue_haa_hfe() const;


        inline void setValue_haa_hfe(double val);

        inline double getValue_upper_leg() const;


        inline void setValue_upper_leg(double val);

        inline double getValue_lower_leg() const;


        inline void setValue_lower_leg(double val);

        inline const HyQReal::Params_lengths& getLengths() const {
            return value;
        }

        inline void setLengths(const HyQReal::Params_lengths& val)
        {
            value = val;
        }

        double getValue_LF_shin() const {
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
            return getValue_foot_x();
        }

        double getValue_RF_shin() const {
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
            return getValue_foot_x();
        }

        double getValue_LH_shin() const {
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
            return getValue_foot_x();
        }

        double getValue_RH_shin() const {
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
            return getValue_foot_x();
        }

        void setValue_LF_shin(double value){
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
        }
        void setValue_RF_shin(double value){
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
        }
        void setValue_LH_shin(double value){
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
        }
        void setValue_RH_shin(double value){
            // TODO
            // the shin frame is not implemented in HyQReal, returning
            // the foot one
        }

        /**
         * @return the whole set of the current values of the inertia parameters
         */
        const dyn::RuntimeInertiaParams& getInertiaParams() const {
            return inertias;
        }
        /**
         * Sets new values for all the inertia parameters
         */
        virtual void setInertiaParams(const dyn::RuntimeInertiaParams& newp) {
            inertias = newp;
        }

    private:
        Params_lengths value;
        dyn::RuntimeInertiaParams inertias;
};

inline DefaultParamsGetter::DefaultParamsGetter()
{
    resetDefaults();
}




inline void DefaultParamsGetter::setValue_foot_x(double val)
{
    value.foot_x = val;
}


inline double DefaultParamsGetter::getValue_haa_x() const
{
    return value.haa_x;
}

inline double DefaultParamsGetter::getValue_haa_z() const
{
    return 0;
}


inline void DefaultParamsGetter::setValue_haa_x(double val)
{
    value.haa_x = val;
}


inline double DefaultParamsGetter::getValue_haa_y() const
{
    return value.haa_y;
}


inline void DefaultParamsGetter::setValue_haa_y(double val)
{
    value.haa_y = val;
}


inline double DefaultParamsGetter::getValue_haa_hfe() const
{
    return value.haa_hfe;
}


inline void DefaultParamsGetter::setValue_haa_hfe(double val)
{
    value.haa_hfe = val;
}

inline double DefaultParamsGetter::getValue_upper_leg() const
{
    return value.upper_leg;
}


inline void DefaultParamsGetter::setValue_upper_leg(double val)
{
    value.upper_leg = val;
}

inline double DefaultParamsGetter::getValue_lower_leg() const
{
    return value.foot_x;
}


inline void DefaultParamsGetter::setValue_lower_leg(double val)
{
    value.foot_x = val;
}

} // HyQReal
} // dog
} // dls

#endif // IIT_HYQREAL_DEFAULT_PARAMS_GETTER_H_

