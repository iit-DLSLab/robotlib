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
#ifndef DOGLIB_HYQREAL_FORWARD_KINEMATICS_HPP
#define DOGLIB_HYQREAL_FORWARD_KINEMATICS_HPP

#include "doglib/base/leg_data_map.hpp"
#include "doglib/base/forward_kinematics.hpp"
#include "doglib/base/declarations.hpp"
#include "doglib/hyqreal/transforms.h"
#include "doglib/rbd/utils.h"

namespace dls {
namespace dog {
namespace HyQReal {

class ForwardKinematics : public dog::ForwardKinematicsBase
{
public:
    ForwardKinematics(const dog::KinDynParamsBase &param_getter) :
        param_getter_(param_getter), transforms_(param_getter_)
    {
        myGetters[dog::LF] = & ForwardKinematics::getFootPosLF;
        myGetters[dog::RF] = & ForwardKinematics::getFootPosRF;
        myGetters[dog::LH] = & ForwardKinematics::getFootPosLH;
        myGetters[dog::RH] = & ForwardKinematics::getFootPosRH;
    }
    ~ForwardKinematics() {}

    dog::Vector3d getFootPosLF(const dog::JointState& q) {
        transforms_.updateParameters();
        return dls::rbd::Utils::positionVector( transforms_.fr_trunk_X_LF_foot(q) );
    }

    dog::Vector3d getFootPosRF(const dog::JointState& q) {
        transforms_.updateParameters();
        return dls::rbd::Utils::positionVector( transforms_.fr_trunk_X_RF_foot(q) );
    }

    dog::Vector3d getFootPosLH(const dog::JointState& q)  {
        transforms_.updateParameters();
        return dls::rbd::Utils::positionVector( transforms_.fr_trunk_X_LH_foot(q) );
    }

    dog::Vector3d getFootPosRH(const dog::JointState& q)  {
        transforms_.updateParameters();
        return dls::rbd::Utils::positionVector( transforms_.fr_trunk_X_RH_foot(q) );
    }

    dog::Vector3d getFootPos(const dog::JointState& q, const dog::LegID& leg) {
        return ((*this).*(myGetters[leg]))(q);
    }

    dog::Matrix3d getFootOrientation(const dog::JointState &q, const dog::LegID &leg){
        switch(leg){
        case dog::LF:
            transforms_.updateParameters();
            return dls::rbd::Utils::rotationMx(transforms_.fr_trunk_X_LF_foot(q));
        case dog::RF:
            transforms_.updateParameters();
            return dls::rbd::Utils::rotationMx(transforms_.fr_trunk_X_RF_foot(q));
        case dog::LH:
            transforms_.updateParameters();
            return dls::rbd::Utils::rotationMx(transforms_.fr_trunk_X_LH_foot(q));
        case dog::RH:
            transforms_.updateParameters();
            return dls::rbd::Utils::rotationMx(transforms_.fr_trunk_X_RH_foot(q));
        }
        return dog::Matrix3d::Identity();
    }
    dog::Vector3d getShinPos(const dog::JointState& q,
                             const double& contact_pos,
                             const dog::LegID& leg)
    {
        return getFootPos(q,leg);
        // TODO generate shin contacts
        /*switch(leg){
        case dog::LF:
            param_getter_.setValue_LF_shin(contact_pos);
            return dls::rbd::Utils::positionVector(transforms_.fr_trunk_X_LF_shin(q));
        case dog::RF:
            param_getter_.setValue_RF_shin(contact_pos);
            return dls::rbd::Utils::positionVector(transforms_.fr_trunk_X_RF_shin(q));
        case dog::LH:
            param_getter_.setValue_LH_shin(contact_pos);
            return dls::rbd::Utils::positionVector(transforms_.fr_trunk_X_LH_shin(q));
        case dog::RH:
            param_getter_.setValue_RH_shin(contact_pos);
            return dls::rbd::Utils::positionVector(transforms_.fr_trunk_X_RH_shin(q));
        }
        return dog::Vector3d::Zero();*/
    }

private:
    const dog::KinDynParamsBase& param_getter_;
    HyQReal::HomogeneousTransforms transforms_;

    typedef  dog::Vector3d(ForwardKinematics::*getter)(const dog::JointState&);

    dog::LegDataMap< getter > myGetters;
};


} // HyQReal
} // dog
} // dls


#endif
