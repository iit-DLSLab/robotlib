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
#ifndef _ROBOTLIB_HYQREAL_FEET_JACOBIANS_HPP_
#define _ROBOTLIB_HYQREAL_FEET_JACOBIANS_HPP_

#include "robotlib/base/feet_jacobians.hpp"
#include "robotlib/base/leg_data_map.hpp"

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/joint_id_tricks.hpp"
#include "robotlib/hyqreal/jacobians.h"

#include "robotlib/rbd/utils.h"

namespace dls {
namespace dog {
namespace HyQReal {

class FeetJacobians : public dls::dog::FeetJacobiansBase
{
typedef dog::JointState JointState;


public:
    FeetJacobians(HyQReal::Jacobians& jacobians) : jacs(jacobians)
    {
        lin_myGetters[dog::LF] = & FeetJacobians::getFootJacobianLF;
        lin_myGetters[dog::RF] = & FeetJacobians::getFootJacobianRF;
        lin_myGetters[dog::LH] = & FeetJacobians::getFootJacobianLH;
        lin_myGetters[dog::RH] = & FeetJacobians::getFootJacobianRH;
	ang_myGetters[dog::LF] = & FeetJacobians::getAngularFootJacobianLF;
        ang_myGetters[dog::RF] = & FeetJacobians::getAngularFootJacobianRF;
        ang_myGetters[dog::LH] = & FeetJacobians::getAngularFootJacobianLH;
        ang_myGetters[dog::RH] = & FeetJacobians::getAngularFootJacobianRH;

    }
    ~FeetJacobians() {}

    dog::FootJac getFootJacobian(const JointState& q, const dog::LegID& leg) {
        return ((*this).*(lin_myGetters[leg]))(q);
    }

    dog::FootJac getFootJacobianLF(const JointState& q) {
        jacs.updateParameters();
        return jacs.fr_trunk_J_LF_foot(q).block<3,3>(dls::rbd::LX,0);
    }

    dog::FootJac getFootJacobianRF(const JointState& q) {
        jacs.updateParameters();
        return jacs.fr_trunk_J_RF_foot(q).block<3,3>(dls::rbd::LX,0);
    }

    dog::FootJac getFootJacobianLH(const JointState& q)  {
        jacs.updateParameters();
        return jacs.fr_trunk_J_LH_foot(q).block<3,3>(dls::rbd::LX,0);
    }

    dog::FootJac getFootJacobianRH(const JointState& q)  {
        jacs.updateParameters();
        return jacs.fr_trunk_J_RH_foot(q).block<3,3>(dls::rbd::LX,0);
    }


   dog::FootJac getFootJacobian(const JointState &q, const dog::LegID &leg,
                                const double& foot_x, const double& foot_y)
   {
       jacs.updateParameters();
       return jacs.getFootJacobianXY(q, leg, foot_x, foot_y);
   }


   dog::FootJac getAngularFootJacobian(const JointState& q, const dog::LegID& leg) {
       return ((*this).*(ang_myGetters[leg]))(q);
   }

   dog::FootJac getAngularFootJacobianLF(const JointState& q) {
       jacs.updateParameters();
       return jacs.fr_trunk_J_LF_foot(q).block<3,3>(dls::rbd::AX,0);
   }

   dog::FootJac getAngularFootJacobianRF(const JointState& q) {
       jacs.updateParameters();
       return jacs.fr_trunk_J_RF_foot(q).block<3,3>(dls::rbd::AX,0);
   }

   dog::FootJac getAngularFootJacobianLH(const JointState& q)  {
       jacs.updateParameters();
       return jacs.fr_trunk_J_LH_foot(q).block<3,3>(dls::rbd::AX,0);
   }

   dog::FootJac getAngularFootJacobianRH(const JointState& q)  {
       jacs.updateParameters();
       return jacs.fr_trunk_J_RH_foot(q).block<3,3>(dls::rbd::AX,0);
   }


  /*dog::FootJac getAngularFootJacobian(const JointState &q, const dog::LegID &leg,
			       const double& foot_x, const double& foot_y)
  {
      jacs.updateParameters();
      return jacs.getAngularFootJacobianXY(q, leg, foot_x, foot_y);
  }*/


private:
    HyQReal::Jacobians& jacs;

    typedef  dog::FootJac(FeetJacobians::*getter)(const JointState&);

    dog::LegDataMap< getter > lin_myGetters, ang_myGetters;
};


} // HyQReal
} // dog
} // dls

#endif
