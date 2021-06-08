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
#ifndef DOGLIB_HYQ_FORWARD_DYNAMICS_HPP
#define DOGLIB_HYQ_FORWARD_DYNAMICS_HPP

#include <Eigen/Dense>
#include "doglib/rbd/rbd.h"
#include "doglib/rbd/InertiaMatrix.h"
#include "doglib/rbd/utils.h"

#include "doglib/base/declarations.hpp"
#include "doglib/hyq/transforms.hpp"
#include "doglib/hyq/inertia_properties.hpp"
#include "doglib/base/link_data_map.hpp"

namespace dls {
namespace dog {
namespace HyQ {
namespace dyn {

typedef dls::rbd::InertiaMatrixDense InertiaMatrix;
typedef dls::dog::JointState JointState;

/**
 * The Forward Dynamics routine for the robot HyQ.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    // Convenient type aliases:
    typedef dog::LinkDataMap<dls::rbd::ForceVector> ExtForces;
    typedef dls::rbd::ForceVector Force;
    typedef dls::rbd::VelocityVector Velocity;
    typedef dls::rbd::VelocityVector Acceleration;
public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot HyQ, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param trunk_a
     * \param trunk_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& trunk_a, // output parameters,
       const Velocity& trunk_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& trunk_a, // output parameters,
        const Velocity& trunk_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    dls::rbd::Matrix66d spareMx; // support variable
    // Link 'trunk'
    InertiaMatrix trunk_AI;
    Force trunk_p;

    // Link 'LF_hipassembly' :
    InertiaMatrix LF_hipassembly_AI;
    Velocity LF_hipassembly_a;
    Velocity LF_hipassembly_v;
    Velocity LF_hipassembly_c;
    Force    LF_hipassembly_p;

    dls::rbd::Column6d LF_hipassembly_U;
    double LF_hipassembly_D;
    double LF_hipassembly_u;
    // Link 'LF_upperleg' :
    InertiaMatrix LF_upperleg_AI;
    Velocity LF_upperleg_a;
    Velocity LF_upperleg_v;
    Velocity LF_upperleg_c;
    Force    LF_upperleg_p;

    dls::rbd::Column6d LF_upperleg_U;
    double LF_upperleg_D;
    double LF_upperleg_u;
    // Link 'LF_lowerleg' :
    InertiaMatrix LF_lowerleg_AI;
    Velocity LF_lowerleg_a;
    Velocity LF_lowerleg_v;
    Velocity LF_lowerleg_c;
    Force    LF_lowerleg_p;

    dls::rbd::Column6d LF_lowerleg_U;
    double LF_lowerleg_D;
    double LF_lowerleg_u;
    // Link 'RF_hipassembly' :
    InertiaMatrix RF_hipassembly_AI;
    Velocity RF_hipassembly_a;
    Velocity RF_hipassembly_v;
    Velocity RF_hipassembly_c;
    Force    RF_hipassembly_p;

    dls::rbd::Column6d RF_hipassembly_U;
    double RF_hipassembly_D;
    double RF_hipassembly_u;
    // Link 'RF_upperleg' :
    InertiaMatrix RF_upperleg_AI;
    Velocity RF_upperleg_a;
    Velocity RF_upperleg_v;
    Velocity RF_upperleg_c;
    Force    RF_upperleg_p;

    dls::rbd::Column6d RF_upperleg_U;
    double RF_upperleg_D;
    double RF_upperleg_u;
    // Link 'RF_lowerleg' :
    InertiaMatrix RF_lowerleg_AI;
    Velocity RF_lowerleg_a;
    Velocity RF_lowerleg_v;
    Velocity RF_lowerleg_c;
    Force    RF_lowerleg_p;

    dls::rbd::Column6d RF_lowerleg_U;
    double RF_lowerleg_D;
    double RF_lowerleg_u;
    // Link 'LH_hipassembly' :
    InertiaMatrix LH_hipassembly_AI;
    Velocity LH_hipassembly_a;
    Velocity LH_hipassembly_v;
    Velocity LH_hipassembly_c;
    Force    LH_hipassembly_p;

    dls::rbd::Column6d LH_hipassembly_U;
    double LH_hipassembly_D;
    double LH_hipassembly_u;
    // Link 'LH_upperleg' :
    InertiaMatrix LH_upperleg_AI;
    Velocity LH_upperleg_a;
    Velocity LH_upperleg_v;
    Velocity LH_upperleg_c;
    Force    LH_upperleg_p;

    dls::rbd::Column6d LH_upperleg_U;
    double LH_upperleg_D;
    double LH_upperleg_u;
    // Link 'LH_lowerleg' :
    InertiaMatrix LH_lowerleg_AI;
    Velocity LH_lowerleg_a;
    Velocity LH_lowerleg_v;
    Velocity LH_lowerleg_c;
    Force    LH_lowerleg_p;

    dls::rbd::Column6d LH_lowerleg_U;
    double LH_lowerleg_D;
    double LH_lowerleg_u;
    // Link 'RH_hipassembly' :
    InertiaMatrix RH_hipassembly_AI;
    Velocity RH_hipassembly_a;
    Velocity RH_hipassembly_v;
    Velocity RH_hipassembly_c;
    Force    RH_hipassembly_p;

    dls::rbd::Column6d RH_hipassembly_U;
    double RH_hipassembly_D;
    double RH_hipassembly_u;
    // Link 'RH_upperleg' :
    InertiaMatrix RH_upperleg_AI;
    Velocity RH_upperleg_a;
    Velocity RH_upperleg_v;
    Velocity RH_upperleg_c;
    Force    RH_upperleg_p;

    dls::rbd::Column6d RH_upperleg_U;
    double RH_upperleg_D;
    double RH_upperleg_u;
    // Link 'RH_lowerleg' :
    InertiaMatrix RH_lowerleg_AI;
    Velocity RH_lowerleg_a;
    Velocity RH_lowerleg_v;
    Velocity RH_lowerleg_c;
    Force    RH_lowerleg_p;

    dls::rbd::Column6d RH_lowerleg_U;
    double RH_lowerleg_D;
    double RH_lowerleg_u;
private:
    static const ExtForces zeroExtForces;
};

} // dyn
} // HyQ
} // dog
} // dls

#endif
