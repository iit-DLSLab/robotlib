#include "commons/geometry/rotations.h"
#include "robotlib/hyqreal/feet_contact_forces.h"
#include "robotlib/hyqreal/default_parameters_getter.h"
#include "robotlib/base/joint_id_tricks.hpp"

namespace dls {
namespace dog {
namespace HyQReal {

FeetContactForces::FeetContactForces(FeetJacobians& feet_jacobians,
                                     dog::InverseDynamicsBase &inverse_dynamics,
                                     dog::JSIMBase &jsim) :
    feet_jacobians_(feet_jacobians),
    inverse_dynamics_(inverse_dynamics),
    jsim_(jsim)
{
    DefaultParamsGetter pgetter;
    foot_x = pgetter.getValue_foot_x();
}

bool FeetContactForces::getFootGRF(const JointState &q,
                                   const JointState &qd,
                                   const JointState &tau,
                                   const Quaterniond& orient,
                                   const LegID& leg,
                                   Vector3d &foot_grf,
                                   const JointState& qdd,
                                   const Vector3d& xd,
                                   const Vector3d& xdd,
                                   const Vector3d& omega,
                                   const Vector3d& omegad)
{
    Eigen::Matrix3d foot_jacobian = feet_jacobians_.getFootJacobian(q, leg, foot_x[leg], foot_y[leg]);

    dls::rbd::Vector6D gravity_world = dls::rbd::Vector6D::Zero();
    gravity_world(dls::rbd::LZ) = -dls::rbd::g;

    dls::rbd::Vector6D gravity_base = dls::rbd::Vector6D::Zero();
    dls::rbd::Vector6D base_acceleration = dls::rbd::Vector6D::Zero();
    dls::rbd::VelocityVector base_twist = dls::rbd::Vector6D::Zero();

    Eigen::Matrix3d R = dls::commons::quatToRotMat(orient);

    gravity_base.segment(dls::rbd::LX, 3) = R * gravity_world.segment(dls::rbd::LX, 3);

    base_acceleration.segment(dls::rbd::LX, 3) = xdd; //this is the absolute accel of the trunk without the gravity!!!
    base_acceleration.segment(dls::rbd::AX, 3) = omegad;

    base_twist.segment(dls::rbd::AX, 3) = omega;
    base_twist.segment(dls::rbd::LX, 3) = xd;

    dls::rbd::ForceVector h_base;
    JointState  h_joints;

    // Update the Joint Space Inertia Matrix with latest encoder values
    jsim_(q);


    // If we set all accelerations to zero, we basically compute the h_base and
    // h_joints components of the dynamics equation
    inverse_dynamics_.id_fully_actuated(h_base,
                                        h_joints,
                                        gravity_base,
                                        base_twist,
                                        base_acceleration,
                                        q,
                                        qd,
                                        qdd);

    // 3 joints of the LF leg, on a 3x1 vector;
    Eigen::Vector3d tau_leg = dog::getLegJointState(LegID(leg), tau);
    Eigen::Vector3d h_leg = dog::getLegJointState(LegID(leg), h_joints);
    Eigen::Matrix3d M_leg = jsim_.getFixedBaseBlock().block<3, 3>(leg * 3, leg * 3);
    Eigen::Matrix<double, 6, 3> F_leg = jsim_.getF().block<6, 3>(0, leg * 3);
    Eigen::Vector3d qdd_leg = qdd.block<3, 1>(leg * 3, 0);


    foot_grf = -(foot_jacobian.transpose()).inverse() *
                (tau_leg - h_leg  - F_leg.transpose() * base_acceleration -
                 M_leg * qdd_leg);
    return true;
}

void FeetContactForces::setContactPoint(dog::LegID leg, double user_foot_x, double user_foot_y)
{
    foot_x[leg] = user_foot_x;
    foot_y[leg] = user_foot_y;
}

} // HyQReal
} // dog
} // dls


