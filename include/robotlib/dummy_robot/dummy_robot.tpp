/**
 * @file dummy_robot.tpp
 *
 * @brief Dummy robot class and functions implementation
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "dummy_robot.hpp"

namespace robotlib
{
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    DummyRobot<NLIMBS, NLINKS, NJOINTS>::DummyRobot(const std::string& name,
                                                    Trunk& trunk,
                                                    Container<LimbBase, NLIMBS>& limbs)
        : Robot<NLIMBS, NLINKS, NJOINTS>(name, trunk, limbs)
	{
        // Set joint limits (dummy limits are used here)
        const double q_min {0};
        const double q_max {90};
        const double qd_max {3};
        const double tau_max {5};

        // for (auto& joint: this->getJoints())
        // {
        //     joint.setJointLimits(q_min, q_max, qd_max, tau_max);
        // }
	}
    
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFramePosition(const JointState &q, const Frame& origin, const Frame& destination) const
    {
        q.size();
        origin.getName();
        destination.getName();

        return Eigen::Vector3d().setZero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFrameOrientation(const robotlib::JointState& q,
                                                                              const robotlib::Frame& origin,
                                                                              const robotlib::Frame& destination) const
    {
        q.size();
        origin.getName();
        destination.getName();

        return Eigen::Matrix3d().setZero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix4d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFramePose(const robotlib::JointState& q,
                                                                      const robotlib::Frame& origin,
                                                                      const robotlib::Frame& destination) const
    {
        Eigen::Matrix4d frame_pose{};
        frame_pose.setZero();

        frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
        frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
        frame_pose.row(3) << 0, 0, 0, 1;

        return frame_pose;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFootPosition(const robotlib::JointState& q,
                                                                         const robotlib::Frame& foot) const
    {
        return this->getFramePosition(q, this->getLink(this->trunk_.getName()), foot);
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFootOrientation(const robotlib::JointState& q,
                                                                            const robotlib::Frame& foot) const
    {
        return this->getFrameOrientation(q, this->getLink(this->trunk_.getName()), foot);
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix4d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFootPose(const robotlib::JointState& q,
                                                                     const robotlib::Frame& foot) const
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::updateLinearJacobian(const robotlib::JointState& joints_positions,
                                                                   robotlib::LimbDataMap<robotlib::Jacobian>& robot_jacobian) const
    {
        joints_positions.size();
        robot_jacobian.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::updateAngularJacobian(const JointState& joints_positions,
                                                                    LimbDataMap<Jacobian>& robot_jacobian) const
    {
        joints_positions.size();
        robot_jacobian.size();
        std::cout << "Update Angular Jacobian" << std::endl;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::getFootJacobian(const JointState& q,
									                          const LimbBase& limb,
									                          Jacobian& footJac) const
    {
        q.size();
        limb.getName();
        footJac.setZero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::updateLinearFootJacobian(const JointState& joints_positions,
                                                                       const LimbBase& limb,
                                                                       Jacobian& footJac) const
    {
        joints_positions.size();
        limb.getName();
        footJac.size();
        std::cout << "Update Linear Foot Jacobian" << std::endl;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::updateAngularFootJacobian(const JointState& joints_positions,
                                                                        const LimbBase& limb,
                                                                        Jacobian& footJac) const
    {
        joints_positions.size();
        limb.getName();
        footJac.size();
        std::cout << "Update Angular Foot Jacobian" << std::endl;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::forwardKinematics(const robotlib::JointState& joint_position,
                                                                robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position) const
    {
        joint_position.size();
        end_effector_position.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::forwardKinematics(const robotlib::JointState& joint_position,
                                                                const robotlib::JointState& joint_velocity,
                                                                robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position,
                                                                robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_velocity) const
    {
        joint_position.size();
        joint_velocity.size();
        end_effector_position.size();
        end_effector_velocity.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::inverseKinematics(const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position,
                                                                const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_velocity,
                                                                const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_acceleration,
                                                                robotlib::JointState& joint_position, 
                                                                robotlib::JointState& joint_velocity,
                                                                robotlib::JointState& joint_acceleration) const
    {
        end_effector_position.size();
        end_effector_velocity.size();
        end_effector_acceleration.size();
        joint_position.size();
        joint_velocity.size();
        joint_acceleration.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::inverseKinematics(const LimbDataMap<Eigen::Vector3d> &end_effector_position,
							   const LimbDataMap<Eigen::Vector3d> &end_effector_velocity,
							   JointState &joint_position,
							   JointState &joint_velocity) const
    {
        end_effector_position.size();
        end_effector_velocity.size();
        joint_position.size();
        joint_velocity.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::inverseKinematics(const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position,
                                                                robotlib::JointState& joint_position) const
    {
        end_effector_position.size();
        joint_position.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::inverseDynamics(const Eigen::Matrix<double, 6, 1>& robot_velocity,
                                                              const Eigen::Matrix<double, 6, 1>& robot_acceleration,
                                                              const Eigen::Matrix<double, 6, 1>& gravity_vector,
                                                              const robotlib::JointState& joint_position,
                                                              const robotlib::JointState& joint_velocity,
                                                              const robotlib::JointState& joint_acceleration,
                                                              Eigen::Matrix<double, 6, 1>& wrench_base, ///output
                                                              robotlib::JointState& tau_joints) const		  ///output
    {
        robot_velocity.size();
        robot_acceleration.size();
        gravity_vector.size();
        joint_position.size();
        joint_velocity.size();
        joint_acceleration.size();
        wrench_base.size();
        tau_joints.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::inverseDynamicsHTerm(JointState &tau_joints,
                                                                   const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                   const JointState &joint_position,
                                                                   const JointState &joint_velocity,
                                                                   const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                                                   const Eigen::Matrix<double, 6, 1> &robot_acceleration) const
    {
        tau_joints.size();
        gravity_vector.size();
        joint_position.size();
        joint_velocity.size();
        robot_velocity.size();
        robot_acceleration.size();

        std::cout << "inverseDynamicsHTerm" << std::endl;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeGravityCompensation(const Eigen::Matrix<double, 6, 1>& gravity_vector,
                                                                         const robotlib::JointState& joint_position,
                                                                         Eigen::Matrix<double, 6, 1>& wrench_base, ///output
                                                                        robotlib::JointState& tau_joints) const        ///output
    {
        gravity_vector.size();
        wrench_base.size();
        joint_position.size();
        tau_joints.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix<double, 6,1> DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                                                     const JointState &joint_position) const
    {
        gravity_vector.size();
        joint_position.size();

        std::cout << "computeWrenchGravityCompensation" << std::endl;

        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeTorquesGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                                const JointState &joint_position,
                                                                                JointState &tau_joints) const
    {
        gravity_vector.size();
        joint_position.size();
        tau_joints.size();

        std::cout << "computeTorquesGravityCompensation" << std::endl;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    double DummyRobot<NLIMBS, NLINKS, NJOINTS>::getRobotMass() const
    {
        return 0.0;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    double DummyRobot<NLIMBS, NLINKS, NJOINTS>::getTrunkMass() const
    {
        return 0.0;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    double DummyRobot<NLIMBS, NLINKS, NJOINTS>::getLegsMass() const
    {
        return 0.0;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getRobotCoM() const
    {
        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getWholeBodyCOM(const robotlib::JointState& joint_position) const
    {
        joint_position.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getLegContribution(const robotlib::JointState& q) const
    {
        q.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getCoMFromBase(const robotlib::JointState& q,
                                                                        const Eigen::Vector3d& base_orient,
                                                                        const Eigen::Vector3d& base_pos) const
    {
        q.size();
        base_orient.size();
        base_pos.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getBaseFromCoM(const robotlib::JointState& q,
                                                                        const Eigen::Vector3d& base_orient,
                                                                        const Eigen::Vector3d& CoM) const
    {
        q.size();
        base_orient.size();
        CoM.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix<double, 6,1> DummyRobot<NLIMBS, NLINKS, NJOINTS>::getWholeBodyCOMVel(const JointState& q,
                                                                                       const JointState& qd) const
    {
        q.size();
        qd.size();

        return Eigen::Matrix<double, 6,1>::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix<double, 6, 1> DummyRobot<NLIMBS, NLINKS, NJOINTS>::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1>& baseVel,
                                                                                          const Eigen::Matrix3d& R,
                                                                                          const robotlib::JointState& q) const
    {
        baseVel.size();
        R.size();
        q.size();

        return Eigen::Matrix<double, 6, 1>::Zero();	
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix<double, 6, 1> DummyRobot<NLIMBS, NLINKS, NJOINTS>::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1>& baseVel,
                                                                                          const Eigen::Matrix3d& R,
                                                                                          const Eigen::Vector3d& offset_com) const
    {	
        baseVel.size();
        R.size();
        offset_com.size();

        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix4d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getImuBaseOffset(const std::string& imu_link_name, 
                                                                          const std::string& base_link_name) const
    {
        std::cout << imu_link_name << std::endl;
        std::cout << base_link_name << std::endl;

        return Eigen::Matrix4d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::setInvKinTimePeriod(const double period)
    {
        std::cout << period << std::endl;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::setTrunkCom(const Eigen::Vector3d& trunk_com)
    {
        trunk_com.size();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::setTrunkMass(const double trunk_mass)
    { 
        std::cout << trunk_mass << std::endl; 
    }
} // namespace robotlib