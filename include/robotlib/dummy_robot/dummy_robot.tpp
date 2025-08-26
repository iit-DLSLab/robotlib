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
                                                    const DynParams& dynamic_parameters,
                                                    Container<LimbBase, NLIMBS>& limbs)
        : Robot<NLIMBS, NLINKS, NJOINTS>(name, dynamic_parameters, limbs)
	{
        // Set joint limits (dummy limits are used here)
        const double q_min {0};
        const double q_max {90};
        const double qd_max {3};
        const double tau_max {5};

        for (auto& joint: this->getJoints())
        {
            joint.setJointLimits(q_min, q_max, qd_max, tau_max);
        }
	}
    
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeFramePosition(const JointState &q, const Frame& origin, const Frame& destination)
    {
        q.size();
        origin.getName();
        destination.getName();

        return Eigen::Vector3d().setZero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeFrameOrientation(const robotlib::JointState& q,
                                                                              const robotlib::Frame& origin,
                                                                              const robotlib::Frame& destination)
    {
        q.size();
        origin.getName();
        destination.getName();

        return Eigen::Matrix3d().setZero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix4d DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeFramePose(const robotlib::JointState& q,
                                                                          const robotlib::Frame& origin,
                                                                          const robotlib::Frame& destination)
    {
        Eigen::Matrix4d frame_pose{};
        frame_pose.setZero();

        frame_pose.block(0, 3, 3, 1) << computeFramePosition(q, origin, destination);
        frame_pose.block(0, 0, 3, 3) << computeFrameOrientation(q, origin, destination);
        frame_pose.row(3) << 0, 0, 0, 1;

        return frame_pose;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeLimbsJacobian(const JointState& q,
							  const Frame& frame,
                              Eigen::MatrixXd &jacobian)
	{
		q.size();
		frame.getName();
	    jacobian.setZero();
	}

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeWholeBodyJacobian(const Eigen::Matrix<double, 7, 1> &robot_pose,
                                  const robotlib::JointState &q,
                                  const Frame& frame,
                                  Eigen::MatrixXd &jacobian)
    {
        
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    double DummyRobot<NLIMBS, NLINKS, NJOINTS>::getRobotMass() const
    {
        return 0.0;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    double DummyRobot<NLIMBS, NLINKS, NJOINTS>::getLinkMass(const Link& link) const
    {
        return 0.0;
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Matrix3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getLinkInertia(const Link& link) const
    {
        return Eigen::Matrix3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::getLinkCoM(const Link& link) const
    {
        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    Eigen::Vector3d DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeWholeBodyCoM(const robotlib::JointState& joint_position)
    {
        joint_position.size();

        return Eigen::Vector3d::Zero();
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
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::forwardKinematics(const JointState &joint_position,
                                LimbDataMap<Eigen::Vector3d> &end_effector_position){

    }

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::forwardKinematics(const JointState &joint_position,
                                const JointState &joint_velocity,
                                LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                LimbDataMap<Eigen::Vector3d> &end_effector_velocity){

}

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::fixedBaseInverseKinematics(const std::string &frame_name,
                                        const robotlib::JointState &q_init_guess,
                                        const Eigen::Vector3d &position_des,
                                        robotlib::JointState &q_des){

}
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::fixedBaseInverseKinematics(const robotlib::JointState &q_init_guess,
                                        const robotlib::LimbDataMap<Eigen::Vector3d> &positions_des,
                                        robotlib::JointState &q_des){

}
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::fixedBaseInverseDiffKinematics(const std::string &frame_name,
                                                    const robotlib::JointState &q,
                                                    const Eigen::Vector3d &velocity_des,
                                                    robotlib::JointState &qd_des){

}
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::fixedBaseInverseDiffKinematics(const robotlib::JointState &q,
                                            const robotlib::LimbDataMap<Eigen::Vector3d> &velocities_des,
                                            robotlib::JointState &qd_des){

}
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::inverseDynamics(
                        const Eigen::Matrix<double, 7, 1> &robot_pose,    // robot base
                        const Eigen::Matrix<double, 6, 1> &robot_velocity,
                        const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                        const robotlib::JointState &joint_position,
                        const robotlib::JointState &joint_velocity,
                        const robotlib::JointState &joint_acceleration,
                        const robotlib::eigen::aligned_map<std::string, Eigen::Vector3d> &f_contact,
                        robotlib::JointState &tau_joints){

}

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeGravityTerm(    const Eigen::Matrix<double, 7, 1> &robot_pose,
                                    const robotlib::JointState &joint_position,
                                    Eigen::Matrix<double, 6, 1> &g_base,
                                    robotlib::JointState &g_joints){

}
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                    const robotlib::JointState &joint_position,
                                    robotlib::JointState &g_joints){

}
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                Eigen::Matrix<double, 6, 1> &nle_base,
                                robotlib::JointState &nle_joints){

}

    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    void DummyRobot<NLIMBS, NLINKS, NJOINTS>::computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                robotlib::JointState &nle_joints){

}

} // namespace robotlib