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
    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::DummyRobot(const std::string &name,
                                                                                                    const std::shared_ptr<Trunk> trunk,
                                                                                                    const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
                                                                                                    const std::array<std::shared_ptr<LimbBase>, NARMS> arms)
                                                                                                    : trunk_(trunk),
                                                                                                      Robot<NJOINTS, NLINKS, NLEGS, NARMS>(name,
                                                                                                                                           trunk,
                                                                                                                                           std::make_shared<const Container<std::shared_ptr<LimbBase>, NLEGS>>(legs),
                                                                                                                                           std::make_shared<const Container<std::shared_ptr<LimbBase>, NARMS>>(arms))
	{
        std::array<std::shared_ptr<Joint>, NLEGS+NARMS> children{};

        unsigned int i{0};
        for (auto leg : *(this->getLegs()))
        {
            for (auto joint : *(leg->getJoints()))
            {
                children.at(i) = Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(joint->getName());
                break;
            }
            i++;
        }
        for (auto arm : *(this->getArms()))
        {
            for (auto joint : *(arm->getJoints()))
            {
                children.at(i) = Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(joint->getName());
                break;
            }
            i++;
        }

        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildrenOfTrunk(std::make_shared<Container<std::shared_ptr<Joint>, NLEGS+NARMS>>(children));

        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfLink(trunk_, nullptr);

        for (auto leg : *(this->getLegs()))
        {
            for (auto joint : *(leg->getJoints()))
            {
                const std::string child_name {leg->jointToChildName(joint)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildOfJoint(joint, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLink(child_name));

                const std::string parent_name {leg->jointToParentName(joint)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfJoint(joint, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLink(parent_name));
            }
        }

        for (auto leg : *(this->getLegs()))
        {
            for (auto link : *(leg->getLinks()))
            {
                const std::string child_name {leg->linkToChildName(link)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildOfLink(link, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(child_name));

                const std::string parent_name {leg->linkToParentName(link)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfLink(link, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(parent_name));
            }
        }

        for (auto arm : *(this->getArms()))
        {
            for (auto joint : *(arm->getJoints()))
            {
                const std::string child_name {arm->jointToChildName(joint)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildOfJoint(joint, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLink(child_name));

                const std::string parent_name {arm->jointToParentName(joint)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfJoint(joint, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLink(parent_name));
            }
        }

        for (auto arm : *(this->getArms()))
        {
            for (auto link : *(arm->getLinks()))
            {
                const std::string child_name {arm->linkToChildName(link)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildOfLink(link, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(child_name));

                const std::string parent_name {arm->linkToParentName(link)};
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfLink(link, Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(parent_name));
            }
        }

        // Set joint limits (dummy limits are used here)
        const double q_min {0};
        const double q_max {90};
        const double qd_max {3};
        const double tau_max {5};

        for (auto leg : *(this->getLegs()))
        {
            for (auto joint : *(leg->getJoints()))
            {
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setJointLimits(joint, q_min, q_max, qd_max, tau_max);
            }
        }

        for (auto arm : *(this->getArms()))
        {
            for (auto joint : *(arm->getJoints()))
            {
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setJointLimits(joint, q_min, q_max, qd_max, tau_max);
            }
        }
	}
    
    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::~DummyRobot(){}

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFramePosition(const RobotBase::JointState &q,
                                                                                                                          const std::shared_ptr<Frame> origin,
                                                                                                                          const std::shared_ptr<Frame> destination)
    {
        q.size();
        origin->getName();
        destination->getName();

        return Eigen::Vector3d().setZero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFrameOrientation(const RobotBase::JointState &q,
                                                                                                                             const std::shared_ptr<Frame> origin,
                                                                                                                             const std::shared_ptr<Frame> destination)
    {
        q.size();
        origin->getName();
        destination->getName();

        return Eigen::Matrix3d().setZero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix4d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFramePose(const RobotBase::JointState &q,
                                                                                                                      const std::shared_ptr<Frame> origin,
                                                                                                                      const std::shared_ptr<Frame> destination)
    {
        Eigen::Matrix4d frame_pose{};
        frame_pose.setZero();

        frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
        frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
        frame_pose.row(3) << 0, 0, 0, 1;

        return frame_pose;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFootPosition(const RobotBase::JointState &q,
                                                                                                                         const std::shared_ptr<Frame> foot)
    {
        return this->getFramePosition(q, this->getLink(trunk_->getName()), foot);
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFootOrientation(const RobotBase::JointState &q,
                                                                                                                            const std::shared_ptr<Frame> foot)
    {
        return this->getFrameOrientation(q, this->getLink(trunk_->getName()), foot);
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix4d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFootPose(const RobotBase::JointState &q,
                                                                                                                     const std::shared_ptr<Frame> foot)
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFootPosition(const RobotBase::JointState &q,
    const std::shared_ptr<LimbBase> leg)
    {
        return this->getFramePosition(q, this->getLink(trunk_->getName()), leg->getEndEffector());
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFootOrientation(const RobotBase::JointState &q,
                                                                                                                            const std::shared_ptr<LimbBase> leg)
    {
        return this->getFrameOrientation(q, this->getLink(trunk_->getName()), leg->getEndEffector());
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix4d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getFootPose(const RobotBase::JointState &q,
                                                                                                                     const std::shared_ptr<LimbBase> leg)
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::updateLinearJacobian(const RobotBase::JointState &joints_positions,
                                                                                                                   RobotBase::LegDataMap<RobotBase::Jacobian> &robot_jacobian)
    {
        joints_positions.size();
        robot_jacobian.getSize();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::forwardKinematics(
        const RobotBase::JointState &joint_position,
        RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_position)
    {
        joint_position.size();
        end_effector_position.getSize();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::forwardKinematics(const RobotBase::JointState &joint_position,
                                                                                                                const RobotBase::JointState &joint_velocity,
                                                                                                                RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                                                                                                RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_velocity)
    {
        joint_position.size();
        joint_velocity.size();
        end_effector_position.getSize();
        end_effector_velocity.getSize();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::inverseKinematics(const RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                                                                                                const RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                                                                                                const RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
                                                                                                                RobotBase::JointState &joint_position,
                                                                                                                RobotBase::JointState &joint_velocity,
                                                                                                                RobotBase::JointState &joint_acceleration)
    {
        end_effector_position.getSize();
        end_effector_velocity.getSize();
        end_effector_acceleration.getSize();
        joint_position.size();
        joint_velocity.size();
        joint_acceleration.size();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::inverseKinematics(const RobotBase::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                                                                                                RobotBase::JointState &joint_position)
    {
        end_effector_position.getSize();
        joint_position.size();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                                                                                              const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                                                                                              const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                                                              const RobotBase::JointState &joint_position,
                                                                                                              const RobotBase::JointState &joint_velocity,
                                                                                                              const RobotBase::JointState &joint_acceleration,
                                                                                                              Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                                                                                              RobotBase::JointState &tau_joints)		  ///output
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

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                                                                         const RobotBase::JointState &joint_position,
                                                                                                                         Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                                                                                                         RobotBase::JointState &tau_joints)        ///output
    {
        gravity_vector.size();
        wrench_base.size();
        joint_position.getSize();
        tau_joints.getSize();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    double DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getRobotMass() const
    {
        return 0.0;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    double DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getTrunkMass() const
    {
        return 0.0;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    double DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getLegsMass() const
    {
        return 0.0;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getWholeBodyCOM(const RobotBase::JointState &joint_position)
    {
        joint_position.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getLegContribution(const RobotBase::JointState &q)
    {
        q.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getCoMFromBase(const RobotBase::JointState & q,
                                                                                                                        const Eigen::Vector3d & base_orient,
                                                                                                                        const Eigen::Vector3d & base_pos)
    {
        q.size();
        base_orient.size();
        base_pos.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getBaseFromCoM(const RobotBase::JointState & q,
                                                                                                                        const Eigen::Vector3d & base_orient,
                                                                                                                        const Eigen::Vector3d & CoM)
    {
        q.size();
        base_orient.size();
        CoM.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                                                                                          const Eigen::Matrix3d & R,
                                                                                                                                          const RobotBase::JointState & q)
    {
        baseVel.size();
        R.size();
        q.size();

        return Eigen::Matrix<double, 6, 1>::Zero();	
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                 const Eigen::Matrix3d &R,
                                                 const Eigen::Vector3d offset_com)
    {	
        baseVel.size();
        R.size();
        offset_com.size();

        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::setInvKinTimePeriod(const double& period)
    {
        std::cout << period << std::endl;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::setTrunkCom(const Eigen::Vector3d &trunk_com)
    {
        trunk_com.size();
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot::setTrunkMass(const double& trunk_mass)
    { 
        std::cout << trunk_mass << std::endl; 
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg::DummyLeg(const std::string &leg_name,
                                                                                                const std::string &trunk_name,
                                                                                                const std::array<std::shared_ptr<Joint>, NJOINTSLEG> &leg_joints,
                                                                                                const std::array<std::shared_ptr<Link>, NLINKSLEG> &leg_links) 
                                                                                                : Leg<NJOINTSLEG, NLINKSLEG>(leg_name, leg_joints, leg_links)
    {
        /*
        joints_map_ = {joint name, parent name, child name}
        links_map_ = {link name, parent name, child name}
        */

        auto trunk_child = std::make_pair(trunk_name, (leg_links.at(0))->getName());
        joints_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((leg_joints.at(0))->getName(), trunk_child));

        for(unsigned int i{0}; i<NJOINTSLEG-1; i++)
        {
            auto parent_child = std::make_pair((leg_links.at(i))->getName(), (leg_links.at(i+1))->getName());
            joints_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((leg_joints.at(i+1))->getName(), parent_child));
        }

        for(unsigned int i{0}; i<NLINKSLEG-1; i++)
        {
            auto parent_child = std::make_pair((leg_joints.at(i))->getName(), (leg_joints.at(i+1))->getName());
            links_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((leg_links.at(i))->getName(), parent_child));
        }

        auto link_no_child = std::make_pair((leg_joints.at(NJOINTSLEG-1))->getName(), "");
        links_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((leg_links.at(NLINKSLEG-1))->getName(), link_no_child));
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg::~DummyLeg(){}

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg::jointToChildName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string child_name {joints_map_.find(joint_name)->second.second}; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
        return child_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg::jointToParentName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string parent_name {joints_map_.find(joint_name)->second.first};
        return parent_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg::linkToChildName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string child_name {links_map_.find(link_name)->second.second};
        return child_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg::linkToParentName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string parent_name {links_map_.find(link_name)->second.first};
        return parent_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm::DummyArm(const std::string &arm_name,
                                                                                                const std::string &trunk_name,
                                                                                                const std::array<std::shared_ptr<Joint>, NJOINTSARM> &arm_joints,
                                                                                                const std::array<std::shared_ptr<Link>, NLINKSARM> &arm_links)
                                                                                                : Arm<NJOINTSARM, NLINKSARM>(arm_name, arm_joints, arm_links)
    {
        /*
        joints_map_ = {joint name, parent name, child name}
        links_map_ = {link name, parent name, child name}
        */

        auto trunk_child = std::make_pair(trunk_name, (arm_links.at(0))->getName());
        joints_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((arm_joints.at(0))->getName(), trunk_child));

        for(unsigned int i{0}; i<NJOINTSARM-1; i++)
        {
            auto parent_child = std::make_pair((arm_links.at(i))->getName(), (arm_links.at(i+1))->getName());
            joints_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((arm_joints.at(i+1))->getName(), parent_child));
        }

        for(unsigned int i{0}; i<NLINKSARM-1; i++)
        {
            auto parent_child = std::make_pair((arm_joints.at(i))->getName(), (arm_joints.at(i+1))->getName());
            links_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((arm_links.at(i))->getName(), parent_child));
        }

        auto link_no_child = std::make_pair((arm_joints.at(NJOINTSARM-1))->getName(), "");
        links_map_.insert(std::pair<std::string, std::pair<std::string, std::string>>((arm_links.at(NLINKSARM-1))->getName(), link_no_child));
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm::~DummyArm(){}

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm::jointToChildName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string child_name {joints_map_.find(joint_name)->second.second}; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
        return child_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm::jointToParentName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string parent_name {joints_map_.find(joint_name)->second.first};
        return parent_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm::linkToChildName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string child_name {links_map_.find(link_name)->second.second};
        return child_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm::linkToParentName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string parent_name {links_map_.find(link_name)->second.first};
        return parent_name;
    }

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobotCreator(){}

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::~DummyRobotCreator(){}

    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS, unsigned int NJOINTSARM, unsigned int NLINKSARM>
    std::shared_ptr<RobotBase> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::createDummyRobot(const std::array<std::string, (2 + NJOINTS + NLINKS + NLEGS + NARMS)> &components_names)
    {
        const std::string name{components_names.at(0)};

        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
         
        const auto trunk = std::make_shared<Trunk>(components_names.at(1), trunk_dyn_params);

        std::array<std::shared_ptr<LimbBase>, NLEGS> legs({});
        std::array<std::shared_ptr<LimbBase>, NARMS> arms({});
        std::array<std::array<std::shared_ptr<Joint>, NJOINTSLEG>, NLEGS> legs_joints({});
        std::array<std::array<std::shared_ptr<Link>, NLINKSLEG>, NLEGS> legs_links({});
        std::array<std::array<std::shared_ptr<Joint>, NJOINTSARM>, NARMS> arms_joints({});
        std::array<std::array<std::shared_ptr<Link>, NLINKSARM>, NARMS> arms_links({});

        for(unsigned int i{0}; i<NLEGS; i++)
        {
            for(unsigned int j{0}; j<NJOINTSLEG; j++)
            {
                legs_joints.at(i).at(j) = std::make_shared<Joint>(components_names.at((i*NJOINTSLEG)+(j+2+NLEGS)));
            }

            for(unsigned int j{0}; j<NLINKSLEG; j++)
            {
                legs_links.at(i).at(j) = std::make_shared<Link>(components_names.at((i*NLINKSLEG)+(j+2+NLEGS+(NJOINTSLEG*NLEGS))));
            }

            legs.at(i) = std::make_shared<DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyLeg>(components_names.at(i+2), components_names.at(1), legs_joints.at(i), legs_links.at(i));
        }

        for(unsigned int i{0}; i<NARMS; i++)
        {
            for(unsigned int j{0}; j<NJOINTSARM; j++)
            {
                arms_joints.at(i).at(j) = std::make_shared<Joint>(components_names.at((i*NJOINTSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS)));
            }

            for(unsigned int j{0}; j<NLINKSARM; j++)
            {
                arms_links.at(i).at(j) = std::make_shared<Link>(components_names.at((i*NLINKSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS+(NJOINTSARM*NARMS))));
            }

            arms.at(i) = std::make_shared<DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyArm>(components_names.at(i+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)), components_names.at(1), arms_joints.at(i), arms_links.at(i));
        }

        return std::make_shared<DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NJOINTSLEG, NLINKSLEG, NARMS, NJOINTSARM, NLINKSARM>::DummyRobot>(name, trunk, legs, arms);
    }
} // namespace robotlib