/**
 * @file dummy_robot.tpp
 *
 * @brief Dummy robot class and functions implementation
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "dummy_robot.hpp"

namespace robotlib
{
    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::DummyRobot(const std::shared_ptr<Trunk> trunk,
                                                                                                    const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
                                                                                                    const std::array<std::shared_ptr<LimbBase>, NARMS> arms) 
                                                                                                    : Robot<NJOINTS, NLINKS, NLEGS, NARMS>("Dummy Robot",
                                                                                                                                           trunk,
                                                                                                                                           std::make_shared<const Container<std::shared_ptr<LimbBase>, NLEGS>>(legs),
                                                                                                                                           std::make_shared<const Container<std::shared_ptr<LimbBase>, NARMS>>(arms))
	{
        std::array<std::shared_ptr<Joint>, NLEGS> children;
        children[0] = Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint("LF_HAA");
        children[1] = Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint("RF_HAA");
        children[2] = Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint("LH_HAA");
        children[3] = Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint("RH_HAA");

        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildrenOfTrunk(std::make_shared<Container<std::shared_ptr<Joint>, NLEGS>>(children));

        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfLink(trunk, nullptr);

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
	}
    
    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::~DummyRobot(){}

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFramePosition(const RobotBase::JointState &q,
                                                                                const std::shared_ptr<Frame> origin,
                                                                                const std::shared_ptr<Frame> destination)
    {
        q.size();
        origin->getName();
        destination->getName();

        return Eigen::Vector3d().setZero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFrameOrientation(const RobotBase::JointState &q,
                                                                                   const std::shared_ptr<Frame> origin,
                                                                                   const std::shared_ptr<Frame> destination)
    {
        q.size();
        origin->getName();
        destination->getName();

        return Eigen::Matrix3d().setZero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix4d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFramePose(const RobotBase::JointState &q,
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

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPosition(const RobotBase::JointState &q,
                                                                               const std::shared_ptr<Frame> foot)
    {
        return this->getFramePosition(q, this->getLink("TRUNK"), foot);
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootOrientation(const RobotBase::JointState &q,
                                                                                  const std::shared_ptr<Frame> foot)
    {
        return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix4d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPose(const RobotBase::JointState &q,
                                                                           const std::shared_ptr<Frame> foot)
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPosition(const RobotBase::JointState &q,
                                                                    const std::shared_ptr<LimbBase> leg,
                                                                    Eigen::Vector3d &footPos)
    {
        footPos = this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootOrientation(const RobotBase::JointState &q,
                                                                                  const std::shared_ptr<LimbBase> leg)
    {
        return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix4d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPose(const RobotBase::JointState &q,
                                                                           const std::shared_ptr<LimbBase> leg)
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootJacobian(const RobotBase::JointState &q,
                                                                    const std::shared_ptr<LimbBase> leg,
                                                                    RobotBase::Jacobian &footJac)
    {
        q.size();
        leg->getName();

        footJac.setZero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::updateLinearJacobian(const RobotBase::JointState &joints_positions,
                                                                         RobotBase::LegDataMap<RobotBase::Jacobian> &robot_jacobian)
    {
        joints_positions.size();
        robot_jacobian.getSize();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    RobotBase::LegDataMap<std::shared_ptr<Frame>> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFeet()
    {
        auto feet = this->template makeLegDataMap<std::shared_ptr<Frame>>();

        for (auto leg : *(this->getLegs()))
        {
            feet[leg] = std::make_shared<Link>("link");
        }

        return feet;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::forwardKinematics(const RobotBase::JointState &joint_position,
                                                                      RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position)
    {
        joint_position.size();
        end_effector_position.getSize();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::forwardKinematics(const RobotBase::JointState &joint_position,
                                                                      const RobotBase::JointState &joint_velocity,
                                                                      RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                                                      RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity)
    {
        joint_position.size();
        joint_velocity.size();
        end_effector_position.getSize();
        end_effector_velocity.getSize();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::inverseKinematics(const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                                                      const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                                                      const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
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

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
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

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                               const RobotBase::JointState &joint_position,
                                                                               Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                                                               RobotBase::JointState &tau_joints)        ///output
    {
        gravity_vector.size();
        wrench_base.size();
        joint_position.getSize();
        tau_joints.getSize();

        std::cout << "Gravity compensation" << std::endl;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    double DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getRobotMass() const
    {
        return 0.0;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    double DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getTrunkMass() const
    {
        return 0.0;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    double DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getLegsMass() const
    {
        return 0.0;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getRobotCoM() { return Eigen::Vector3d().setZero(); }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 3, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOM()
    {
        return Eigen::Matrix<double, 3, 1>::Zero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 3, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOM(const RobotBase::JointState &joint_state)
    {
        joint_state.size();

        return Eigen::Matrix<double, 3, 1>::Zero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getLegContribution(const RobotBase::JointState &q)
    {
        q.size();

        return Eigen::Vector3d::Zero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getCoMFromBase(const RobotBase::JointState & q,
                                                                              const Eigen::Vector3d & base_orient,
                                                                              const Eigen::Vector3d & base_pos)
    {
        q.size();
        base_orient.size();
        base_pos.size();

        return Eigen::Vector3d::Zero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getBaseFromCoM(const RobotBase::JointState & q,
                                                                              const Eigen::Vector3d & base_orient,
                                                                              const Eigen::Vector3d & CoM)
    {
        q.size();
        base_orient.size();
        CoM.size();

        return Eigen::Vector3d::Zero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVel(const RobotBase::JointState & q,
                                                                                              const RobotBase::JointState & qd)
    {
        q.size();
        qd.size();

        return Eigen::Matrix<double, 6, 1>::Zero();		
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                                                const Eigen::Matrix3d & rotationMx,
                                                                                                const RobotBase::JointState & q,
                                                                                                const RobotBase::JointState & qd)
    {
        baseVel.size();
        rotationMx.size();
        q.size();
        qd.size();
        
        std::cout << "Get whole body COM vel FB - considering joint influence" << std::endl;

        return Eigen::Matrix<double, 6, 1>::Zero();	
    };

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                                                const Eigen::Matrix3d & rotationMx,
                                                                                                const RobotBase::JointState & q)
    {
        baseVel.size();
        rotationMx.size();
        q.size();
        
        std::cout << "Get whole body COM vel FB - without joint influence" << std::endl;

        return Eigen::Matrix<double, 6, 1>::Zero();	
    };

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                                                const Eigen::Matrix3d &rotationMx,
                                                                                                const Eigen::Vector3d offset_com)
    {	
        baseVel.size();
        rotationMx.size();
        offset_com.size();
        
        std::cout << "Get whole body COM vel FB, with com offset as input, without considering joint influence" << std::endl;

        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::setInvKinTimePeriod(const double& period){std::cout << period << std::endl;};

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::setTrunkCom(const Eigen::Vector3d &trunk_com) { trunk_com.size(); }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot::setTrunkMass(const double& trunk_mass){ std::cout << trunk_mass << std::endl; }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg::DummyLeg(const std::string &name,
                                                                                                const std::array<std::shared_ptr<Joint>, NJOINTSLEG> &joints,
                                                                                                const std::array<std::shared_ptr<Link>, NLINKSLEG> &links) 
                                                                                                : Leg<NJOINTSLEG, NLINKSLEG>(name, joints, links),
                                                                                                jointMap({//joint name, parent name, child name
                                                                                                        {"LF_HAA", std::make_pair("TRUNK", "LF_ASSEMBLY")},
                                                                                                        {"LF_HFE", std::make_pair("LF_ASSEMBLY", "LF_UPPERLEG")},
                                                                                                        {"LF_KFE", std::make_pair("LF_UPPERLEG", "LF_LOWERLEG")},
                                                                                                        {"RF_HAA", std::make_pair("TRUNK", "RF_ASSEMBLY")},
                                                                                                        {"RF_HFE", std::make_pair("RF_ASSEMBLY", "RF_UPPERLEG")},
                                                                                                        {"RF_KFE", std::make_pair("RF_UPPERLEG", "RF_LOWERLEG")},
                                                                                                        {"LH_HAA", std::make_pair("TRUNK", "LH_ASSEMBLY")},
                                                                                                        {"LH_HFE", std::make_pair("LH_ASSEMBLY", "LH_UPPERLEG")},
                                                                                                        {"LH_KFE", std::make_pair("LH_UPPERLEG", "LH_LOWERLEG")},
                                                                                                        {"RH_HAA", std::make_pair("TRUNK", "RH_ASSEMBLY")},
                                                                                                        {"RH_HFE", std::make_pair("RH_ASSEMBLY", "RH_UPPERLEG")},
                                                                                                        {"RH_KFE", std::make_pair("RH_UPPERLEG", "RH_LOWERLEG")}}),
                                                                                                linkMap({//link name, parent name, child name
                                                                                                        {"LF_ASSEMBLY", std::make_pair("LF_HAA", "LF_HFE")},
                                                                                                        {"LF_UPPERLEG", std::make_pair("LF_HFE", "LF_KFE")},
                                                                                                        {"LF_LOWERLEG", std::make_pair("LF_KFE", "")},
                                                                                                        {"RF_ASSEMBLY", std::make_pair("RF_HAA", "RF_HFE")},
                                                                                                        {"RF_UPPERLEG", std::make_pair("RF_HFE", "RF_KFE")},
                                                                                                        {"RF_LOWERLEG", std::make_pair("RF_KFE", "")},
                                                                                                        {"LH_ASSEMBLY", std::make_pair("LH_HAA", "LH_HFE")},
                                                                                                        {"LH_UPPERLEG", std::make_pair("LH_HFE", "LH_KFE")},
                                                                                                        {"LH_LOWERLEG", std::make_pair("LH_KFE", "")},
                                                                                                        {"RH_ASSEMBLY", std::make_pair("RH_HAA", "RH_HFE")},
                                                                                                        {"RH_UPPERLEG", std::make_pair("RH_HFE", "RH_KFE")},
                                                                                                        {"RH_LOWERLEG", std::make_pair("RH_KFE", "")}}){}

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg::~DummyLeg(){}

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg::jointToChildName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string child_name {jointMap.find(joint_name)->second.second}; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
        return child_name;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg::jointToParentName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string parent_name {jointMap.find(joint_name)->second.first};
        return parent_name;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg::linkToChildName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string child_name {linkMap.find(link_name)->second.second};
        return child_name;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg::linkToParentName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string parent_name {linkMap.find(link_name)->second.first};
        return parent_name;
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobotCreator(){}

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::~DummyRobotCreator(){}

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    std::shared_ptr<typename DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::makeLeg(const std::string &legName)
    {
        auto haa = std::make_shared<Joint>(legName + "_HAA");
        auto assembly = std::make_shared<Link>(legName + "_ASSEMBLY");
        auto hfe = std::make_shared<Joint>(legName + "_HFE");

        auto upperleg = std::make_shared<Link>(legName + "_UPPERLEG");
        auto kfe = std::make_shared<Joint>(legName + "_KFE");
        auto lowerleg = std::make_shared<Link>(legName + "_LOWERLEG");

        return std::make_shared<DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyLeg>(legName,
                                                                                                                   std::array<std::shared_ptr<Joint>, NJOINTSLEG>({haa, hfe, kfe}),
                                                                                                                   std::array<std::shared_ptr<Link>, NLINKSLEG>({assembly, upperleg, lowerleg}));
    }

    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    std::shared_ptr<RobotBase> DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::createDummyRobot()
    {
        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
         
        const auto trunk = std::make_shared<Trunk>("TRUNK", trunk_dyn_params);
        const std::array<std::shared_ptr<LimbBase>, NLEGS> legs(
            {makeLeg("LF"),
             makeLeg("RF"),
             makeLeg("LH"),
             makeLeg("RH")});
        const std::array<std::shared_ptr<LimbBase>, NARMS> arms({});

        return std::make_shared<DummyRobotCreator<NJOINTS, NLINKS, NLEGS, NARMS, NJOINTSLEG, NLINKSLEG>::DummyRobot>(trunk, legs, arms);
    }
} // namespace robotlib