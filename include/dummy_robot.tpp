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
    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::DummyRobot(const std::string &name,
                                                                                                    const std::shared_ptr<Trunk> trunk,
                                                                                                    const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
                                                                                                    const std::array<std::shared_ptr<LimbBase>, NARMS> arms)
                                                                                                    : trunk_(trunk),
                                                                                                      Robot<NJOINTS, NLINKS, NLEGS, NARMS>(name,
                                                                                                                                           trunk,
                                                                                                                                           std::make_shared<const Container<std::shared_ptr<LimbBase>, NLEGS>>(legs),
                                                                                                                                           std::make_shared<const Container<std::shared_ptr<LimbBase>, NARMS>>(arms))
	{
        std::array<std::shared_ptr<Joint>, NLEGS> children{};

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

        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildrenOfTrunk(std::make_shared<Container<std::shared_ptr<Joint>, NLEGS>>(children));

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
    
    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::~DummyRobot(){}

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFramePosition(const RobotBase::JointState &q,
                                                                                                                          const std::shared_ptr<Frame> origin,
                                                                                                                          const std::shared_ptr<Frame> destination)
    {
        q.size();
        origin->getName();
        destination->getName();

        return Eigen::Vector3d().setZero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFrameOrientation(const RobotBase::JointState &q,
                                                                                                                             const std::shared_ptr<Frame> origin,
                                                                                                                             const std::shared_ptr<Frame> destination)
    {
        q.size();
        origin->getName();
        destination->getName();

        return Eigen::Matrix3d().setZero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix4d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFramePose(const RobotBase::JointState &q,
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

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPosition(const RobotBase::JointState &q,
                                                                                                                         const std::shared_ptr<Frame> foot)
    {
        return this->getFramePosition(q, this->getLink(trunk_->getName()), foot);
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootOrientation(const RobotBase::JointState &q,
                                                                                                                            const std::shared_ptr<Frame> foot)
    {
        return this->getFrameOrientation(q, this->getLink(trunk_->getName()), foot);
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix4d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPose(const RobotBase::JointState &q,
                                                                                                                     const std::shared_ptr<Frame> foot)
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPosition(const RobotBase::JointState &q,
                                                                                                              const std::shared_ptr<LimbBase> leg,
                                                                                                              Eigen::Vector3d &footPos)
    {
        footPos = this->getFramePosition(q, this->getLink(trunk_->getName()), leg->getEndEffector());
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootOrientation(const RobotBase::JointState &q,
                                                                                                                            const std::shared_ptr<LimbBase> leg)
    {
        return this->getFrameOrientation(q, this->getLink(trunk_->getName()), leg->getEndEffector());
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix4d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootPose(const RobotBase::JointState &q,
                                                                                                                     const std::shared_ptr<LimbBase> leg)
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFootJacobian(const RobotBase::JointState &q,
                                                                                                              const std::shared_ptr<LimbBase> leg,
                                                                                                              RobotBase::Jacobian &footJac)
    {
        q.size();
        leg->getName();

        footJac.setZero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::updateLinearJacobian(const RobotBase::JointState &joints_positions,
                                                                                                                   RobotBase::LegDataMap<RobotBase::Jacobian> &robot_jacobian)
    {
        joints_positions.size();
        robot_jacobian.getSize();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    RobotBase::LegDataMap<std::shared_ptr<Frame>> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getFeet()
    {
        auto feet = this->template makeLegDataMap<std::shared_ptr<Frame>>();

        for (auto leg : *(this->getLegs()))
        {
            feet[leg] = std::make_shared<Link>("");
        }

        return feet;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::forwardKinematics(const RobotBase::JointState &joint_position,
                                                                                                                RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position)
    {
        joint_position.size();
        end_effector_position.getSize();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::forwardKinematics(const RobotBase::JointState &joint_position,
                                                                                                                const RobotBase::JointState &joint_velocity,
                                                                                                                RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                                                                                                RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity)
    {
        joint_position.size();
        joint_velocity.size();
        end_effector_position.getSize();
        end_effector_velocity.getSize();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::inverseKinematics(const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
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

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::inverseKinematics(const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                                                                                                RobotBase::JointState &joint_position)
    {
        end_effector_position.getSize();
        joint_position.size();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
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

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                                                                         const RobotBase::JointState &joint_position,
                                                                                                                         Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                                                                                                         RobotBase::JointState &tau_joints)        ///output
    {
        gravity_vector.size();
        wrench_base.size();
        joint_position.getSize();
        tau_joints.getSize();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    double DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getRobotMass() const
    {
        return 0.0;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    double DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getTrunkMass() const
    {
        return 0.0;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    double DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getLegsMass() const
    {
        return 0.0;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getRobotCoM() 
    { 
        return Eigen::Vector3d().setZero(); 
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 3, 1> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOM()
    {
        return Eigen::Matrix<double, 3, 1>::Zero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 3, 1> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOM(const RobotBase::JointState &joint_state)
    {
        joint_state.size();

        return Eigen::Matrix<double, 3, 1>::Zero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getLegContribution(const RobotBase::JointState &q)
    {
        q.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getCoMFromBase(const RobotBase::JointState & q,
                                                                                                                        const Eigen::Vector3d & base_orient,
                                                                                                                        const Eigen::Vector3d & base_pos)
    {
        q.size();
        base_orient.size();
        base_pos.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Vector3d DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getBaseFromCoM(const RobotBase::JointState & q,
                                                                                                                        const Eigen::Vector3d & base_orient,
                                                                                                                        const Eigen::Vector3d & CoM)
    {
        q.size();
        base_orient.size();
        CoM.size();

        return Eigen::Vector3d::Zero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVel(const RobotBase::JointState & q,
                                                                                                                                        const RobotBase::JointState & qd)
    {
        q.size();
        qd.size();

        return Eigen::Matrix<double, 6, 1>::Zero();		
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                                                                                          const Eigen::Matrix3d & rotationMx,
                                                                                                                                          const RobotBase::JointState & q,
                                                                                                                                          const RobotBase::JointState & qd)
    {
        baseVel.size();
        rotationMx.size();
        q.size();
        qd.size();
        
        return Eigen::Matrix<double, 6, 1>::Zero();	
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                                                                                          const Eigen::Matrix3d & rotationMx,
                                                                                                                                          const RobotBase::JointState & q)
    {
        baseVel.size();
        rotationMx.size();
        q.size();

        return Eigen::Matrix<double, 6, 1>::Zero();	
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    Eigen::Matrix<double, 6, 1> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                                                                                          const Eigen::Matrix3d &rotationMx,
                                                                                                                                          const Eigen::Vector3d offset_com)
    {	
        baseVel.size();
        rotationMx.size();
        offset_com.size();

        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::setInvKinTimePeriod(const double& period)
    {
        std::cout << period << std::endl;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::setTrunkCom(const Eigen::Vector3d &trunk_com) 
    {
        trunk_com.size();
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    void DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot::setTrunkMass(const double& trunk_mass)
    { 
        std::cout << trunk_mass << std::endl; 
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg::DummyLeg(const std::string &leg_name,
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

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg::~DummyLeg(){}

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg::jointToChildName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string child_name {joints_map_.find(joint_name)->second.second}; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
        return child_name;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg::jointToParentName(const std::shared_ptr<Joint> joint) const
    {
        const std::string joint_name {joint->getName()};
        const std::string parent_name {joints_map_.find(joint_name)->second.first};
        return parent_name;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg::linkToChildName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string child_name {links_map_.find(link_name)->second.second};
        return child_name;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    const std::string DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg::linkToParentName(const std::shared_ptr<Link> link) const
    {
        const std::string link_name {link->getName()};
        const std::string parent_name {links_map_.find(link_name)->second.first};
        return parent_name;
    }

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobotCreator(){}

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::~DummyRobotCreator(){}

    template <unsigned int NLEGS, unsigned int NARMS, int NJOINTS, int NLINKS, int NJOINTSLEG, int NLINKSLEG>
    std::shared_ptr<RobotBase> DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::createDummyRobot(const std::array<std::string, (1 + NLEGS + NARMS + NJOINTS + NLINKS + 1)> &components_names)
    {
        const std::string name{components_names.front()};

        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
         
        const auto trunk = std::make_shared<Trunk>(components_names.back(), trunk_dyn_params);

        std::array<std::shared_ptr<LimbBase>, NLEGS> legs({});
        std::array<std::shared_ptr<LimbBase>, NARMS> arms({});
        std::array<std::array<std::shared_ptr<Joint>, NJOINTSLEG>, NLEGS> legs_joints({});
        std::array<std::array<std::shared_ptr<Link>, NLINKSLEG>, NLEGS> legs_links({});

        for(unsigned int i{0}; i<NLEGS; i++)
        {
            for(unsigned int j{0}; j<NJOINTSLEG; j++)
            {
                legs_joints.at(i).at(j) = std::make_shared<Joint>(components_names.at((i*NJOINTSLEG)+(j+1+NLEGS+NARMS)));
            }

            for(unsigned int j{0}; j<NLINKSLEG; j++)
            {
                legs_links.at(i).at(j) = std::make_shared<Link>(components_names.at((i*NJOINTSLEG)+(j+1+NLEGS+NARMS+NJOINTS)));
            }

            legs.at(i) = std::make_shared<DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyLeg>(components_names.at(i+1), components_names.back(), legs_joints.at(i), legs_links.at(i));
        }

        return std::make_shared<DummyRobotCreator<NLEGS, NARMS, NJOINTS, NLINKS, NJOINTSLEG, NLINKSLEG>::DummyRobot>(name, trunk, legs, arms);
    }
} // namespace robotlib