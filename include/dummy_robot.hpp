/**
 * @file dummy_robot.hpp
 *
 * @brief Dummy robot class definition and functions prototypes. Used to create generic types of robots for tests
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_DUMMY_ROBOT_HPP_
#define _ROBOTLIB_DUMMY_ROBOT_HPP_

#include "robot.hpp"
#include "leg.hpp"
#include "dyn_params.hpp"
#include <iostream>

namespace robotlib
{
    template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS, int NJOINTSLEG, int NLINKSLEG>
    class DummyRobotCreator
    {
    public:
        class DummyRobot : public Robot<NJOINTS, NLINKS, NLEGS, NARMS>
        {
        public:
            DummyRobot(const std::shared_ptr<Trunk> trunk,
              const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
              const std::array<std::shared_ptr<LimbBase>, NARMS> arms);
            ~DummyRobot();

            virtual Eigen::Vector3d getFramePosition(const RobotBase::JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) override;

            virtual Eigen::Matrix3d getFrameOrientation(const RobotBase::JointState &q,
                                                        const std::shared_ptr<Frame> origin,
                                                        const std::shared_ptr<Frame> destination) override;

            virtual Eigen::Matrix4d getFramePose(const RobotBase::JointState &q,
                                                const std::shared_ptr<Frame> origin,
                                                const std::shared_ptr<Frame> destination) override;

            virtual Eigen::Vector3d getFootPosition(const RobotBase::JointState &q,
                                                    const std::shared_ptr<Frame> foot) override;

            virtual Eigen::Matrix3d getFootOrientation(const RobotBase::JointState &q,
                                                      const std::shared_ptr<Frame> foot) override;

            virtual Eigen::Matrix4d getFootPose(const RobotBase::JointState &q,
                                                const std::shared_ptr<Frame> foot) override;

            virtual void getFootPosition(const RobotBase::JointState &q,
                                        const std::shared_ptr<LimbBase> leg,
                                        Eigen::Vector3d &footPos) override;

            virtual Eigen::Matrix3d getFootOrientation(const RobotBase::JointState &q,
                                                      const std::shared_ptr<LimbBase> leg) override;

            virtual Eigen::Matrix4d getFootPose(const RobotBase::JointState &q,
                                                const std::shared_ptr<LimbBase> leg) override;

            virtual void getFootJacobian(const RobotBase::JointState &q,
                                        const std::shared_ptr<LimbBase> leg,
                                        RobotBase::Jacobian &footJac) override;

            virtual void updateLinearJacobian(const RobotBase::JointState &joints_positions,
                                              RobotBase::LegDataMap<RobotBase::Jacobian> &robot_jacobian) override;

            virtual RobotBase::LegDataMap<std::shared_ptr<Frame>> getFeet() override;

            virtual void forwardKinematics(const RobotBase::JointState &joint_position,
                                          RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position) override;

            virtual void forwardKinematics(const RobotBase::JointState &joint_position,
                                          const RobotBase::JointState &joint_velocity,
                                          RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                          RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity) override;

            virtual void inverseKinematics(const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                          const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                          const RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                          RobotBase::JointState &joint_position,
                                          RobotBase::JointState &joint_velocity,
                                          RobotBase::JointState &joint_acceleration) override;

            virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                        const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                        const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                        const RobotBase::JointState &joint_position,
                                        const RobotBase::JointState &joint_velocity,
                                        const RobotBase::JointState &joint_acceleration,
                                        Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                        RobotBase::JointState &tau_joints) override;		   ///output

            virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                    const RobotBase::JointState &joint_position,
                                                    Eigen::Matrix<double, 6, 1> &wrench_base,     ///output
                                                    RobotBase::JointState &tau_joints) override;             ///output

            virtual double getRobotMass() const override;

            virtual double getTrunkMass() const override;

            virtual double getLegsMass() const override;

            virtual Eigen::Vector3d getRobotCoM() override;

            virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM() override;

            virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const RobotBase::JointState &joint_state) override;

            virtual Eigen::Vector3d getLegContribution(const RobotBase::JointState &q) override;

            virtual Eigen::Vector3d getCoMFromBase(const RobotBase::JointState & q,
                                                  const Eigen::Vector3d & base_orient,
                                                  const Eigen::Vector3d & base_pos) override;

            virtual Eigen::Vector3d getBaseFromCoM(const RobotBase::JointState & q,
                                                  const Eigen::Vector3d & base_orient,
                                                  const Eigen::Vector3d & CoM) override;

            virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const RobotBase::JointState & q,
                                                                  const RobotBase::JointState & qd) override;

            virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                    const Eigen::Matrix3d & rotationMx,
                                                                    const RobotBase::JointState & q,
                                                                    const RobotBase::JointState & qd) override;

            virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                    const Eigen::Matrix3d & rotationMx,
                                                                    const RobotBase::JointState & q) override;

            virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                    const Eigen::Matrix3d &rotationMx,
                                                                    const Eigen::Vector3d offset_com) override;

            virtual void setInvKinTimePeriod(const double& period) override;

            virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) override;

            virtual void setTrunkMass(const double& trunk_mass) override;
        };

        class DummyLeg : public Leg<NJOINTSLEG, NLINKSLEG>
        {
        public:
        DummyLeg(const std::string &name,
            const std::array<std::shared_ptr<Joint>, NJOINTSLEG> &joints,
            const std::array<std::shared_ptr<Link>, NLINKSLEG> &links);
            ~DummyLeg();

        virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) const override;

        virtual const std::string jointToParentName(const std::shared_ptr<Joint> joint) const override;

        virtual const std::string linkToChildName(const std::shared_ptr<Link> link) const override;

        virtual const std::string linkToParentName(const std::shared_ptr<Link> link) const override;

      private:
        const std::map<std::string, std::pair<std::string, std::string>> jointMap;
        const std::map<std::string, std::pair<std::string, std::string>> linkMap;
        };

		    DummyRobotCreator();
        ~DummyRobotCreator();

        std::shared_ptr<DummyRobotCreator::DummyLeg> makeLeg(const std::string &legName);
        std::shared_ptr<RobotBase> createDummyRobot();
    };
} // namespace robotlib

#include "dummy_robot.tpp"

#endif // _ROBOTLIB_DUMMY_ROBOT_HPP_