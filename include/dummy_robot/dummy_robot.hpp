/*!
 * @file dummy_robot.hpp
 *
 * @brief Dummy robot class definition and functions prototypes. Used to create generic types of robots for tests
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_DUMMY_ROBOT_HPP_
#define _ROBOTLIB_DUMMY_ROBOT_HPP_

#include "robot.hpp"
#include "leg.hpp"
#include "arm.hpp"
#include "dyn_params.hpp"
#include <iostream>

namespace robotlib
{
    /*!
     * @brief DummyRobotCreator class.
     * @details
     * This class allows to create dummy robots having different morphologies with the only purpose of testing Robotlib structures.
     */
    template <unsigned int NJOINTS, unsigned int NLINKS, unsigned int NLEGS, unsigned int NJOINTSLEG, unsigned int NLINKSLEG, unsigned int NARMS = 0, unsigned int NJOINTSARM = 0, unsigned int NLINKSARM = 0>
    class DummyRobotCreator
    {
    public:
        /*!
        * @brief DummyRobot class.
        * @details
        * This class allows to create dummy robots having a specific morphology with the only purpose of testing Robotlib structures.
        */
        class DummyRobot : public Robot<NJOINTS, NLINKS, NLEGS, NARMS>
        {
        public:
            /*!
            * @brief Constructor.
            * @param[in] name name of the robot.
            * @param[in] trunk shared pointer pointing to the trunk object.
            * @param[in] legs shared pointer pointing to the robot's legs.
            * @param[in] arms shared pointer pointing to the robot's arms.
            */
            DummyRobot(const std::string &name,
                       const std::shared_ptr<Trunk> trunk,
                       const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
                       const std::array<std::shared_ptr<LimbBase>, NARMS> arms);

            /*!
            * @brief Destructor.
            */
            virtual ~DummyRobot();

            /*!
            * @brief Get position of the destination frame expressed in the origin one.
            * @param[in] q angles of the joints.
            * @param[in] origin origin frame.
            * @param[in] destination destination frame.
            * @return destination frame position expressed in origin one.
            */
            virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) override;

            /*!
            * @brief Get orientation of the destination frame expressed in the origin one.
            * @param[in] q angles of the joints.
            * @param[in] origin origin frame.
            * @param[in] destination destination frame.
            * @return destination frame orientation expressed in origin one.
            */
            virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                        const std::shared_ptr<Frame> origin,
                                                        const std::shared_ptr<Frame> destination) override;

            /*!
            * @brief Get pose of the destination frame expressed in the origin one.
            * @param[in] q angles of the joints.
            * @param[in] origin origin frame.
            * @param[in] destination destination frame.
            * @return destination frame pose expressed in origin one.
            */
            virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                                const std::shared_ptr<Frame> origin,
                                                const std::shared_ptr<Frame> destination) override;

            /*!
            * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
            * @param[in] q angles of the joints.
            * @param[in] foot foot frame.
            * @return foot position expressed in trunk frame.
            */
            virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                    const std::shared_ptr<Frame> foot) override;

            /*!
            * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
            * @details
            * This function gets the foot corresponding to the leg in input and then it computes the foot position.
            * @param[in] q angles of the joints.
            * @param[in] leg leg corresponding to the foot.
            * @return foot position expressed in trunk frame.
            */
            virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                        const std::shared_ptr<LimbBase> leg) override;

            /*!
            * @brief Get foot orientation expressed in trunk frame.
            * @param[in] q angles of the joints.
            * @param[in] foot foot frame.
            * @return foot orientation expressed in trunk frame.
            */
            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                    const std::shared_ptr<Frame> foot) override;

            /*!
            * @brief Get foot orientation with respect to the trunk frame, expressed in trunk frame.
            * @details
            * This function gets the foot corresponding to the leg in input and then it computes the foot orientation.
            * @param[in] q angles of the joints.
            * @param[in] leg leg corresponding to the foot.
            * @return foot orientation expressed in trunk frame.
            */
            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                    const std::shared_ptr<LimbBase> leg) override;

            /*!
            * @brief Get foot pose expressed in trunk frame.
            * @param[in] q angles of the joints.
            * @param[in] foot foot frame.
            * @return foot pose expressed in trunk frame.
            */
            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const std::shared_ptr<Frame> foot) override;

            /*!
            * @brief Get foot pose with respect to the trunk frame, expressed in trunk frame.
            * @details
            * This function gets the foot corresponding to the leg in input and then it computes the foot pose.
            * @param[in] q angles of the joints.
            * @param[in] leg leg corresponding to the foot.
            * @return foot pose expressed in trunk frame.
            */
            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const std::shared_ptr<LimbBase> leg) override;

            /*!
            * @brief Update the linear part of the jacobians in input.
            * @details
            * Each jacobian is associated to a leg. So it maps all the velocities of the leg's joints to foot velocity.
            * @param[in] joint_position angles of the joints.
            * @param[out] robot_jacobian a LegDataMap object, associating a jacobian to each leg.
            */
            virtual void updateLinearJacobian(const JointState &joint_position,
                                            LegDataMap<Jacobian> &robot_jacobian) override;

            /*!
            * @brief Get total robot mass.
            * @return total robot mass.
            */
            virtual double getRobotMass() const override;

            /*!
            * @brief Get trunk mass.
            * @return trunk mass.
            */
            virtual double getTrunkMass() const override;

            /*!
            * @brief Get total legs' mass.
            * @return total legs' mass.
            */
            virtual double getLegsMass() const override;

            /*!
            * @brief Get the CoM of the trunk.
            * @return trunk's CoM.
            */
            virtual Eigen::Matrix<double, 3, 1> getTrunkCOM() const override;

            /*!
            * @brief Compute whole body CoM in base frame.
            * @param[in] joint_position angles of the joints.
            * @return whole body CoM in base frame.
            */
            virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_position) override;

            /*!
            * @brief Compute CoM legs contribution in base frame.
            * @param[in] q angles of the joints.
            * @return CoM legs contribution in base frame.
            */
            virtual Eigen::Vector3d getLegContribution(const JointState &q) override;

            /*!
            * @brief Compute robot CoM position in world frame, from base pose in world frame.
            * @param[in] q angles of the joints.
            * @param[in] base_orient base orientation in world frame.
            * @param[in] base_pos base position in world frame.
            * @return CoM position in world frame.
            */
            virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                                const Eigen::Vector3d &base_orient,
                                                const Eigen::Vector3d &base_pos) override;

            /*!
            * @brief Compute robot base position in world frame, from CoM position in world frame.
            * @param[in] q angles of the joints.
            * @param[in] base_orient base orientation in world frame.
            * @param[in] com robot CoM postion in world frame.
            * @return base position in world frame.
            */
            virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                                const Eigen::Vector3d &base_orient,
                                                const Eigen::Vector3d &com) override;

            /*!
            * @brief Compute whole body CoM velocity in world frame.
            * @param[in] baseVel base velocity in base frame.
            * @param[in] R rotation matrix of base frame expressed in world frame.
            * @param[in] q angles of the joints.
            * @return CoM velocity in world frame.
            */
            virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                    const Eigen::Matrix3d &R,
                                                                    const JointState &q) override;

            /*!
            * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
            * @param[in] baseVel base velocity in base frame.
            * @param[in] R rotation matrix of base frame expressed in world frame.
            * @param[in] offset_com CoM offset in base frame.
            * @return CoM velocity in world frame.
            */
            virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                    const Eigen::Matrix3d &R,
                                                                    const Eigen::Vector3d offset_com) override;

            // ** SET FUNCTIONS **

            /*!
            * @brief Set trunk's CoM.
            * @param[in] trunk_com CoM of trunk to be set.
            */
            virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) override;

            /*!
            * @brief Set trunk's mass.
            * @param[in] trunk_mass mass of trunk to be set.
            */
            virtual void setTrunkMass(const double& trunk_mass) override;

            /*!
            * @brief Set inverse kinematics time period.
            * @details
            * This time period is the controller's loop time period. The time period needs to be set before calling the inverse kinematics.
            * @param[in] period period of the controller.
            */
            virtual void setInvKinTimePeriod(const double& period) override;

        private:
            //! Trunk.
            std::shared_ptr<Trunk> trunk_;
        };

        /*!
        * @brief DummyLeg class.
        * @details
        * This class allows to create a dummy leg having a specific morphology with the only purpose of testing Robotlib structures.
        * @tparam NJOINTSLEG number of joints of the leg.
        * @tparam NLINKSLEG number of links of the leg.
        */
        class DummyLeg : public Leg<NJOINTSLEG, NLINKSLEG>
        {
        public:
            /*!
            * @brief Constructor.
            * @param[in] leg_name name of the leg.
            * @param[in] trunk_name name of the robot trunk.
            * @param[in] leg_joints array of shared pointers pointing to leg's joints.
            * @param[in] leg_links array of shared pointers pointing to leg's links.
            */
            DummyLeg(const std::string &leg_name,
                    const std::string &trunk_name,
                    const std::array<std::shared_ptr<Joint>, NJOINTSLEG> &leg_joints,
                    const std::array<std::shared_ptr<Link>, NLINKSLEG> &leg_links);

            /*!
            * @brief Destructor.
            */
            virtual ~DummyLeg();

            /*!
            * @brief Get the name of the joint's parent of the leg.
            * @param[in] joint Joint object for which you get the name of the parent.
            * @return name of the joint's parent. 
            */
            virtual std::string jointToParentName(const std::shared_ptr<Joint> joint) const override;

            /*!
            * @brief Get the name of the joint's child of the leg.
            * @param[in] joint Joint object for which you get the name of the child.
            * @return name of the joint's child.
            */
            virtual std::string jointToChildName(const std::shared_ptr<Joint> joint) const override;

            /*!
            * @brief Get the name of link's parent of the leg.
            * @param[in] link Link object for which you get the name of the parent.
            * @return name of the link's parent.
            */
            virtual std::string linkToParentName(const std::shared_ptr<Link> link) const override;

            /*!
            * @brief Get the name of the link's child of the leg.
            * @param[in] link Link object for which you get the name of the child.
            * @return name of the link's child.
            */
            virtual std::string linkToChildName(const std::shared_ptr<Link> link) const override;

      private:
            //! Variable mapping each joint name to its parent and child names.
            std::map<std::string, std::pair<std::string, std::string>> joints_map_{};

            //! Variable mapping each link name to its parent and child names.
            std::map<std::string, std::pair<std::string, std::string>> links_map_{};
        };

        /*!
        * @brief DummyArm class.
        * @details
        * This class allows to create a dummy arm having a specific morphology with the only purpose of testing Robotlib structures.
        */
        class DummyArm : public Arm<NJOINTSARM, NLINKSARM>
        {
        public:
            /*!
            * @brief Constructor.
            * @param[in] arm_name name of the arm.
            * @param[in] trunk_name name of the robot trunk.
            * @param[in] arm_joints array of shared pointers pointing to arm's joints.
            * @param[in] arm_links array of shared pointers pointing to arm's links.
            */
            DummyArm(const std::string &arm_name,
                    const std::string &trunk_name,
                    const std::array<std::shared_ptr<Joint>, NJOINTSARM> &arm_joints,
                    const std::array<std::shared_ptr<Link>, NLINKSARM> &arm_links);

            /*!
            * @brief Destructor.
            */
            virtual ~DummyArm();

            /*!
            * @brief Get the name of the joint's parent of the leg.
            * @param[in] joint Joint object for which you get the name of the parent.
            * @return name of the joint's parent. 
            */
            virtual std::string jointToParentName(const std::shared_ptr<Joint> joint) const override;

            /*!
            * @brief Get the name of the joint's child of the leg.
            * @param[in] joint Joint object for which you get the name of the child.
            * @return name of the joint's child.
            */
            virtual std::string jointToChildName(const std::shared_ptr<Joint> joint) const override;

            /*!
            * @brief Get the name of link's parent of the leg.
            * @param[in] link Link object for which you get the name of the parent.
            * @return name of the link's parent.
            */
            virtual std::string linkToParentName(const std::shared_ptr<Link> link) const override;

            /*!
            * @brief Get the name of the link's child of the leg.
            * @param[in] link Link object for which you get the name of the child.
            * @return name of the link's child.
            */
            virtual std::string linkToChildName(const std::shared_ptr<Link> link) const override;

      private:
            //! Variable mapping each joint name to its parent and child names.
            std::map<std::string, std::pair<std::string, std::string>> joints_map_{};

            //! Variable mapping each link name to its parent and child names.
            std::map<std::string, std::pair<std::string, std::string>> links_map_{};
        };

        /*!
        * @brief Constructor.
        */
        DummyRobotCreator();

        /*!
        * @brief Destructor.
        */
        virtual ~DummyRobotCreator();

        /*!
        * @brief Function to create a dummy robot.
        * @details
        * The names of the items defining the robot structure follow this order:
        * 
        * Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names]
        * @param[in] components_names names of the items defining the robot.
        * @return shared pointer pointing to the RobotBase object.
        */
        std::shared_ptr<RobotBase> createDummyRobot(const std::array<std::string, (2 + NJOINTS + NLINKS + NLEGS + NARMS)> &components_names);
    };
} // namespace robotlib

#include "dummy_robot.tpp"

#endif // _ROBOTLIB_DUMMY_ROBOT_HPP_