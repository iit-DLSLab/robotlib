/*!
 * @file dummy_robot.hpp
 *
 * @brief Dummy robot class definition and functions prototypes. Used to create generic types of robots for tests
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

#ifndef _ROBOTLIB_DUMMY_ROBOT_HPP_
#define _ROBOTLIB_DUMMY_ROBOT_HPP_

#include "robot.hpp"
#include "dummy_leg.hpp"
#include "dummy_arm.hpp"

#include <iostream>

namespace robotlib
{
    /*!
    * @brief DummyRobot class.
    * @details
    * This class allows to create dummy robots having a specific morphology with the only purpose of testing Robotlib structures.
    */
    template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    class DummyRobot : public Robot<NLIMBS, NLINKS, NJOINTS>
    {
    public:
        /*!
        * @brief Constructor.
        * @param[in] name name of the robot.
        * @param[in] trunk shared pointer pointing to the trunk object.
        * @param[in] legs shared pointer pointing to the robot's legs.
        * @param[in] arms shared pointer pointing to the robot's arms.
        */
        DummyRobot(const std::string& name,
                   Trunk& trunk,
                   std::array<LimbBase, NLIMBS> limbs);

        /*!
        * @brief Destructor.
        */
        virtual ~DummyRobot() = default;

        /*!
        * @brief Forward kinematics.
        * @details
        * It computes the position of each end effector (foot) expressed in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] end_effector_position position of each end effector (foot) in base frame.
        */
        virtual void forwardKinematics(const robotlib::JointState& joint_position,
                                       robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position) const override;

        /*!
        * @brief Forward kinematics.
        * @details
        * It computes the position and velocity of each end effector (foot) expressed in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[in] joint_velocity velocity of each joint.
        * @param[out] end_effector_position position of each end effector (foot) in base frame.
        * @param[out] end_effector_velocity velocity of each end effector (foot) in base frame.
        */
        virtual void forwardKinematics(const robotlib::JointState& joint_position,
                                       const robotlib::JointState& joint_velocity,
                                       robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position,
                                       robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_velocity) const override;

        /*!
        * @brief Inverse kinematics.
        * @details
        * It computes the angle, velocity and acceleration of each joint from the position, velocity and acceleration of each end effector expressed in base frame.
        * @param[in] end_effector_position position of each end effector (foot) in base frame.
        * @param[in] end_effector_velocity velocity of each end effector (foot) in base frame.
        * @param[in] end_effector_acceleration acceleration of each end effector (foot) in base frame.
        * @param[out] joint_position angle of each joint.
        * @param[out] joint_velocity velocity of each joint.
        * @param[out] joint_acceleration acceleration of each joint.
        */
        virtual void inverseKinematics(const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position,
                                       const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_velocity,
                                       const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_acceleration,
                                       robotlib::JointState& joint_position,
                                       robotlib::JointState& joint_velocity,
                                       robotlib::JointState& joint_acceleration) const override;

        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle and velocity of each joint from the position and velocity of each end effector expressed in base frame.
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[in] end_effector_velocity velocity of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         * @param[out] joint_velocity velocity of each joint.
         */
        virtual void inverseKinematics(const LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                       const LimbDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       JointState &joint_position,
                                       JointState &joint_velocity) const override;

        /*!
        * @brief Inverse kinematics.
        * @details
        * It computes the angle of each joint from the position of each end effector expressed in base frame.
        * @param[in] end_effector_position position of each end effector (foot) in base frame.
        * @param[out] joint_position angle of each joint.
        */
        virtual void inverseKinematics(const robotlib::LimbDataMap<Eigen::Vector3d>& end_effector_position,
                                       robotlib::JointState& joint_position) const override;

        /*!
        * @brief Inverse dynamics.
        * @details
        * It computes the torque of each joint and the wrench at the base.
        * @param[in] robot_velocity velocity of the robot base in base frame.
        * @param[in] robot_acceleration  acceleration of the robot base in base frame.
        * @param[in] gravity_vector gravity vector in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[in] joint_velocity velocity of each joint.
        * @param[in] joint_acceleration acceleration of each joint.
        * @param[out] wrench_base wrench applied to the base.
        * @param[out] tau_joints torque of each joint.
        */
        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const robotlib::JointState &joint_position,
                                     const robotlib::JointState &joint_velocity,
                                     const robotlib::JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base,
                                     robotlib::JointState &tau_joints) const override;

        /*!
         * @brief Inverse dynamics to compute the Centrifugal, Coriolis and Gravity terms.
         * @details
         * The robot velocity and acceleration are set to zero by default.
         * @param[out] tau_joints torque of each joint.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         */
        virtual void inverseDynamicsHTerm(  JointState &tau_joints,
                                            const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                            const JointState &joint_position,
                                            const JointState &joint_velocity,
                                            const Eigen::Matrix<double, 6, 1> &robot_velocity = Eigen::Matrix<double, 6, 1>::Zero(),
                                            const Eigen::Matrix<double, 6, 1> &robot_acceleration = Eigen::Matrix<double, 6, 1>::Zero()) const override;
        
        /*!
        * @brief Compute gravity terms.
        * @details
        * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
        * @param[in] gravity_vector gravity vector in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] wrench_base wrench applied to the base.
        * @param[out] tau_joints torque of each joint.
        */
        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                const robotlib::JointState &joint_position,
                                                Eigen::Matrix<double, 6, 1> &wrench_base,
                                                robotlib::JointState &tau_joints) const override;

        /*!
         * @brief Compute gravity terms and return only the wrench applied to the base to compensate for gravity.
         * @details
         * Instead of using the full version of the computeGravityCompensation function, you can use this function to only get the desired wrench, without taking care of the gravity compensation torques.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @return wrench applied to the base.
         */
        virtual Eigen::Matrix<double, 6, 1> computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                                             const JointState &joint_position) const override;

        /*!
         * @brief Compute gravity terms and return only the joint torques compensating for gravity.
         * @details
         * Instead of using the full version of the computeGravityCompensation function, you can use this function to only get the desired joint torques that compensate for gravity, without taking care of the wrench applied to the base. Notice that this function has the joint state as output parameter, because returning a JointState object leads to dynamic memory allocation.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[out] tau_joints torque of each joint.
         */
        virtual void computeTorquesGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position,
                                                      JointState &tau_joints) const override;
    


        // ** GET FUNCTIONS **

        /*!
        * @brief Get position of the destination frame expressed in the origin one.
        * @param[in] q angles of the joints.
        * @param[in] origin origin frame.
        * @param[in] destination destination frame.
        * @return destination frame position expressed in origin one.
        */
        virtual Eigen::Vector3d getFramePosition(const robotlib::JointState &q, const robotlib::Frame& origin, const robotlib::Frame& destination) const override;

        /*!
        * @brief Get orientation of the destination frame expressed in the origin one.
        * @param[in] q angles of the joints.
        * @param[in] origin origin frame.
        * @param[in] destination destination frame.
        * @return destination frame orientation expressed in origin one.
        */
        virtual Eigen::Matrix3d getFrameOrientation(const robotlib::JointState& q,
                                                    const robotlib::Frame& origin,
                                                    const robotlib::Frame& destination) const override;

        /*!
        * @brief Get pose of the destination frame expressed in the origin one.
        * @param[in] q angles of the joints.
        * @param[in] origin origin frame.
        * @param[in] destination destination frame.
        * @return destination frame pose expressed in origin one.
        */
        virtual Eigen::Matrix4d getFramePose(const robotlib::JointState &q,
                                                const robotlib::Frame& origin,
                                                const robotlib::Frame& destination) const override;

        /*!
        * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
        * @param[in] q angles of the joints.
        * @param[in] foot foot frame.
        * @return foot position expressed in trunk frame.
        */
        virtual Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
                                                const robotlib::Frame& foot) const override;

        /*!
        * @brief Get foot orientation expressed in trunk frame.
        * @param[in] q angles of the joints.
        * @param[in] foot foot frame.
        * @return foot orientation expressed in trunk frame.
        */
        virtual Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
                                                    const robotlib::Frame& foot) const override;

        /*!
        * @brief Get foot pose expressed in trunk frame.
        * @param[in] q angles of the joints.
        * @param[in] foot foot frame.
        * @return foot pose expressed in trunk frame.
        */
        virtual Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
                                            const robotlib::Frame& foot) const override;

        /*!
        * @brief Update the linear part of the jacobians in input.
        * @details
        * Each jacobian is associated to a leg. So it maps all the velocities of the leg's joints to foot velocity.
        * @param[in] joint_position angles of the joints.
        * @param[out] robot_jacobian a LimbDataMap object, associating a jacobian to each leg.
        */
        virtual void updateLinearJacobian(const robotlib::JointState& joint_position,
                                          robotlib::LimbDataMap<robotlib::Jacobian>& robot_jacobian) const override;

        /*!
         * @brief Update the angular part of the jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
        virtual void updateAngularJacobian(const JointState& q, LimbDataMap<Jacobian>& robot_jacobian) const override;

        /*!
         * @brief Get the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void getFootJacobian(const JointState& q, const LimbBase& leg, Jacobian &footJac) const override;

        /*!
         * @brief Update the linear part of the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void updateLinearFootJacobian(const JointState& joints_positions, const LimbBase& leg, Jacobian& footJac) const override;

        /*!
         * @brief Update the angular part of the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void updateAngularFootJacobian(const JointState& q, const LimbBase& leg, Jacobian& footJac) const override;

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
         * @brief Compute robot CoM in base frame.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d getRobotCoM() const override;

        /*!
        * @brief Compute whole body CoM in base frame.
        * @param[in] joint_position angles of the joints.
        * @return whole body CoM in base frame.
        */
        virtual Eigen::Vector3d getWholeBodyCOM(const robotlib::JointState& joint_position) const override;

        /*!
        * @brief Compute CoM legs contribution in base frame.
        * @param[in] q angles of the joints.
        * @return CoM legs contribution in base frame.
        */
        virtual Eigen::Vector3d getLegContribution(const robotlib::JointState& q) const override;

        /*!
        * @brief Compute robot CoM position in world frame, from base pose in world frame.
        * @param[in] q angles of the joints.
        * @param[in] base_orient base orientation in world frame.
        * @param[in] base_pos base position in world frame.
        * @return CoM position in world frame.
        */
        virtual Eigen::Vector3d getCoMFromBase(const robotlib::JointState& q,
                                                const Eigen::Vector3d& base_orient,
                                                const Eigen::Vector3d& base_pos) const override;

        /*!
        * @brief Compute robot base position in world frame, from CoM position in world frame.
        * @param[in] q angles of the joints.
        * @param[in] base_orient base orientation in world frame.
        * @param[in] com robot CoM postion in world frame.
        * @return base position in world frame.
        */
        virtual Eigen::Vector3d getBaseFromCoM(const robotlib::JointState& q,
                                                const Eigen::Vector3d& base_orient,
                                                const Eigen::Vector3d& com) const override;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
		 */
        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVel(const JointState& q,
                                                               const JointState& qd) const override;

        /*!
        * @brief Compute whole body CoM velocity in world frame.
        * @param[in] baseVel base velocity in base frame.
        * @param[in] R rotation matrix of base frame expressed in world frame.
        * @param[in] q angles of the joints.
        * @return CoM velocity in world frame.
        */
        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1>& baseVel,
                                                                const Eigen::Matrix3d& R,
                                                                const robotlib::JointState& q) const override;

        /*!
        * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
        * @param[in] baseVel base velocity in base frame.
        * @param[in] R rotation matrix of base frame expressed in world frame.
        * @param[in] offset_com CoM offset in base frame.
        * @return CoM velocity in world frame.
        */
        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1>& baseVel,
                                                                const Eigen::Matrix3d& R,
                                                                const Eigen::Vector3d& offset_com) const override;

        /*!
         *@brief Get the IMU pose in base frame.
         *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
         *@param[in] base_link_name name of the base link.
         *@return IMU pose in base frame.
         */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string& imu_link_name="trunk_imu",
                                                 const std::string& base_link_name="base_link") const override;

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
        virtual void setTrunkMass(const double trunk_mass) override;

        /*!
        * @brief Set inverse kinematics time period.
        * @details
        * This time period is the controller's loop time period. The time period needs to be set before calling the inverse kinematics.
        * @param[in] period period of the controller.
        */
        virtual void setInvKinTimePeriod(const double period) override;

    };

} // namespace robotlib

#include "dummy_robot.tpp"

#endif // _ROBOTLIB_DUMMY_ROBOT_HPP_