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
                   const DynParams& dynamic_parameters,
                   Container<LimbBase, NLIMBS>& limbs);

        /*!
        * @brief Destructor.
        */
        virtual ~DummyRobot() = default;


        /*!
          * @brief Forward kinematics.
          * @details
          * It computes the position of each end effector expressed in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[out] end_effector_position position of each end effector in base frame.
          */
        virtual void forwardKinematics(const JointState &joint_position,
                                       LimbDataMap<Eigen::Vector3d> &end_effector_position) override;

        /*!
         * @brief Forward kinematics.
         * @details
         * It computes the position and velocity of each end effector expressed in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[out] end_effector_position position of each end effector in base frame.
         * @param[out] end_effector_velocity velocity of each end effector in base frame.
         */
        virtual void forwardKinematics(const JointState &joint_position,
                                       const JointState &joint_velocity,
                                       LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                       LimbDataMap<Eigen::Vector3d> &end_effector_velocity) override;


        /*!
        * @brief Inverse kinematics. Does not consider the floating base joint. It computes the joint angles from the desired frame position expressed in base frame. Redundancy is not handled yet.
        * @param[in] frame_name name of the frame.
        * @param[in] q_init_guess initial guess for the joint angles.
        * @param[in] position_des desired position of the frame expressed in base frame.
        * @param[out] q_des desired joint angles.
        */
        virtual void fixedBaseInverseKinematics(const std::string &frame_name,
                                                const robotlib::JointState &q_init_guess,
                                                const Eigen::Vector3d &position_des,
                                                robotlib::JointState &q_des) override;
        /*!
        * @brief Inverse kinematics considering all the legs. Does not consider the floating base joint. It computes the joint angles from the desired frame position expressed in base frame. Redundancy is not handled yet.
        * @param[in] q_init_guess initial guess for the joint angles.
        * @param[in] positions_des desired positions of all the legs expressed in base frame.
        * @param[out] q_des desired joint angles.
        */
        virtual void fixedBaseInverseKinematics(const robotlib::JointState &q_init_guess,
                                                const robotlib::LimbDataMap<Eigen::Vector3d> &positions_des,
                                                robotlib::JointState &q_des) override;
        /*!
        * @brief Inverse differential kinematics. Does not consider the floating base joint. It computes the joint velocities from the desired frame linear velocity expressed in base frame. Redundancy is not handled yet.
        * @param[in] frame_name name of the frame
        * @param[in] q joint angles
        * @param[in] velocity_des desired frame linear velocity expressed in base frame
        * @param[out] qd_des desired joint velocities
        */
        virtual void fixedBaseInverseDiffKinematics(const std::string &frame_name,
                                                            const robotlib::JointState &q,
                                                            const Eigen::Vector3d &velocity_des,
                                                            robotlib::JointState &qd_des) override;
        /*!
        * @brief Inverse differential kinematics for all the legs. Does not consider the floating base joint. It computes the joint velocities from the desired frame linear velocity expressed in base frame. Redundancy is not handled yet.
        * @param[in] q joint angles
        * @param[in] velocity_des desired linear velocities of all the legs expressed in base frame
        * @param[out] qd_des desired joint velocities
        */
        virtual void fixedBaseInverseDiffKinematics(const robotlib::JointState &q,
                                                    const robotlib::LimbDataMap<Eigen::Vector3d> &velocities_des,
                                                    robotlib::JointState &qd_des) override;

        /*!
         * @brief Inverse dynamics.
         * @details
         * It computes the torque at each joint. Before calling this function, you need to call forwardKinematics first
         * Use cases:
         * - robot gravity compensation: robot_velocity = 0, robot_acceleration = 0, joint_velocity = 0, joint_acceleration = 0, f_contact = forces to substain robot weight.
         * - leg gravity compensation: robot_velocity = 0, robot_acceleration = 0, joint_velocity = 0, joint_acceleration = 0, f_contact = 0.
         * - realize desired contact forces and robot accelerations: 
         *    robot_velocity = actual robot velocity
         *    robot_acceleration = desired robot acceleration
         *    joint_position = actual joint position
         *    joint_velocity = actual joint velocity
         *    joint_acceleration = desired joint acceleration
         *    f_contact = desired contact forces. 
         * @param[in] robot_pose pose of the robot base in base frame.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] joint_acceleration acceleration of each joint.
         * @param[in] f_contact map defined as follows: [contact_frame, contact_force], where contact_force is expressed in base_frame.
         * @param[out] tau_joints torque of each joint.
         */
        virtual void inverseDynamics(
                                const Eigen::Matrix<double, 7, 1> &robot_pose,    // robot base
                                const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                const robotlib::JointState &joint_acceleration,
                                const robotlib::eigen::aligned_map<std::string, Eigen::Vector3d> &f_contact,
                                robotlib::JointState &tau_joints) override;

        /*!
        * @brief Compute gravity terms.
        * @details
        * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
        * @param[in] robot_pose pose of the robot base in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] g_base gravity term related to the base.
        * @param[out] g_joints gravity term related to the joints.
        */
        virtual void computeGravityTerm(    const Eigen::Matrix<double, 7, 1> &robot_pose,
                                            const robotlib::JointState &joint_position,
                                            Eigen::Matrix<double, 6, 1> &g_base,
                                            robotlib::JointState &g_joints) override;
        /*!
        * @brief Compute gravity terms, setting only the one related to the joints.
        * @details
        * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
        * @param[in] robot_pose pose of the robot base in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] g_joints gravity term related to the joints.
        */
        virtual void computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                          const robotlib::JointState &joint_position,
                                          robotlib::JointState &g_joints) override;
        /*!
          * @brief Inverse dynamics to compute the Centrifugal, Coriolis and Gravity terms.
          * @param[in] robot_pose pose of the robot base in world frame.
          * @param[in] robot_velocity velocity of the robot base in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[in] joint_velocity velocity of each joint.
          * @param[out] nle_base non linear effects acting on the base.
          * @param[out] nle_joints non linear effects acting on the joints.
          */
        virtual void computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        Eigen::Matrix<double, 6, 1> &nle_base,
                                        robotlib::JointState &nle_joints) override;

        /*!
        * @brief Inverse dynamics to compute the Centrifugal, Coriolis and Gravity terms.
        * @details
        * The robot velocity is to zero by default.
        * @param[in] robot_pose pose of the robot base in world frame.
        * @param[in] robot_velocity velocity of the robot base in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[in] joint_velocity velocity of each joint.
        * @param[out] nle_joints non linear effects acting on the joints.
        */
        virtual void computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        robotlib::JointState &nle_joints) override;

        /*!
         * @brief Get position of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Vector3d computeFramePosition(const JointState &q, const Frame& origin, const Frame& destination) override;

        /*!
         * @brief Get orientation of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Matrix3d computeFrameOrientation(const JointState& q, const Frame& origin, const Frame& destination) override;

        /*!
         * @brief Get pose of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Matrix4d computeFramePose(const JointState& q, const Frame& origin, const Frame& destination) override;

        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame, related to the limbs only (so considering the actuated joints). The order is linear_jacobian, angular_jacobian. For a complete jacobian, see computeWholeBodyJacobian.
        * @param[in] q angles of the joints.
        * @param[in] frame frame used to compute the jacobian.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeLimbsJacobian(const JointState& q,
                                          const Frame& frame,
                                          Eigen::MatrixXd& jacobian) override;

        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame. The order is linear_jacobian, angular_jacobian. For a jacobian considering only the joints, see getLimbsJacobian.
        * @param[in] robot_pose pose of the robot base in world frame.
        * @param[in] q angles of the joints.
        * @param[in] frame frame used to compute the jacobian.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeWholeBodyJacobian(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &q,
                                        const Frame& frame,
                                        Eigen::MatrixXd &jacobian) override;

        /*!
        * @brief Get total robot mass.
        * @return total robot mass.
        */
        virtual double getRobotMass() const override;

        /*!
         * @brief Get link mass.
         * @param[in] link the link
         * @return link mass.
         */
        virtual double getLinkMass(const Link& link) const override;

        /*!
         * @brief Get link inertia about the CoM.
         * @param[in] link the link
         * @return link inertia.
        */
        virtual Eigen::Matrix3d getLinkInertia(const Link& link) const override;

        /*!
         * @brief Get link CoM in the joint frame(see https://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file).
         * @param[in] link the link 
         * @return link CoM.
        */
        virtual Eigen::Vector3d getLinkCoM(const Link& link) const override;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d computeWholeBodyCoM(const JointState& q) override;

        /*!
         *@brief Get the IMU pose in base frame.
         *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
         *@param[in] base_link_name name of the base link.
         *@return IMU pose in base frame.
         */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string& imu_link_name="trunk_imu",
                                                 const std::string& base_link_name="base_link") const override;
    };

} // namespace robotlib

#include "dummy_robot.tpp"

#endif // _ROBOTLIB_DUMMY_ROBOT_HPP_