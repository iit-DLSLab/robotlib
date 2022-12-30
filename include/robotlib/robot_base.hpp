#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_

#include "trunk.hpp"
#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"
#include "dyn_params.hpp"
#include "leg_data_map.hpp"
#include "joint_state.hpp"
#include "jacobian.hpp"
#include "link_data_map.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <stdexcept>


namespace robotlib
{
    class RobotBase
    {
    public:
        RobotBase(const std::string &name);
        ~RobotBase();

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
                                                      const JointState &joint_position,
                                                      Eigen::Matrix<double, 6, 1> &wrench_base,
                                                      JointState &tau_joints) const = 0;

        /*!
         * @brief Compute gravity terms and return only the wrench applied to the base to compensate for gravity.
         * @details
         * Instead of using the full version of the computeGravityCompensation function, you can use this function to only get the desired wrench, without taking care of the gravity compensation torques.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @return wrench applied to the base.
         */
        virtual Eigen::Matrix<double, 6, 1> computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position) const = 0;
        
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
                                                      JointState &tau_joints) const = 0;

        // ** GET FUNCTIONS **
        const std::string getName() const;

        virtual int getNLEGS() const = 0;
        virtual int getNARMS() const = 0;
        virtual int getNJOINTS() const = 0;
        virtual int getNLINKS() const = 0;

        // virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) = 0;    ///TODO: required for the print inside CGaitTimerHex::run() of Ant Controller

        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;
        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

        virtual const std::shared_ptr<Link> getLink(const std::string &name) const = 0;
        virtual const std::shared_ptr<Joint> getJoint(const std::string &name) const = 0;
        virtual const std::shared_ptr<LimbBase> getLeg(const std::string &name) const = 0;

        virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

        virtual void getMinJointAngle(JointState &q_min) = 0;
        virtual void getMaxJointAngle(JointState &q_max) = 0;
        virtual void getMaxJointVelocity(JointState &qd_max) = 0;
        virtual void getMaxJointEffort(JointState &tau_max) = 0;

        virtual void getMinJointAngle(const std::shared_ptr<Joint> joint, double &q_min);
        virtual void getMaxJointAngle(const std::shared_ptr<Joint> joint, double &q_max);
        virtual void getMaxJointVelocity(const std::shared_ptr<Joint> joint, double &qd_max);
        virtual void getMaxJointEffort(const std::shared_ptr<Joint> joint, double &tau_max);
        
        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::shared_ptr<Frame> origin,
                                             const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        /*!
         * @brief Update the linear part of the jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
        virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) const = 0; //overridden by Glue

        /*!
         * @brief Update the angular part of the jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
        virtual void updateAngularJacobian(const JointState &q,
                                           LegDataMap<Jacobian> &robot_jacobian) const = 0;

        /*!
         * @brief Get the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void getFootJacobian(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Jacobian &footJac) const = 0;

        /*!
         * @brief Update the linear part of the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void updateLinearFootJacobian(const JointState &joints_positions,
                                          const std::shared_ptr<LimbBase> leg,
                                          Jacobian &footJac) const = 0;

        /*!
         * @brief Update the angular part of the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void updateAngularFootJacobian(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Jacobian &footJac) const = 0;

        // TODO: compute total mass from links and trunk masses (it could be even implemented in Robot class)
        virtual double getRobotMass() const = 0;

        virtual double getTrunkMass() const = 0;

        virtual double getLegsMass() const = 0;

        // NB: eventually make the get function void for possible NRT issue
        virtual const Eigen::Vector3d& getTrunkCOM() const = 0;

        virtual Eigen::Vector3d getRobotCoM() = 0;

        virtual Eigen::Vector3d getWholeBodyCOM() = 0;

        virtual Eigen::Vector3d getWholeBodyCOM(const JointState &joint_state) const = 0;

        virtual Eigen::Vector3d getLegContribution(const JointState &q) const = 0;

        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) = 0;

        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &CoM) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const JointState &q,
                                                               const JointState &qd) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const JointState &q,
                                                                 const JointState &qd) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const robotlib::JointState &q) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const Eigen::Vector3d offset_com) = 0;

        /*!
        *@brief Get the IMU pose in base frame.
        *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
        *@param[in] base_link_name name of the base link.
        *@return IMU pose in base frame.
        */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string imu_link_name="trunk_imu", const std::string base_link_name="base_link") const = 0;

        // ** FUNCTIONS TO MAKE NRT OBJECTS ** 

        // Create a joint state
        JointState makeJointState(const double value = 0.0) const;

        // Create a leg data map pair
        template <class Data>
        LegDataMap<Data> makeLegDataMap() const; // NRT

        // Create a leg data map pair
        template <class Data>
        LegDataMap<Data> makeLegDataMap(const Data&) const;

        // Create a link data map pair
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(); // NRT
        
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(const Data&); // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMap(); // NRT

        template <class Data>
        JointDataMap<Data> makeJointDataMap(const Data&); // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase>); // NRT
        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data&); // NRT

        // TODO
        Jacobian makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest); // NRT

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<Frame> frame); // NRT

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data = 0.0); // NRT

        LegDataMap<Jacobian> makeFeetJacobian(const double data = 0.0) const; // NRT

        // ** FORWARD KINEMATICS ** 

        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position) const = 0;

        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       const robotlib::JointState &joint_velocity,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity) const = 0;

        virtual void forwardKinematics(const JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                       const JointState &joint_velocity,
                                       const JointState &joint_acceleration,
                                       LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       LegDataMap<Eigen::Vector3d> &end_effector_acceleration) const = 0;

        virtual robotlib::LegDataMap<Eigen::Vector3d> forwardKinematics(const robotlib::JointState &) const = 0;

        // ** INVERSE KINEMATICS ** 

        virtual void inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                robotlib::JointState &joint_position) const = 0;
        
        virtual void inverseKinematics(const LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       const LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       const LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) const = 0;   

        virtual JointState inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d>& ) const = 0;

        // ** INVERSE DYNAMICS ** 

        /*!
         * @brief Inverse dynamics.
         * @details
         * It computes the torque of each joint and the wrench at the base. By default, the robot velocity and acceleration are set to 0.
         * @param[out] wrench_base wrench applied to the base.
         * @param[out] tau_joints torque of each joint.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] joint_acceleration acceleration of each joint.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         */
        virtual void inverseDynamics(Eigen::Matrix<double, 6, 1> &wrench_base,
                                    robotlib::JointState &tau_joints,
                                    const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                    const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    const robotlib::JointState &joint_acceleration,
                                    const Eigen::Matrix<double, 6, 1> &robot_velocity = Eigen::Matrix<double, 6, 1>::Zero(),
                                    const Eigen::Matrix<double, 6, 1> &robot_acceleration = Eigen::Matrix<double, 6, 1>::Zero()) const = 0;

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
                                            const Eigen::Matrix<double, 6, 1> &robot_acceleration = Eigen::Matrix<double, 6, 1>::Zero())
                                            const = 0;

        // ** SET FUNCTIONS **

        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) const = 0;

        virtual void setTrunkMass(const double& trunk_mass) const = 0;

		/**
		 * @brief Set inverse kinematics time period
         * @param period period of the controller
		 */
        virtual void setInvKinTimePeriod(const double& period) const = 0;


        // ** PLUGIN TYPEDEFS ** 

        typedef std::shared_ptr<RobotBase> createRobot_t();
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);
        typedef std::shared_ptr<RobotBase> createRobotWithUrdf_t(const std::string&);
        typedef void destroyRobotWithUrdf_t(std::shared_ptr<RobotBase>);

    protected:
        const std::string name_;
    };
} // namespace robotlib

#include "robot_base.tpp"

#endif // _ROBOTLIB_ROBOT_BASE_HPP_