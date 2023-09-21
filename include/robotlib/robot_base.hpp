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
    /*!
     * @class RobotBase
     * @brief RobotBase class.
     * @details
     * This class represents a generic abstract robot.  It provides data structures such as LegDataMap, JointDataMap, JointState and Jacobian classes. It also provides the hierarchical structure of limbs as a sequence of joints and links together with utility functions like forwardKinematics, inverseKinematics and inverseDynamics.
    */
    class RobotBase
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] name name of the robot
         */
        RobotBase(const std::string &name);

        /*!
         * @brief Destructor.
         */
        virtual ~RobotBase();

        // ** GET FUNCTIONS **
        /*!
         * @brief Get robot name.
         * @return robot name.
         */
        const std::string getName() const;

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

        /*!
        * @brief Compute the number of legs in stance
        * @param stance_legs
        * @return
        */
        int computeNumStanceLegs(const LegDataMap<bool>& stance_legs) const;

        /*!
        * @brief Computing robot proprio height
        * @param[in] w_rpy_b orientation of the base frame in world coordinates
        * @param[in] stance_legs variable identifying the stance status of each leg
        * @param[in] actual_foot_position actual foot position
        * @param[out] proprio_height robot proprio height
        * @remark{DMA}
        */
        void computeProprioHeight(const Eigen::Vector3d& w_rpy_b, const LegDataMap<bool>& stance_legs, const LegDataMap<Eigen::Vector3d>& actual_foot_position, double& proprio_height) const;

        /*!
         * @brief Estimate feet external ground reaction forces
        * @param[in] q
        * @param[in] qd
        * @param[in] qdd
        * @param[in] tau
        * @param[in] g_b gravity vector in base frame
        * @param[out] estimated_feet_grf
        */
        void estimateFeetGRF(const robotlib::JointState& q, const robotlib::JointState& qd, const robotlib::JointState& qdd, const robotlib::JointState& tau, const Eigen::Matrix<double, 6,1>& g_b, robotlib::LegDataMap<Eigen::Vector3d>& estimated_feet_grf);

        /*!
         * @brief Get number of robot's legs.
         * @return number of robot's legs.
         */
        virtual int getNLEGS() const = 0;

        /*!
         * @brief Get number of robot's arms.
         * @return number of robot's arms.
         */
        virtual int getNARMS() const = 0;

        /*!
         * @brief Get number of robot's joints.
         * @return number of robot's joints.
         */
        virtual int getNJOINTS() const = 0;

        /*!
         * @brief Get number of robot's links.
         * @return number of robot's links.
         */
        virtual int getNLINKS() const = 0;

        /*!
         * @brief Get robot's legs.
         * @return robot's legs as a shared pointer to a ContainerBase object.
         */
        virtual std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;

        /*!
         * @brief Get robot's arms.
         * @return robot's arms as a shared pointer to a ContainerBase object.
         */
        virtual std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

        /*!
         * @brief Get robot's link from link's name.
         * @param[in] name name of the link.
         * @return a shared pointer pointing to the link.
         */
        virtual std::shared_ptr<Link> getLink(const std::string &name) const = 0;

        /*!
         * @brief Get robot's joint from joint's name.
         * @param[in] name name of the joint.
         * @return a shared pointer pointing to the joint.
         */
        virtual std::shared_ptr<Joint> getJoint(const std::string &name) const = 0;

        /*!
         * @brief Get robot's leg from leg's name.
         * @param[in] name name of the leg.
         * @return a shared pointer pointing to the leg.
         */
        virtual std::shared_ptr<LimbBase> getLeg(const std::string &name) const = 0;

        /*!
         * @brief Get robot's arm from arm's name.
         * @param[in] name name of the arm.
         * @return a shared pointer pointing to the arm.
         */
        virtual std::shared_ptr<LimbBase> getArm(const std::string &name) const = 0;

        /*!
         * @brief Get lower angle limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the lower limits of the joints' angles. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] q_min a joint state object to be filled with the lower limits of the joints' angles.
         */
        virtual void getMinJointAngle(JointState &q_min) = 0;

        /*!
         * @brief Get upper angle limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the upper limits of the joints' angles. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] q_max a joint state object to be filled with the upper limits of the joints' angles.
         */
        virtual void getMaxJointAngle(JointState &q_max) = 0;

        /*!
         * @brief Get maximum velocity limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the maximum velocity limits of the joints.This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] qd_max a joint state object to be filled with the maximum velocity limits of the joints.
         */
        virtual void getMaxJointVelocity(JointState &qd_max) = 0;

        /*!
         * @brief Get maximum torque limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the maximum torque limits of the joints. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] tau_max a joint state object to be filled with the maximum torque limits of the joints.
         */
        virtual void getMaxJointEffort(JointState &tau_max) = 0;

        /*!
         * @brief Get lower angle limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's lower angle limit.
         */
        virtual double getMinJointAngle(const std::shared_ptr<Joint> joint);

        /*!
         * @brief Get upper angle limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's upper angle limit.
         */
        virtual double getMaxJointAngle(const std::shared_ptr<Joint> joint);

        /*!
         * @brief Get maximum velocity limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's maximum velocity limit.
         */
        virtual double getMaxJointVelocity(const std::shared_ptr<Joint> joint);

        /*!
         * @brief Get maximum torque limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's maximum torque limit.
         */
        virtual double getMaxJointEffort(const std::shared_ptr<Joint> joint);

        /*!
         * @brief Get position of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame position expressed in origin one.
         */
        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) const = 0;

        /*!
         * @brief Get orientation of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame orientation expressed in origin one.
         */
        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) const = 0;

        /*!
         * @brief Get pose of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame pose expressed in origin one.
         */
        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::shared_ptr<Frame> origin,
                                             const std::shared_ptr<Frame> destination) const = 0;

        /*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot position expressed in trunk frame.
         */
        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                       const std::shared_ptr<Frame> foot) const = 0;

        /*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot position.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot position expressed in trunk frame.
         */
        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                       const std::shared_ptr<LimbBase> leg) const = 0;

        /*!
         * @brief Get foot orientation expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot orientation expressed in trunk frame.
         */
        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                          const std::shared_ptr<Frame> foot) const = 0;

        /*!
         * @brief Get foot orientation with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot orientation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot orientation expressed in trunk frame.
         */
        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                         const std::shared_ptr<LimbBase> leg) const = 0;

        /*!
         * @brief Get foot pose expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot pose expressed in trunk frame.
         */
        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<Frame> foot) const = 0;

        /*!
         * @brief Get foot pose with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot pose.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot pose expressed in trunk frame.
         */
        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                  const std::shared_ptr<LimbBase> leg) const = 0;

        /*!
         * @brief Update the linear part of the jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
        virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) const = 0;

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

        /*!
         * @brief Get total robot mass.
         * @return total robot mass.
         */
        virtual double getRobotMass() const = 0;

        /*!
         * @brief Get trunk mass.
         * @return trunk mass.
         */
        virtual double getTrunkMass() const = 0;

        /*!
         * @brief Get total legs' mass.
         * @return total legs' mass.
         */
        virtual double getLegsMass() const = 0;

        /*!
         * @brief Get the CoM of the trunk.
         * @return trunk's CoM.
         */
        virtual Eigen::Vector3d getTrunkCOM() const = 0;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d getRobotCoM() const = 0;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d getWholeBodyCOM(const JointState &joint_state) const = 0;

        /*!
         * @brief Compute CoM legs contribution in base frame.
         * @param[in] q angles of the joints.
         * @return CoM legs contribution in base frame.
         */
        virtual Eigen::Vector3d getLegContribution(const JointState &q) const = 0;


        /*!
         * @brief Compute robot CoM position in world frame, from base pose in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] base_pos base position in world frame.
         * @return CoM position in world frame.
         */
        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) const = 0;

        /*!
          * @brief Compute robot base position in world frame, from CoM position in world frame.
          * @param[in] q angles of the joints.
          * @param[in] base_orient base orientation in world frame.
          * @param[in] com robot CoM postion in world frame.
          * @return base position in world frame.
         */
        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &CoM) const = 0;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
		 */
        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVel(const JointState &q,
                                                               const JointState &qd) const = 0;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
		 */
        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &R,
                                                                const JointState &q) const = 0;

        /*!
         * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] offset_com CoM offset in base frame.
         * @return CoM velocity in world frame.
         */
        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &R,
                                                                 const Eigen::Vector3d offset_com) const = 0;

        /*!
        *@brief Get the IMU pose in base frame.
        *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
        *@param[in] base_link_name name of the base link.
        *@return IMU pose in base frame.
        */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string& imu_link_name="trunk_imu", const std::string& base_link_name="base_link") const = 0;

        // ** FUNCTIONS TO MAKE NRT OBJECTS ** 

        /*!
         * @brief Function to create a JointState object.
         * @param[in] value value used to initialize the joint state.
         * @return joint state.
         */
        JointState makeJointState(const double value = 0.0) const;

        /*!
         * @brief Function to create a LegDataMap object object.
         * @tparam Data data type associated to each leg.
         * @return LegDataMap<Data> object.
         */
        template <class Data>
        LegDataMap<Data> makeLegDataMap() const; // NRT

        /*!
         * @brief Function to create a LegDataMap object.
         * @tparam Data data type associated to each leg.
         * @param[in] data data used to initialize the LegDataMap object.
         * @return LegDataMap<Data> object.
         */
        template <class Data>
        LegDataMap<Data> makeLegDataMap(const Data& data) const;

        /*!
         * @brief Function to create a LinkDataMap object.
         * @tparam Data data type associated to each link.
         * @return LinkDataMap<Data> object.
         */
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(); // NRT

        /*!
         * @brief Function to create a LinkDataMap object.
         * @tparam Data data type associated to each link.
         * @param[in] data data used to initialize the LinkDataMap object.
         * @return LinkDataMap<Data> object.
         */
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(const Data& data); // NRT

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the robot.
         * @tparam Data data type associated to each joint.
         * @return JointDataMap<Data> object.
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMap(); // NRT

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the robot.
         * @tparam Data data type associated to each joint.
         * @param[in] data data used to initialize the JointDataMap object.
         * @return JointDataMap<Data> object.
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMap(const Data& data); // NRT

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the leg in input.
         * @tparam Data data type associated to each joint.
         * @param[in] leg leg whose joints are used to create the JointDataMap object.
         * @return JointDataMap<Data> object.
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg); // NRT

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the leg in input.
         * @tparam Data data type associated to each joint.
         * @param[in] leg leg whose joints are used to create the JointDataMap object.
         * @param[in] data data used to initialize the JointDataMap object.
         * @return JointDataMap<Data> object.
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data&); // NRT

        // TODO
        Jacobian makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest); // NRT

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<Frame> frame); // NRT

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data = 0.0); // NRT

        /*!
         * @brief Function to create a LegDataMap object, associating a Jacobian to each leg.
         * @details
         * Each Jacobian has dimention 6xn_joints_leg. The rows are 6 to have both linear and angular parts of the jacobian; n_joints_leg is the  number of joints of the leg to which the Jacobian is associated to.
         * @param[in] data data used to initialize the JointDataMap object.
         * @return LegDataMap<Jacobian> object.
         */
        LegDataMap<Jacobian> makeFeetJacobian(const double data = 0.0) const; // NRT

        // ** FORWARD KINEMATICS ** 

        /*!
          * @brief Forward kinematics.
          * @details
          * It computes the position of each end effector (foot) expressed in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[out] end_effector_position position of each end effector (foot) in base frame.
          */
        virtual void forwardKinematics(const JointState &joint_position,
                                       LegDataMap<Eigen::Vector3d> &end_effector_position) const = 0;
        /*!
         * @brief Forward kinematics.
         * @details
         * It computes the position and velocity of each end effector (foot) expressed in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[out] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] end_effector_velocity velocity of each end effector (foot) in base frame.
         */
        virtual void forwardKinematics(const JointState &joint_position,
                                       const JointState &joint_velocity,
                                       LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       LegDataMap<Eigen::Vector3d> &end_effector_velocity) const = 0;
        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle, velocity and acceleration of each joint from the position, velocity and acceleration of each end effector expressed in base frame.
        *
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[in] end_effector_velocity velocity of each end effector (foot) in base frame.
         * @param[in] end_effector_acceleration acceleration of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         * @param[out] joint_velocity velocity of each joint.
         * @param[out] joint_acceleration acceleration of each joint.
         */
        virtual void inverseKinematics(const LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       const LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       const LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) const = 0;
        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle of each joint from the position of each end effector expressed in base frame.
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         */
        virtual void inverseKinematics(const LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       JointState &joint_position) const = 0;

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
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base,
                                     JointState &tau_joints) const = 0;

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

        /*!
         * @brief Set trunk's CoM.
         * @param[in] trunk_com CoM of trunk to be set.
         */
        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) = 0;

        /*!
         * @brief Set trunk's mass.
         * @param[in] trunk_mass mass of trunk to be set.
         */
        virtual void setTrunkMass(const double trunk_mass) = 0;

		/*!
		 * @brief Set inverse kinematics time period.
         * @details
         * This time period is the controller's loop time period. The time period needs to be set before calling the inverse kinematics.
         * @param[in] period period of the controller.
		 */
        virtual void setInvKinTimePeriod(const double period) = 0;

		/*!
		 * @brief Print robot hierarchy.
		 */
        void printRobotHierarchy();

        // ** CLASS FACTORY FUNCTION DECLARATIONS **

        /*!
         * @brief Factory function to load at run-time the glue code, creating a robot object.
		 */
        typedef std::shared_ptr<RobotBase> createRobot_t();

        /*!
         * @brief Factory function to load at run-time the glue code, with external urdf in input.
         * @param[in] robot_urdf the urdf of the robot in string format.
		 */
        typedef std::shared_ptr<RobotBase> createRobotWithUrdf_t(const std::string& robot_urdf);

         /*!
         * @brief Factory function to destroy the robot object.
		 */
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

    protected:
        //! Robot name
        const std::string name_{};
    };
} // namespace robotlib

#include "robot_base.tpp"

#endif // _ROBOTLIB_ROBOT_BASE_HPP_