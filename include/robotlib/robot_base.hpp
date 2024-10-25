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

#include "robotlib/utils/eigen_utils.hpp"
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
        * @param[in] robot_pose pose of the robot base in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] g_base gravity term related to the base.
        * @param[out] g_joints gravity term related to the joints.
        */
        virtual void computeGravityTerm(    const Eigen::Matrix<double, 7, 1> &robot_pose,
                                            const robotlib::JointState &joint_position,
                                            Eigen::Matrix<double, 6, 1> &g_base,
                                            robotlib::JointState &g_joints) = 0;
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
                                          robotlib::JointState &g_joints) = 0;
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
                                        robotlib::JointState &nle_joints) = 0;

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
                                        robotlib::JointState &nle_joints) = 0;
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
        * @param[in] pose robot pose
        * @param[in] q
        * @param[in] qd
        * @param[in] qdd
        * @param[in] tau
        * @param[in] g_b gravity vector in base frame
        * @param[out] estimated_feet_grf
        */
        void estimateFeetGRF(const Eigen::Matrix<double,7,1>& pose, const robotlib::JointState& q, const robotlib::JointState& qd, const robotlib::JointState& qdd, const robotlib::JointState& tau, robotlib::LegDataMap<Eigen::Vector3d>& estimated_feet_grf);

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
         * @brief Get position of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::string origin,
                                                 const std::string destination) = 0;

        /*!
         * @brief Get orientation of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::string origin,
                                                    const std::string destination) = 0;

        /*!
         * @brief Get pose of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::string origin,
                                             const std::string destination) = 0;

        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame, related to the limbs only (so considering the actuated joints). The order is linear_jacobian, angular_jacobian. For a complete jacobian, see getFrameWholeBodyJacobian.
        * @param[in] q angles of the joints.
        * @param[in] frame_name name of the frame.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeLimbsJacobian( const robotlib::JointState &q,
                                    const std::string& frame_name,
                                    Eigen::MatrixXd &jacobian) = 0;
        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame. The order is linear_jacobian, angular_jacobian. For a jacobian considering only the joints, see getLimbsJacobian.
        * @param[in] robot_pose pose of the robot base in world frame.
        * @param[in] q angles of the joints.
        * @param[in] frame_name name of the frame.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeWholeBodyJacobian(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &q,
                                        const std::string& frame_name,
                                        Eigen::MatrixXd &jacobian) = 0;
        /*!
         * @brief Get total robot mass.
         * @return total robot mass.
         */
        virtual double getRobotMass() const = 0;
        // inertia?
        /*!
         * @brief Get link mass.
         * @param[in] name name of the link.
         * @return link mass.
         */
        virtual double getLinkMass(const std::string& name) const = 0;

        /*!
         * @brief Get link inertia about the CoM.
         * @param[in] name name of the link.
         * @return link inertia.
        */
        virtual Eigen::Matrix3d getLinkInertia(const std::string& name) const = 0;

        /*!
         * @brief Get link CoM in the joint frame(see https://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file).
         * @param[in] name name of the link.
         * @return link CoM.
        */
        virtual Eigen::Vector3d getLinkCOM(const std::string& name) const = 0;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @param[in] q angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d getWholeBodyCoM(const JointState q) = 0;

        /*!
         * @brief Compute robot CoM position in world frame, from base pose in world frame.
         * @param[in] q angles of the joints.
         * @param[in] robot_pose robot pose (position, quaternion (x,y,z,w)) in world frame.
         * @return CoM position in world frame.
         */
        virtual Eigen::Vector3d getCoMFromBase(const robotlib::JointState &q,
                                const Eigen::Matrix<double, 7, 1> &robot_pose);

        /*!
          * @brief Compute robot base position in world frame, from CoM position in world frame.
          * @param[in] q angles of the joints.
          * @param[in] base_orient base orientation in world frame (quaternion (x,y,z,w)).
          * @param[in] com robot CoM postion in world frame.
          * @return base position in world frame.
         */
        Eigen::Vector3d getBaseFromCoM( const JointState &q,
                                    const Eigen::Matrix<double, 4, 1> &base_orient,
                                    const Eigen::Vector3d &CoM);

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
                                       LegDataMap<Eigen::Vector3d> &end_effector_position) = 0;
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
                                       LegDataMap<Eigen::Vector3d> &end_effector_velocity) = 0;

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
                                                robotlib::JointState &q_des) = 0;
        /*!
        * @brief Inverse kinematics considering all the legs. Does not consider the floating base joint. It computes the joint angles from the desired frame position expressed in base frame. Redundancy is not handled yet.
        * @param[in] q_init_guess initial guess for the joint angles.
        * @param[in] positions_des desired positions of all the legs expressed in base frame.
        * @param[out] q_des desired joint angles.
        */
        virtual void fixedBaseInverseKinematics(const robotlib::JointState &q_init_guess,
                                                const robotlib::LegDataMap<Eigen::Vector3d> &positions_des,
                                                robotlib::JointState &q_des) = 0;
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
                                                            robotlib::JointState &qd_des) = 0;
        /*!
        * @brief Inverse differential kinematics for all the legs. Does not consider the floating base joint. It computes the joint velocities from the desired frame linear velocity expressed in base frame. Redundancy is not handled yet.
        * @param[in] q joint angles
        * @param[in] velocity_des desired linear velocities of all the legs expressed in base frame
        * @param[out] qd_des desired joint velocities
        */
        virtual void fixedBaseInverseDiffKinematics(const robotlib::JointState &q,
                                                    const robotlib::LegDataMap<Eigen::Vector3d> &velocities_des,
                                                    robotlib::JointState &qd_des) = 0;
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
                                robotlib::JointState &tau_joints) = 0;

        // ** SET FUNCTIONS **

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