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

        // ** GET FUNCTIONS **
        const std::string getName() const;

        virtual const int getNLEGS() const = 0;
        virtual const int getNARMS() const = 0;
        virtual const int getNJOINTS() const = 0;
        virtual const int getNLINKS() const = 0;

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

        virtual void getFootPosition(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Eigen::Vector3d &footPos) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual void getFootJacobian(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Jacobian &footJac) = 0; //overridden by Glue

        virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) = 0; //overridden by Glue


        // TODO: compute total mass from links and trunk masses (it could be even implemented in Robot class)
        virtual double getRobotMass() const = 0;

        virtual double getTrunkMass() const = 0;

        virtual double getLegsMass() const = 0;

        // NB: eventually make the get function void for possible NRT issue
        virtual const Eigen::Matrix<double, 3, 1>& getTrunkCOM() const = 0;

        virtual Eigen::Vector3d getRobotCoM() = 0;

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM() = 0;

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) const = 0;

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
        
   
        // ** FUNCTIONS TO MAKE NRT OBJECTS ** 

        // Create a joint state
        JointState makeJointState(const double value = 0.0) const;

        // Create a leg data map pair
        template <class Data>
        LegDataMap<Data> makeLegDataMap() const; // NRT

        // Create a leg data map pair
        template <class Data>
        LegDataMap<Data> makeLegDataMap(const Data &data) const; // NRT

        // Create a link data map pair
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(); // NRT
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(const Data &data); // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMap(); // NRT
        template <class Data>
        JointDataMap<Data> makeJointDataMap(const Data &data); // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg); // NRT
        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data &data); // NRT

        // TODO
        Jacobian makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest); // NRT

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<Frame> frame); // NRT

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data = 0.0); // NRT

        LegDataMap<Jacobian> makeFeetJacobian(const double data = 0.0); // NRT

        // ** FORWARD KINEMATICS ** 

        virtual void forwardKinematics(const JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                       const JointState &joint_velocity,
                                       const JointState &joint_acceleration,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) const = 0;

        // ** INVERSE KINEMATICS ** 
        
        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) const = 0;                              

        // ** INVERSE DYNAMICS ** 

        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                     JointState &tau_joints) const = 0;              ///output

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