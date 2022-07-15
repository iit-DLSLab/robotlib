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
        std::string getName();

        virtual const int getNLEGS() = 0;
        virtual const int getNARMS() = 0;
        virtual const int getNJOINTS() = 0;
        virtual const int getNLINKS() = 0;

        // virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) = 0;    ///TODO: required for the print inside CGaitTimerHex::run() of Ant Controller

        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;
        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

        virtual const std::shared_ptr<Link> getLink(const std::string &name) = 0;
        virtual const std::shared_ptr<Joint> getJoint(const std::string &name) = 0;
        virtual const std::shared_ptr<LimbBase> getLeg(const std::string &name) = 0;

        virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

        virtual void getMinJointAngle(JointState &q_min) = 0;
        virtual void getMaxJointAngle(JointState &q_max) = 0;
        virtual void getMaxJointVelocity(JointState &qd_max) = 0;
        virtual void getMaxJointEffort(JointState &tau_max) = 0;

        virtual void getMinJointAngle(const std::shared_ptr<Joint> joint, double &q_min) { q_min = joint->getMinAngle(); };
        virtual void getMaxJointAngle(const std::shared_ptr<Joint> joint, double &q_max) { q_max = joint->getMaxAngle(); };
        virtual void getMaxJointVelocity(const std::shared_ptr<Joint> joint, double &qd_max) { qd_max = joint->getMaxVelocity(); };
        virtual void getMaxJointEffort(const std::shared_ptr<Joint> joint, double &tau_max) { tau_max = joint->getMaxEffort(); };

        
        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Vector3d getFramePosition(const JointDataMap<double> &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Matrix3d getFrameOrientation(const JointDataMap<double> &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::shared_ptr<Frame> origin,
                                             const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Matrix4d getFramePose(const JointDataMap<double> &q,
                                             const std::shared_ptr<Frame> origin,
                                             const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Vector3d getFootPosition(const JointDataMap<double> &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Matrix3d getFootOrientation(const JointDataMap<double> &q,
                                                   const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Matrix4d getFootPose(const JointDataMap<double> &q,
                                            const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual void getFootPosition(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Eigen::Vector3d &footPos) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual void getFootPosition(const JointDataMap<double> &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Eigen::Vector3d &footPos) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Matrix3d getFootOrientation(const JointDataMap<double> &q,
                                                   const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual Eigen::Matrix4d getFootPose(const JointDataMap<double> &q,
                                            const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual void getFootJacobian(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Jacobian &footJac) = 0; //overridden by Glue

        /// TODO: verify if needed with JointDataMap<double>
        virtual void getFootJacobian(const JointDataMap<double> &q,
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

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) = 0;

        virtual Eigen::Vector3d getLegContribution(const JointState &q) = 0;

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
        JointState makeJointState(const double value = 0.0)
        {
            JointState joint_state(this->getLegs());

            for (auto leg : *this->getLegs())
            {
                JointDataMap<double> *jdm;
                jdm = new JointDataMap<double>(leg, value);
                std::shared_ptr<JointDataMap<double>> ptr(jdm);
                joint_state[leg] = ptr;
            }

            return joint_state;
        } // NRT

        // Create a leg data map pair
        template <class Data>
        LegDataMap<Data> makeLegDataMap() { return LegDataMap<Data>(this->getLegs()); } // NRT
        // Create a leg data map pair
        template <class Data>
        LegDataMap<Data> makeLegDataMap(const Data &data) { return LegDataMap<Data>(this->getLegs(), data); } // NRT

        // Create a link data map pair
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap() { return LinkDataMap<Data>(this); } // NRT
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(const Data &data) { return LinkDataMap<Data>(this, data); } // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMap() { return JointDataMap<Data>(this); } // NRT
        template <class Data>
        JointDataMap<Data> makeJointDataMap(const Data &data) { return JointDataMap<Data>(this, data); } // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg) { return JointDataMap<Data>(leg); } // NRT
        // Create a joint data map pair
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data &data) { return JointDataMap<Data>(leg, data); } // NRT

        // TODO
        Jacobian makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest) // NRT
        {
            std::cout << "makeJacobian function: TODO\n";
            return Jacobian(1);
        };

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<Frame> frame) // NRT
        {
            // Link foot = static_cast<const Link &>(frame); //TODO: try without static_cast

            // const LimbBase *l = foot.getParentLimb();
            // const int nJoints = l->getNJoints();

            // return Jacobian(nJoints);
            std::cout << "makeFootJacobian-Input: foot function: TODO\n";
            return Jacobian(1);
        };

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data = 0.0) // NRT
        {
            return Jacobian(leg->getNJoints(), data);
        };

        LegDataMap<Jacobian> makeFeetJacobian(const double data = 0.0) // NRT
        {
            auto feetJac = this->makeLegDataMap<Jacobian>();

            for (auto leg : *(this->getLegs()))
            {
                feetJac[leg].init(leg->getNJoints(), data);
            }
            return feetJac;
        };

        // ** FORWARD KINEMATICS ** 

        virtual void forwardKinematics(const JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                       const JointState &joint_velocity,
                                       const JointState &joint_acceleration,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) = 0;

        virtual void forwardKinematics(const Eigen::Vector3d &joint_position,
                                       const Eigen::Vector3d &joint_velocity,
                                       const Eigen::Vector3d &joint_acceleration,
                                       Eigen::Vector3d &end_effector_position,
                                       Eigen::Vector3d &end_effector_velocity,
                                       Eigen::Vector3d &end_effector_acceleration,
                                       const std::shared_ptr<Frame> end_effector) = 0; // TODO: Better to use end effector or leg (as in ANT controller)?


        // ** INVERSE KINEMATICS ** 
        
        virtual void inverseKinematics(const Eigen::Vector3d &end_effector_position,
                                       const Eigen::Vector3d &end_effector_velocity,
                                       const Eigen::Vector3d &end_effector_acceleration,
                                       Eigen::Vector3d &joint_position,
                                       Eigen::Vector3d &joint_velocity,
                                       Eigen::Vector3d &joint_acceleration,
                                       const std::shared_ptr<Frame> end_effector) = 0; // TODO: Better to use end effector or leg (as in ANT controller)?

        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;

        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       const LegDataMap<Jacobian> &robot_jacobian,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;
                                       

        // ** INVERSE DYNAMICS ** 

        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                     JointState &tau_joints) = 0;              ///output





        // ** SET FUNCTIONS **

        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) = 0;

        virtual void setTrunkMass(const double& trunk_mass) = 0;

		/**
		 * @brief Set inverse kinematics time period
         * @param period period of the controller
		 */
        virtual void setInvKinTimePeriod(const double& period) = 0;


        // ** PLUGIN TYPEDEFS ** 

        typedef std::shared_ptr<RobotBase> createRobot_t();
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);
        typedef std::shared_ptr<RobotBase> createRobotWithUrdf_t(const std::string&);
        typedef void destroyRobotWithUrdf_t(std::shared_ptr<RobotBase>);

    protected:
        const std::string name_;
    };
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_HPP_