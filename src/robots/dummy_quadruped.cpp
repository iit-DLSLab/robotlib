/**
 * @file dummy_quadruped.cpp
 *
 * @brief DummyLeg and DummyQuadruped class definition and functions implementation
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

#include "robot.hpp"
#include "leg.hpp"
#include "trunk.hpp"
#include "dyn_params.hpp"

#include <map>

namespace robotlib
{
namespace dummy_quadruped
{
	const int NJOINTS {3};
	const int NLINKS {3};
	const int NJOINTS_TOT {12};
	const int NLINKS_TOT {12};
	const int NLEGS {4};
	const int NARMS {0};

	class DummyLeg : public Leg<NLINKS, NJOINTS>
	{
	public:
		DummyLeg(const std::string& name,
				 const std::array<Link, NLINKS>& links,
				 const std::array<Joint, NJOINTS>& joints)
			: Leg<NLINKS, NJOINTS>(name, links, joints){}

		virtual ~DummyLeg(){};
	};

	class DummyQuadruped : public Robot<NLEGS, NLINKS_TOT, NJOINTS_TOT>
	{
	public:
		DummyQuadruped(Trunk& trunk, std::array<LimbBase, NLEGS>& limbs)
			: Robot<NLEGS, NLINKS_TOT, NJOINTS_TOT>("Quadruped", trunk, limbs)
		{
			// Set joint limits (dummy limits are used here)
			const double q_min {0};
			const double q_max {90};
			const double qd_max {3};
			const double tau_max {5};

			for (auto& joint: this->getJoints())
			{
				joint->setJointLimits(q_min, q_max, qd_max, tau_max);
			}
		}

		virtual ~DummyQuadruped(){}

		virtual Eigen::Vector3d getFramePosition(const JointState &q, const Frame& origin, const Frame& destination) const override
		{
        	q.size();
			origin.getName();
			destination.getName();

			return Eigen::Vector3d().setZero();
		}

        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                const JointState &joint_position,
                                                Eigen::Matrix<double, 6, 1> &wrench_base,
                                                JointState &tau_joints) const
		{
			gravity_vector.size();
			wrench_base.size();
			joint_position.size();
			tau_joints.size();

			std::cout << "computeGravityCompensation" << std::endl;
		}

        virtual Eigen::Matrix<double, 6,1> computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      						 		const JointState &joint_position) const
		{
			gravity_vector.size();
			joint_position.size();

			std::cout << "computeWrenchGravityCompensation" << std::endl;

			return Eigen::Matrix<double, 6, 1>::Zero();
		}

        virtual void computeTorquesGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position,
                                                      JointState &tau_joints) const
		{
			gravity_vector.size();
			joint_position.size();
			tau_joints.size();

			std::cout << "computeTorquesGravityCompensation" << std::endl;
		}

		virtual void inverseDynamicsHTerm(JointState &tau_joints,
										  const Eigen::Matrix<double, 6, 1> &gravity_vector,
										  const JointState &joint_position,
										  const JointState &joint_velocity,
										  const Eigen::Matrix<double, 6, 1> &robot_velocity = Eigen::Matrix<double, 6, 1>::Zero(),
										  const Eigen::Matrix<double, 6, 1> &robot_acceleration = Eigen::Matrix<double, 6, 1>::Zero()) const
		{
			tau_joints.size();
			gravity_vector.size();
			joint_position.size();
			joint_velocity.size();
			robot_velocity.size();
			robot_acceleration.size();

			std::cout << "inverseDynamicsHTerm" << std::endl;
		}

		virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
											const robotlib::Frame& origin,
											const robotlib::Frame& destination) const override
		{
        	q.size();
			origin.getName();
			destination.getName();

			return Eigen::Matrix3d().setZero();
		}

		virtual Eigen::Matrix4d getFramePose(const JointState &q,
									 const Frame& origin,
									 const Frame& destination) const override
		{
			Eigen::Matrix4d frame_pose{};
			frame_pose.setZero();

			frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
			frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
			frame_pose.row(3) << 0, 0, 0, 1;

			return frame_pose;
		}

		virtual Eigen::Vector3d getFootPosition(const JointState &q, const Frame& foot_frame) const override
		{
			return this->getFramePosition(q, this->getLink("TRUNK"), foot_frame);
		}

		virtual Eigen::Matrix3d getFootOrientation(const JointState &q, const Frame& foot_frame) const override
		{
			return this->getFrameOrientation(q, this->getLink("TRUNK"), foot_frame);
		}

		virtual Eigen::Matrix4d getFootPose(const JointState &q, const Frame& foot_frame) const override
		{
			Eigen::Matrix4d foot_pose{};
			foot_pose.setZero();

			foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot_frame);
			foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot_frame);
			foot_pose.row(3) << 0, 0, 0, 1;

			return foot_pose;
		}

		virtual void updateLinearJacobian(const JointState &joints_positions,
										  LimbDataMap<Jacobian> &robot_jacobian) const override
		{
			joints_positions.size();
			robot_jacobian.size();
		}

		virtual void getFootJacobian(const JointState& q,
									 const LimbBase& limb,
									 Jacobian& footJac) const override
		{
			q.size();
			limb.getName();
			footJac.setZero();
		}

		virtual void updateAngularJacobian(const JointState& joints_positions,
								   		   LimbDataMap<Jacobian>& robot_jacobian) const override
		{
			joints_positions.size();
			robot_jacobian.size();
			std::cout << "Update Angular Jacobian" << std::endl;
		}

		virtual void updateLinearFootJacobian(const JointState& joints_positions,
                                        	  const LimbBase& limb,
                                        	  Jacobian& footJac) const override
		{
			joints_positions.size();
			limb.getName();
			footJac.size();
			std::cout << "Update Linear Foot Jacobian" << std::endl;
		}

    	virtual void updateAngularFootJacobian(const JointState& joints_positions,
                                               const LimbBase& limb,
                                               Jacobian& footJac) const override
		{
			joints_positions.size();
			limb.getName();
			footJac.size();
			std::cout << "Update Angular Foot Jacobian" << std::endl;
		}
        
		virtual void forwardKinematics(const JointState& joint_position,
                               		   LimbDataMap<Eigen::Vector3d>& end_effector_position) const  override
		{
			joint_position.size();
			end_effector_position.size();
		}

        virtual void forwardKinematics(const JointState &joint_position,
                               const JointState &joint_velocity,
                               LimbDataMap<Eigen::Vector3d> &end_effector_position,
                               LimbDataMap<Eigen::Vector3d> &end_effector_velocity) const  override
		{
			joint_position.size();
			joint_velocity.size();
			end_effector_position.size();
			end_effector_velocity.size();
		}

		virtual void inverseKinematics(const LimbDataMap<Eigen::Vector3d> &end_effector_position,
							   const LimbDataMap<Eigen::Vector3d> &end_effector_velocity,
							   const LimbDataMap<Eigen::Vector3d> &end_effector_acceleration,
							   JointState &joint_position,
							   JointState &joint_velocity,
							   JointState &joint_acceleration) const  override
		{
			end_effector_position.size();
			end_effector_velocity.size();
			end_effector_acceleration.size();
			joint_position.size();
			joint_velocity.size();
			joint_acceleration.size();
		}

		virtual void inverseKinematics(const LimbDataMap<Eigen::Vector3d> &end_effector_position,
							   const LimbDataMap<Eigen::Vector3d> &end_effector_velocity,
							   JointState &joint_position,
							   JointState &joint_velocity) const  override
		{
			end_effector_position.size();
			end_effector_velocity.size();
			joint_position.size();
			joint_velocity.size();
		}

		virtual void inverseKinematics(const LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                       JointState &joint_position) const  override
		{
			end_effector_position.size();
			joint_position.size();
		}

		virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
							 const Eigen::Matrix<double, 6, 1> &robot_acceleration,
							 const Eigen::Matrix<double, 6, 1> &gravity_vector,
							 const JointState &joint_position,
							 const JointState &joint_velocity,
							 const JointState &joint_acceleration,
							 Eigen::Matrix<double, 6, 1> &wrench_base,
							 JointState &tau_joints) const  override
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

		virtual double getRobotMass() const override
		{
			return 0.0;
		}

        virtual double getTrunkMass() const override
		{
			return 0.0;
		}

		virtual double getLegsMass() const override
		{
			return 0.0;
		}

		virtual Eigen::Vector3d getWholeBodyCOM(const JointState& joint_position) const override
		{
			joint_position.size();

			return Eigen::Vector3d::Zero();
		}

        virtual Eigen::Vector3d getLegContribution(const JointState& q) const override
		{
			q.size();

			return Eigen::Vector3d::Zero();
		}

		virtual Eigen::Vector3d getCoMFromBase(const JointState& q,
									   		   const Eigen::Vector3d& base_orient,
									   		   const Eigen::Vector3d& base_pos) const override
		{
			q.size();
			base_orient.size();
			base_pos.size();

			return Eigen::Vector3d::Zero();
		}

        virtual Eigen::Vector3d getBaseFromCoM(const JointState& q,
                                       		   const Eigen::Vector3d& base_orient,
                                       		   const Eigen::Vector3d& CoM) const override
		{
			q.size();
			base_orient.size();
			CoM.size();

			std::cout << "Get the base from COM" << std::endl;

			return Eigen::Vector3d::Zero();
		}

        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVel(const JointState& q,
                                                              const JointState& qd) const override
		{
			q.size();
			qd.size();

			return Eigen::Matrix<double, 6,1>::Zero();
		}

		virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1>& baseVel,
                                                               	const Eigen::Matrix3d& R,
                                                               	const JointState& q) const override
		{
			baseVel.size();
			R.size();
			q.size();
			
			std::cout << "Get whole body COM vel FB - without joint influence" << std::endl;

			return Eigen::Matrix<double, 6, 1>::Zero();	
		};

		virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1>& baseVel,
                                                                const Eigen::Matrix3d& R,
                                                                const Eigen::Vector3d& offset_com) const override
		{	
			baseVel.size();
			R.size();
			offset_com.size();
			
			std::cout << "Get whole body COM vel FB, with com offset as input, without considering joint influence" << std::endl;

			return Eigen::Matrix<double, 6, 1>::Zero();
		}
		
		virtual void setInvKinTimePeriod(const double period) override
		{
			std::cout << period << std::endl;
		}

		virtual void setTrunkCom (const Eigen::Vector3d &trunk_com) override
		{
			trunk_com.size();
		}

		virtual void setTrunkMass(const double trunk_mass) override
		{
			std::cout << trunk_mass << std::endl;
		}

		virtual Eigen::Vector3d getRobotCoM() const override
		{
			return Eigen::Vector3d::Zero();
		}

        /*!
        *@brief Get the IMU pose in base frame.
        *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
        *@param[in] base_link_name name of the base link.
        *@return IMU pose in base frame.
        */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string& imu_link_name="trunk_imu", const std::string& base_link_name="base_link") const override
        {
			std::cout << imu_link_name << std::endl;
			std::cout << base_link_name << std::endl;

        	return Eigen::Matrix4d::Zero();
        }
	};
} // namespace dummy_quadruped
} // namespace robotlib

robotlib::dummy_quadruped::DummyLeg makeLeg(const std::string &legName, robotlib::Link& trunk)
{
	robotlib::Joint haa(legName + "_HAA", trunk);
	robotlib::Link assembly(legName + "_ASSEMBLY", haa);
	robotlib::Joint hfe(legName + "_HFE", assembly);
	robotlib::Link upperleg(legName + "_UPPERLEG", hfe);
	robotlib::Joint kfe(legName + "_KFE", upperleg);
	robotlib::Link lowerleg(legName + "_LOWERLEG", kfe);

	return robotlib::dummy_quadruped::DummyLeg(legName, {assembly, upperleg, lowerleg}, {haa, hfe, kfe});
}

extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t()
{
	const robotlib::DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
	
	robotlib::Trunk trunk("TRUNK", trunk_dyn_params);
	
	std::array<robotlib::LimbBase, robotlib::dummy_quadruped::NLEGS> 
		limbs({makeLeg("LF", trunk), makeLeg("RF", trunk), makeLeg("LH", trunk), makeLeg("RH", trunk)});
	
	return std::make_shared<robotlib::dummy_quadruped::DummyQuadruped>(trunk, limbs);
}

extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot){ robot->getName(); }