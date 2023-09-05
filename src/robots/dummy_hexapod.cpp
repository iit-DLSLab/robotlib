/**
 * @file dummy_hexapod.cpp
 *
 * @brief DummyLeg and DummyHexapod class definition and functions implementation
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
	const int NJOINTS {3};
	const int NLINKS {3};

	class DummyLeg : public Leg<NJOINTS, NLINKS>
	{
	public:
		DummyLeg(const std::string &name,
				 const std::array<std::shared_ptr<robotlib::Joint>, NJOINTS> &joints,
				 const std::array<std::shared_ptr<robotlib::Link>, NLINKS> &links)
			: Leg<NJOINTS, NLINKS>(name, joints, links),
			  jointMap({//joint name, parent name, child name
						{"LF_HAA", std::make_pair("TRUNK", "LF_ASSEMBLY")},
						{"LF_HFE", std::make_pair("LF_ASSEMBLY", "LF_UPPERLEG")},
						{"LF_KFE", std::make_pair("LF_UPPERLEG", "LF_LOWERLEG")},
						{"RF_HAA", std::make_pair("TRUNK", "RF_ASSEMBLY")},
						{"RF_HFE", std::make_pair("RF_ASSEMBLY", "RF_UPPERLEG")},
						{"RF_KFE", std::make_pair("RF_UPPERLEG", "RF_LOWERLEG")},
						{"LH_HAA", std::make_pair("TRUNK", "LH_ASSEMBLY")},
						{"LH_HFE", std::make_pair("LH_ASSEMBLY", "LH_UPPERLEG")},
						{"LH_KFE", std::make_pair("LH_UPPERLEG", "LH_LOWERLEG")},
						{"RH_HAA", std::make_pair("TRUNK", "RH_ASSEMBLY")},
						{"RH_HFE", std::make_pair("RH_ASSEMBLY", "RH_UPPERLEG")},
						{"RH_KFE", std::make_pair("RH_UPPERLEG", "RH_LOWERLEG")},
						{"LC_HAA", std::make_pair("TRUNK", "LC_ASSEMBLY")},
						{"LC_HFE", std::make_pair("LC_ASSEMBLY", "LC_UPPERLEG")},
						{"LC_KFE", std::make_pair("LC_UPPERLEG", "LC_LOWERLEG")},
						{"RC_HAA", std::make_pair("TRUNK", "RC_ASSEMBLY")},
						{"RC_HFE", std::make_pair("RC_ASSEMBLY", "RC_UPPERLEG")},
						{"RC_KFE", std::make_pair("RC_UPPERLEG", "RC_LOWERLEG")}}),
			  linkMap({//link name, parent name, child name
					   {"LF_ASSEMBLY", std::make_pair("LF_HAA", "LF_HFE")},
					   {"LF_UPPERLEG", std::make_pair("LF_HFE", "LF_KFE")},
					   {"LF_LOWERLEG", std::make_pair("LF_KFE", "")},
					   {"RF_ASSEMBLY", std::make_pair("RF_HAA", "RF_HFE")},
					   {"RF_UPPERLEG", std::make_pair("RF_HFE", "RF_KFE")},
					   {"RF_LOWERLEG", std::make_pair("RF_KFE", "")},
					   {"LH_ASSEMBLY", std::make_pair("LH_HAA", "LH_HFE")},
					   {"LH_UPPERLEG", std::make_pair("LH_HFE", "LH_KFE")},
					   {"LH_LOWERLEG", std::make_pair("LH_KFE", "")},
					   {"RH_ASSEMBLY", std::make_pair("RH_HAA", "RH_HFE")},
					   {"RH_UPPERLEG", std::make_pair("RH_HFE", "RH_KFE")},
					   {"RH_LOWERLEG", std::make_pair("RH_KFE", "")},
					   {"LC_ASSEMBLY", std::make_pair("LC_HAA", "LC_HFE")},
					   {"LC_UPPERLEG", std::make_pair("LC_HFE", "LC_KFE")},
					   {"LC_LOWERLEG", std::make_pair("LC_KFE", "")},
					   {"RC_ASSEMBLY", std::make_pair("RC_HAA", "RC_HFE")},
					   {"RC_UPPERLEG", std::make_pair("RC_HFE", "RC_KFE")},
					   {"RC_LOWERLEG", std::make_pair("RC_KFE", "")}}){}

		virtual ~DummyLeg(){}

		virtual std::string jointToChildName(const std::shared_ptr<Joint> joint) const override
		{
			const std::string joint_name {joint->getName()};
			const std::string child_name {jointMap.find(joint_name)->second.second}; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
			return child_name;
		}

		virtual std::string jointToParentName(const std::shared_ptr<Joint> joint) const override
		{
			const std::string joint_name {joint->getName()};
			const std::string parent_name {jointMap.find(joint_name)->second.first};
			return parent_name;
		}

		virtual std::string linkToChildName(const std::shared_ptr<Link> link) const override
		{
			const std::string link_name {link->getName()};
			const std::string child_name {linkMap.find(link_name)->second.second};
			return child_name;
		}

		virtual std::string linkToParentName(const std::shared_ptr<Link> link) const override
		{
			const std::string link_name {link->getName()};
			const std::string parent_name {linkMap.find(link_name)->second.first};
			return parent_name;
		}

	private:
		const std::map<std::string, std::pair<std::string, std::string>> jointMap;
		const std::map<std::string, std::pair<std::string, std::string>> linkMap;
	};

	const int NJOINTS_TOT {18};
	const int NLINKS_TOT {18};
	const int NLEGS {6};
	const int NARMS {0};

	class DummyHexapod : public Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>
	{
	public:
		DummyHexapod(const std::shared_ptr<Trunk> trunk,
					   const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
					   const std::array<std::shared_ptr<LimbBase>, NARMS> arms)
			: Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>(
				  "Hexapod",
				  trunk,
				  std::make_shared<const Container<std::shared_ptr<LimbBase>, NLEGS>>(legs),
				  std::make_shared<const Container<std::shared_ptr<LimbBase>, NARMS>>(arms))

		{
			std::array<std::shared_ptr<Joint>, NLEGS> children;
			children[0] = getJoint("LF_HAA");
			children[1] = getJoint("RF_HAA");
			children[2] = getJoint("LH_HAA");
			children[3] = getJoint("RH_HAA");
			children[4] = getJoint("LC_HAA");
			children[5] = getJoint("RC_HAA");

			setChildrenOfTrunk(std::make_shared<Container<std::shared_ptr<Joint>, NLEGS>>(children));

			setParentOfLink(trunk_, nullptr);

			for (auto leg : *(this->getLegs()))
			{
				for (auto joint : *(leg->getJoints()))
				{
					const std::string child_name {leg->jointToChildName(joint)};
					setChildOfJoint(joint, getLink(child_name));
					const std::string parent_name {leg->jointToParentName(joint)};
					setParentOfJoint(joint, getLink(parent_name));
				}
			}

			for (auto leg : *(this->getLegs()))
			{
				for (auto link : *(leg->getLinks()))
				{
					const std::string child_name {leg->linkToChildName(link)};
					setChildOfLink(link, getJoint(child_name));

					const std::string parent_name {leg->linkToParentName(link)};
					setParentOfLink(link, getJoint(parent_name));
				}
			}

			// Set joint limits (dummy limits are used here)
			const double q_min {0};
			const double q_max {90};
			const double qd_max {3};
			const double tau_max {5};

			for (auto leg : *(this->getLegs()))
			{
				for (auto joint : *(leg->getJoints()))
				{
					setJointLimits(joint, q_min, q_max, qd_max, tau_max);
				}
			}
		}

		virtual ~DummyHexapod(){}

		virtual Eigen::Vector3d getFramePosition(const JointState &q,
										 				const std::shared_ptr<Frame> origin,
										 				const std::shared_ptr<Frame> destination) const override
		{
        	q.size();
			origin->getName();
			destination->getName();

			return Eigen::Vector3d().setZero();
		}

        virtual Eigen::Matrix<double, 6,1> computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      						 		const JointState &joint_position) const
		{
			gravity_vector.size();
			joint_position.getSize();
			return Eigen::Matrix<double, 6, 1>::Zero();

			std::cout << "computeWrenchGravityCompensation" << std::endl;
		}

        virtual void computeTorquesGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position,
                                                      JointState &tau_joints) const
		{
			gravity_vector.size();
			joint_position.getSize();
			tau_joints.getSize();

			std::cout << "computeTorquesGravityCompensation" << std::endl;
		}

		virtual void inverseDynamicsHTerm(  JointState &tau_joints,
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
											const std::shared_ptr<Frame> origin,
											const std::shared_ptr<Frame> destination) const override
		{
        	q.size();
			origin->getName();
			destination->getName();

			return Eigen::Matrix3d().setZero();
		}

		virtual Eigen::Matrix4d getFramePose(const JointState &q,
									 const std::shared_ptr<Frame> origin,
									 const std::shared_ptr<Frame> destination) const override
		{
			Eigen::Matrix4d frame_pose{};
			frame_pose.setZero();

			frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
			frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
			frame_pose.row(3) << 0, 0, 0, 1;

			return frame_pose;
		}

		virtual Eigen::Vector3d getFootPosition(const JointState &q,
										const std::shared_ptr<Frame> foot) const override
		{
			return this->getFramePosition(q, this->getLink("TRUNK"), foot);
		}

		virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
										   const std::shared_ptr<Frame> foot) const override
		{
			return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
		}

		virtual Eigen::Matrix4d getFootPose(const JointState &q,
									const std::shared_ptr<Frame> foot) const override
		{
			Eigen::Matrix4d foot_pose{};
			foot_pose.setZero();

			foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
			foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
			foot_pose.row(3) << 0, 0, 0, 1;

			return foot_pose;
		}

		virtual Eigen::Vector3d getFootPosition(const JointState &q,
							 const std::shared_ptr<LimbBase> leg) const override
		{
			return this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
		}

		virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
										   const std::shared_ptr<LimbBase> leg) const override
		{
			return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
		}

		virtual Eigen::Matrix4d getFootPose(const JointState &q,
									const std::shared_ptr<LimbBase> leg) const override
		{
			Eigen::Matrix4d foot_pose{};
			foot_pose.setZero();

			foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
			foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
			foot_pose.row(3) << 0, 0, 0, 1;

			return foot_pose;
		}

		virtual void updateLinearJacobian(const JointState &joints_positions,
										  LegDataMap<Jacobian> &robot_jacobian) const override
		{
			joints_positions.size();
			robot_jacobian.getSize();
		}

		virtual void getFootJacobian(const JointState &q,
									 const std::shared_ptr<LimbBase> leg,
									 Jacobian &footJac) const override
		{
			q.size();
			leg->getName();
			footJac.setZero();
		}

		virtual void updateAngularJacobian(const JointState &joints_positions,
								   LegDataMap<Jacobian> &robot_jacobian) const override
		{
			joints_positions.size();
			robot_jacobian.getSize();
			std::cout << "Update Angular Jacobian" << std::endl;
		}

		virtual void updateLinearFootJacobian(const JointState &joints_positions,
                                        const std::shared_ptr<LimbBase> leg,
                                        Jacobian &footJac) const override
		{
			joints_positions.size();
			leg->getName();
			footJac.size();
			std::cout << "Update Linear Foot Jacobian" << std::endl;
		}

    	virtual void updateAngularFootJacobian(const JointState &joints_positions,
                                        const std::shared_ptr<LimbBase> leg,
                                        Jacobian &footJac) const override
		{
			joints_positions.size();
			leg->getName();
			footJac.size();
			std::cout << "Update Angular Foot Jacobian" << std::endl;
		}
        
		virtual void forwardKinematics(const JointState &joint_position,
                               LegDataMap<Eigen::Vector3d> &end_effector_position) const override
		{
			joint_position.size();
			end_effector_position.getSize();
		}

        virtual void forwardKinematics(const JointState &joint_position,
                               const JointState &joint_velocity,
                               LegDataMap<Eigen::Vector3d> &end_effector_position,
                               LegDataMap<Eigen::Vector3d> &end_effector_velocity) const override
		{
			joint_position.size();
			joint_velocity.size();
			end_effector_position.getSize();
			end_effector_velocity.getSize();
		}

		virtual void inverseKinematics(const LegDataMap<Eigen::Vector3d> &end_effector_position,
							   const LegDataMap<Eigen::Vector3d> &end_effector_velocity,
							   const LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
							   JointState &joint_position,
							   JointState &joint_velocity,
							   JointState &joint_acceleration) const override
		{
			end_effector_position.getSize();
			end_effector_velocity.getSize();
			end_effector_acceleration.getSize();
			joint_position.size();
			joint_velocity.size();
			joint_acceleration.size();
		}

		virtual void inverseKinematics(const LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       JointState &joint_position) const override
		{
			end_effector_position.getSize();
			joint_position.size();
		}

		virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
							 const Eigen::Matrix<double, 6, 1> &robot_acceleration,
							 const Eigen::Matrix<double, 6, 1> &gravity_vector,
							 const JointState &joint_position,
							 const JointState &joint_velocity,
							 const JointState &joint_acceleration,
							 Eigen::Matrix<double, 6, 1> &wrench_base,
							 JointState &tau_joints) const override
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

        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                        const JointState &joint_position,
                                        Eigen::Matrix<double, 6, 1> &wrench_base,
                                        JointState &tau_joints) const override
		{
			gravity_vector.size();
			wrench_base.size();
			joint_position.getSize();
			tau_joints.getSize();

			std::cout << "Gravity compensation" << std::endl;
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

		virtual Eigen::Vector3d getWholeBodyCOM(const JointState &joint_position) const override
		{
			joint_position.size();

			return Eigen::Vector3d::Zero();
		}

        virtual Eigen::Vector3d getLegContribution(const JointState &q) const override
		{
			q.size();

			return Eigen::Vector3d::Zero();
		}

		virtual Eigen::Vector3d getCoMFromBase(const JointState & q,
									   				  const Eigen::Vector3d & base_orient,
									   				  const Eigen::Vector3d & base_pos) const override
		{
			q.size();
			base_orient.size();
			base_pos.size();

			return Eigen::Vector3d::Zero();
		}

        virtual Eigen::Vector3d getBaseFromCoM(const JointState & q,
                                       		   		  const Eigen::Vector3d & base_orient,
                                       		   		  const Eigen::Vector3d & CoM) const override
		{
			q.size();
			base_orient.size();
			CoM.size();

			std::cout << "Get the base from COM" << std::endl;

			return Eigen::Vector3d::Zero();
		}

        virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVel(const JointState &q,
                                                               		  const JointState &qd) const override
		{
			q.size();
			qd.size();

			return Eigen::Matrix<double, 6,1>::Zero();
		}

		virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                                 		const Eigen::Matrix3d & R,
                                                                 		const JointState & q) const override
		{
			baseVel.size();
			R.size();
			q.size();
			
			std::cout << "Get whole body COM vel FB - without joint influence" << std::endl;

			return Eigen::Matrix<double, 6, 1>::Zero();	
		};

		virtual Eigen::Matrix<double, 6,1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 		const Eigen::Matrix3d &R,
                                                                 		const Eigen::Vector3d offset_com) const override
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
} // namespace robotlib

std::shared_ptr<robotlib::DummyLeg> makeLeg(const std::string &legName)
{
	std::shared_ptr<robotlib::Joint> haa = std::make_shared<robotlib::Joint>(legName + "_HAA");
	std::shared_ptr<robotlib::Link> assembly = std::make_shared<robotlib::Link>(legName + "_ASSEMBLY");
	std::shared_ptr<robotlib::Joint> hfe = std::make_shared<robotlib::Joint>(legName + "_HFE");

	std::shared_ptr<robotlib::Link> upperleg = std::make_shared<robotlib::Link>(legName + "_UPPERLEG");
	std::shared_ptr<robotlib::Joint> kfe = std::make_shared<robotlib::Joint>(legName + "_KFE");
	std::shared_ptr<robotlib::Link> lowerleg = std::make_shared<robotlib::Link>(legName + "_LOWERLEG");

	return std::make_shared<robotlib::DummyLeg>(legName,
												std::array<std::shared_ptr<robotlib::Joint>, robotlib::NJOINTS>({haa, hfe, kfe}),
												std::array<std::shared_ptr<robotlib::Link>, robotlib::NLINKS>({assembly, upperleg, lowerleg}));
}

extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t()
{
	const robotlib::DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
	
	const std::shared_ptr<robotlib::Trunk> trunk = std::make_shared<robotlib::Trunk>("TRUNK", trunk_dyn_params);
	const std::array<std::shared_ptr<robotlib::LimbBase>, robotlib::NLEGS> legs(
		{makeLeg("LF"),
		 makeLeg("RF"),
		 makeLeg("LH"),
		 makeLeg("RH"),
		 makeLeg("LC"),
		 makeLeg("RC")});
	const std::array<std::shared_ptr<robotlib::LimbBase>, robotlib::NARMS> arms({});

	return std::make_shared<robotlib::DummyHexapod>(trunk, legs, arms);
}

extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot){ robot->getName(); }