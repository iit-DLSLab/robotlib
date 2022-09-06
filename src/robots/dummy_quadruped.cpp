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
						{"RH_KFE", std::make_pair("RH_UPPERLEG", "RH_LOWERLEG")}}),
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
					   {"RH_LOWERLEG", std::make_pair("RH_KFE", "")}}){}

		virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) const
		{
			const std::string joint_name {joint->getName()};
			const std::string child_name {jointMap.find(joint_name)->second.second}; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
			return child_name;
		}

		virtual const std::string jointToParentName(const std::shared_ptr<Joint> joint) const
		{
			const std::string joint_name {joint->getName()};
			const std::string parent_name {jointMap.find(joint_name)->second.first};
			return parent_name;
		}

		virtual const std::string linkToChildName(const std::shared_ptr<Link> link) const
		{
			const std::string link_name {link->getName()};
			const std::string child_name {linkMap.find(link_name)->second.second};
			return child_name;
		}

		virtual const std::string linkToParentName(const std::shared_ptr<Link> link) const
		{
			const std::string link_name {link->getName()};
			const std::string parent_name {linkMap.find(link_name)->second.first};
			return parent_name;
		}

	private:
		const std::map<std::string, std::pair<std::string, std::string>> jointMap;
		const std::map<std::string, std::pair<std::string, std::string>> linkMap;
	};

	const int NJOINTS_TOT {12};
	const int NLINKS_TOT {12};
	const int NLEGS {4};
	const int NARMS {0};
	const int NCHILDRENS {NLEGS};

	class DummyQuadruped : public Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>
	{
	public:
		DummyQuadruped(const std::shared_ptr<Trunk> trunk,
					   const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
					   const std::array<std::shared_ptr<LimbBase>, NARMS> arms)
			: Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>(
				  "Quadruped",
				  trunk,
				  std::make_shared<const Container<std::shared_ptr<LimbBase>, NLEGS>>(legs),
				  std::make_shared<const Container<std::shared_ptr<LimbBase>, NARMS>>(arms))

		{
			std::array<std::shared_ptr<Joint>, NLEGS> children;
			children[0] = getJoint("LF_HAA");
			children[1] = getJoint("RF_HAA");
			children[2] = getJoint("LH_HAA");
			children[3] = getJoint("RH_HAA");

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

		Eigen::Vector3d getFramePosition(const JointState &q,
										 const std::shared_ptr<Frame> origin,
										 const std::shared_ptr<Frame> destination) override
		{
        	q.size();
			origin->getName();
			destination->getName();

			return Eigen::Vector3d().setZero();
		}

		Eigen::Matrix3d getFrameOrientation(const JointState &q,
											const std::shared_ptr<Frame> origin,
											const std::shared_ptr<Frame> destination) override
		{
        	q.size();
			origin->getName();
			destination->getName();

			return Eigen::Matrix3d().setZero();
		}

		Eigen::Matrix4d getFramePose(const JointState &q,
									 const std::shared_ptr<Frame> origin,
									 const std::shared_ptr<Frame> destination) override
		{
			Eigen::Matrix4d frame_pose{};
			frame_pose.setZero();

			frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
			frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
			frame_pose.row(3) << 0, 0, 0, 1;

			return frame_pose;
		}

		Eigen::Vector3d getFootPosition(const JointState &q,
										const std::shared_ptr<Frame> foot) override
		{
			return this->getFramePosition(q, this->getLink("TRUNK"), foot);
		}

		Eigen::Matrix3d getFootOrientation(const JointState &q,
										   const std::shared_ptr<Frame> foot) override
		{
			return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
		}

		Eigen::Matrix4d getFootPose(const JointState &q,
									const std::shared_ptr<Frame> foot) override
		{
			Eigen::Matrix4d foot_pose{};
			foot_pose.setZero();

			foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
			foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
			foot_pose.row(3) << 0, 0, 0, 1;

			return foot_pose;
		}

		void getFootPosition(const JointState &q,
							 const std::shared_ptr<LimbBase> leg,
							 Eigen::Vector3d &footPos)
		{
			footPos = this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
		}

		Eigen::Matrix3d getFootOrientation(const JointState &q,
										   const std::shared_ptr<LimbBase> leg) override
		{
			return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
		}

		Eigen::Matrix4d getFootPose(const JointState &q,
									const std::shared_ptr<LimbBase> leg) override
		{
			Eigen::Matrix4d foot_pose{};
			foot_pose.setZero();

			foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
			foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
			foot_pose.row(3) << 0, 0, 0, 1;

			return foot_pose;
		}

		virtual void getFootJacobian(const JointState &q,
									 const std::shared_ptr<LimbBase> leg,
									 Jacobian &footJac)
		{
			q.size();
			leg->getName();

			footJac.setZero();
		}

		virtual void updateLinearJacobian(const JointState &joints_positions,
										  LegDataMap<Jacobian> &robot_jacobian)
		{
			joints_positions.size();
			robot_jacobian.getSize();
		}

		LegDataMap<std::shared_ptr<Frame>> getFeet() override
		{
			auto feet = this->makeLegDataMap<std::shared_ptr<Frame>>();

			for (auto leg : *(this->getLegs()))
			{
				feet[leg] = std::make_shared<Link>("link");
			}

			return feet;
		}

		void forwardKinematics(const JointState &joint_position,
							   const JointState &joint_velocity,
							   const JointState &joint_acceleration,
							   LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
							   LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
							   LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) override
		{
			joint_position.size();
			joint_velocity.size();
			joint_acceleration.size();
			end_effector_position.getSize();
			end_effector_velocity.getSize();
			end_effector_acceleration.getSize();
		}

		void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
							   const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
							   const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
							   JointState &joint_position,
							   JointState &joint_velocity,
							   JointState &joint_acceleration) override
		{
			end_effector_position.getSize();
			end_effector_velocity.getSize();
			end_effector_acceleration.getSize();
			joint_position.size();
			joint_velocity.size();
			joint_acceleration.size();
		}

		void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
							 const Eigen::Matrix<double, 6, 1> &robot_acceleration,
							 const Eigen::Matrix<double, 6, 1> &gravity_vector,
							 const JointState &joint_position,
							 const JointState &joint_velocity,
							 const JointState &joint_acceleration,
							 Eigen::Matrix<double, 6, 1> &wrench_base, ///output
							 JointState &tau_joints) override		   ///output
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

        void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                        const JointState &joint_position,
                                        Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                        JointState &tau_joints)              ///output
		{
			std::cout << "Gravity compensation" << std::endl;
		}

		double getRobotMass() const override
		{
			return 0.0;
		}

        double getTrunkMass() const override
		{
			return 0.0;
		}

		double getLegsMass() const override
		{
			return 0.0;
		}

		Eigen::Vector3d getRobotCoM() { return Eigen::Vector3d().setZero(); }

		Eigen::Matrix<double, 3, 1> getWholeBodyCOM() override
		{
			return Eigen::Matrix<double, 3, 1>::Zero();
		}

		Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) override
		{
			joint_state.size();

			return Eigen::Matrix<double, 3, 1>::Zero();
		}

        Eigen::Vector3d getLegContribution(const JointState &q) override
		{
			q.size();

			return Eigen::Vector3d::Zero();
		}

		Eigen::Vector3d getCoMFromBase(const JointState & q,
									   const Eigen::Vector3d & base_orient,
									   const Eigen::Vector3d & base_pos) override
		{
			q.size();
			base_orient.size();
			base_pos.size();

			return Eigen::Vector3d::Zero();
		}

        Eigen::Vector3d getBaseFromCoM(const JointState & q,
                                       const Eigen::Vector3d & base_orient,
                                       const Eigen::Vector3d & CoM) override
		{
			q.size();
			base_orient.size();
			CoM.size();

			return Eigen::Vector3d::Zero();
		}

		Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const JointState & q,
                                                       const JointState & qd) override
		{
			q.size();
			qd.size();

			return Eigen::Matrix<double, 6, 1>::Zero();		
		}

        Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> & baseVel,
                                                         const Eigen::Matrix3d & rotationMx,
                                                         const JointState & q,
                                                         const JointState & qd) override
		{
			baseVel.size();
			rotationMx.size();
			q.size();
			qd.size();

			return Eigen::Matrix<double, 6, 1>::Zero();	
		};
		Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const JointState &q,
                                                                 const JointState &qd,
                                                                 const Eigen::Vector3d offset_com)
		{	
			std::cout << "Get whole body COM vel FB, with com offset as input" << std::endl;

			return Eigen::Matrix<double, 6, 1>::Zero();
		}
		void setInvKinTimePeriod(const double& period){std::cout << period << std::endl;};

		virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) { trunk_com.size(); }

		virtual void setTrunkMass(const double& trunk_mass){ std::cout << trunk_mass << std::endl; }
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
		 makeLeg("RH")});
	const std::array<std::shared_ptr<robotlib::LimbBase>, robotlib::NARMS> arms({});

	return std::make_shared<robotlib::DummyQuadruped>(trunk, legs, arms);
}

extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot){ robot->getName(); }