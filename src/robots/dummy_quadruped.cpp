#include "robot.hpp"
#include "leg.hpp"
#include "trunk.hpp"
#include <map>
namespace dls
{
	namespace robotlib
	{
		const int NJOINTS = 3;
		const int NLINKS = 3;

		class DummyLeg : public Leg<NJOINTS, NLINKS>
		{
		public:
			DummyLeg(std::string name, const std::array<std::shared_ptr<dls::robotlib::Joint>, NJOINTS> joints,
					 const std::array<std::shared_ptr<dls::robotlib::Link>, NLINKS> links)
				: Leg<NJOINTS, NLINKS>(name, joints, links),
				  jointMap({//joint name, parent name, child name
							{"LF_haa", std::make_pair("trunk", "LF_assembly")},
							{"LF_hfe", std::make_pair("LF_assembly", "LF_upperleg")},
							{"LF_kfe", std::make_pair("LF_upperleg", "LF_lowerleg")},
							{"RF_haa", std::make_pair("trunk", "RF_assembly")},
							{"RF_hfe", std::make_pair("RF_assembly", "RF_upperleg")},
							{"RF_kfe", std::make_pair("RF_upperleg", "RF_lowerleg")},
							{"LH_haa", std::make_pair("trunk", "LH_assembly")},
							{"LH_hfe", std::make_pair("LH_assembly", "LH_upperleg")},
							{"LH_kfe", std::make_pair("LH_upperleg", "LH_lowerleg")},
							{"RH_haa", std::make_pair("trunk", "RH_assembly")},
							{"RH_hfe", std::make_pair("RH_assembly", "RH_upperleg")},
							{"RH_kfe", std::make_pair("RH_upperleg", "RH_lowerleg")}}),
				  linkMap({//link name, parent name, child name
						   {"LF_assembly", std::make_pair("LF_haa", "LF_hfe")},
						   {"LF_upperleg", std::make_pair("LF_hfe", "LF_kfe")},
						   {"LF_lowerleg", std::make_pair("LF_kfe", "")},
						   {"RF_assembly", std::make_pair("RF_haa", "RF_hfe")},
						   {"RF_upperleg", std::make_pair("RF_hfe", "RF_kfe")},
						   {"RF_lowerleg", std::make_pair("RF_kfe", "")},
						   {"LH_assembly", std::make_pair("LH_haa", "LH_hfe")},
						   {"LH_upperleg", std::make_pair("LH_hfe", "LH_kfe")},
						   {"LH_lowerleg", std::make_pair("LH_kfe", "")},
						   {"RH_assembly", std::make_pair("RH_haa", "RH_hfe")},
						   {"RH_upperleg", std::make_pair("RH_hfe", "RH_kfe")},
						   {"RH_lowerleg", std::make_pair("RH_kfe", "")}}){};

			virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) const
			{
				const std::string joint_name = joint->getName();
				const std::string child_name = jointMap.find(joint_name)->second.second; //find(.)->second get the pair (find(.)->first get the key...I'm sorry but it's the only way to have const member functions using const map variables)
				return child_name;
			};
			virtual const std::string jointToParentName(const std::shared_ptr<Joint> joint) const
			{
				const std::string joint_name = joint->getName();
				const std::string parent_name = jointMap.find(joint_name)->second.first;
				return parent_name;
			};

			virtual const std::string linkToChildName(const std::shared_ptr<Link> link) const
			{
				const std::string link_name = link->getName();
				const std::string child_name = linkMap.find(link_name)->second.second;
				return child_name;
			};
			virtual const std::string linkToParentName(const std::shared_ptr<Link> link) const
			{
				const std::string link_name = link->getName();
				const std::string parent_name = linkMap.find(link_name)->second.first;
				return parent_name;
			};

		private:
			const std::map<std::string, std::pair<std::string, std::string>> jointMap;
			const std::map<std::string, std::pair<std::string, std::string>> linkMap;
		};
		const int NJOINTS_TOT = 12;
		const int NLINKS_TOT = 12;
		const int NLEGS = 4;
		const int NARMS = 0;
		const int NCHILDRENS = NLEGS;
		class DummyQuadruped : public Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>
		{
		public:
			DummyQuadruped(const std::shared_ptr<Trunk> trunk,
						   const std::array<std::shared_ptr<LimbBase>, NLEGS> legs,
						   const std::array<std::shared_ptr<LimbBase>, NARMS> arms)
				: Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>(
					  "Quadruped",
					  trunk,
					  std::make_shared<Container<LimbBase, NLEGS>>(legs),
					  std::make_shared<Container<LimbBase, NARMS>>(arms))

			{
				std::array<std::shared_ptr<Joint>, NLEGS> children;
				children[0] = getJoint("LF_hfe");
				children[1] = getJoint("RF_hfe");
				children[2] = getJoint("LH_hfe");
				children[3] = getJoint("RH_hfe");

				setChildrenOfTrunk(std::make_shared<Container<Joint, NLEGS>>(children));

				setParentOfLink(trunk_, nullptr);

				for (auto leg : *(this->getLegs()))
				{
					for (auto joint : *(leg->getJoints()))
					{
						const std::string child_name = leg->jointToChildName(joint);
						setChildOfJoint(joint, getLink(child_name));
						const std::string parent_name = leg->jointToParentName(joint);
						setParentOfJoint(joint, getLink(parent_name));
					}
				}

				for (auto leg : *(this->getLegs()))
				{
					for (auto link : *(leg->getLinks()))
					{
						const std::string child_name = leg->linkToChildName(link);
						setChildOfLink(link, getJoint(child_name));

						const std::string parent_name = leg->linkToParentName(link);
						setParentOfLink(link, getJoint(parent_name));
					}
				}
			};

			Eigen::Vector3d getFramePosition(const JointState &q,
											 const std::shared_ptr<Frame> origin,
											 const std::shared_ptr<Frame> destination) override
			{
				return Eigen::Vector3d().setZero();
			};

			Eigen::Vector3d getFramePosition(const JointDataMapPair<double> &q,
											 const std::shared_ptr<Frame> origin,
											 const std::shared_ptr<Frame> destination) override
			{
				return Eigen::Vector3d().setZero();
			};

			Eigen::Matrix3d getFrameOrientation(const JointState &q,
												const std::shared_ptr<Frame> origin,
												const std::shared_ptr<Frame> destination) override
			{
				return Eigen::Matrix3d().setZero();
			};

			Eigen::Matrix3d getFrameOrientation(const JointDataMapPair<double> &q,
												const std::shared_ptr<Frame> origin,
												const std::shared_ptr<Frame> destination) override
			{
				return Eigen::Matrix3d().setZero();
			};

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
			};

			Eigen::Matrix4d getFramePose(const JointDataMapPair<double> &q,
										 const std::shared_ptr<Frame> origin,
										 const std::shared_ptr<Frame> destination) override
			{
				Eigen::Matrix4d frame_pose{};
				frame_pose.setZero();

				frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
				frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
				frame_pose.row(3) << 0, 0, 0, 1;

				return frame_pose;
			};

			Eigen::Vector3d getFootPosition(const JointState &q,
											const std::shared_ptr<Frame> foot) override
			{
				return this->getFramePosition(q, this->getLink("trunk"), foot);
			};

			Eigen::Vector3d getFootPosition(const JointDataMapPair<double> &q,
											const std::shared_ptr<Frame> foot) override
			{
				return this->getFramePosition(q, this->getLink("trunk"), foot);
			};

			Eigen::Matrix3d getFootOrientation(const JointState &q,
											   const std::shared_ptr<Frame> foot) override
			{
				return this->getFrameOrientation(q, this->getLink("trunk"), foot);
			};

			Eigen::Matrix3d getFootOrientation(const JointDataMapPair<double> &q,
											   const std::shared_ptr<Frame> foot) override
			{
				return this->getFrameOrientation(q, this->getLink("trunk"), foot);
			};

			Eigen::Matrix4d getFootPose(const JointState &q,
										const std::shared_ptr<Frame> foot) override
			{
				Eigen::Matrix4d foot_pose{};
				foot_pose.setZero();

				foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
				foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
				foot_pose.row(3) << 0, 0, 0, 1;

				return foot_pose;
			};

			Eigen::Matrix4d getFootPose(const JointDataMapPair<double> &q,
										const std::shared_ptr<Frame> foot) override
			{
				Eigen::Matrix4d foot_pose{};
				foot_pose.setZero();

				foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
				foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
				foot_pose.row(3) << 0, 0, 0, 1;

				return foot_pose;
			};

			void getFootPosition(const JointState &q,
								 const std::shared_ptr<LimbBase> leg,
								 Eigen::Vector3d &footPos)
			{
				footPos = this->getFramePosition(q, this->getLink("trunk"), leg->getEndEffector());
			};

			void getFootPosition(const JointDataMapPair<double> &q,
								 const std::shared_ptr<LimbBase> leg,
								 Eigen::Vector3d &footPos)
			{
				footPos = this->getFramePosition(q, this->getLink("trunk"), leg->getEndEffector());
			};

			Eigen::Matrix3d getFootOrientation(const JointState &q,
											   const std::shared_ptr<LimbBase> leg) override
			{
				return this->getFrameOrientation(q, this->getLink("trunk"), leg->getEndEffector());
			};

			Eigen::Matrix3d getFootOrientation(const JointDataMapPair<double> &q,
											   const std::shared_ptr<LimbBase> leg) override
			{
				return this->getFrameOrientation(q, this->getLink("trunk"), leg->getEndEffector());
			};

			Eigen::Matrix4d getFootPose(const JointState &q,
										const std::shared_ptr<LimbBase> leg) override
			{
				Eigen::Matrix4d foot_pose{};
				foot_pose.setZero();

				foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
				foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
				foot_pose.row(3) << 0, 0, 0, 1;

				return foot_pose;
			};

			Eigen::Matrix4d getFootPose(const JointDataMapPair<double> &q,
										const std::shared_ptr<LimbBase> leg) override
			{
				Eigen::Matrix4d foot_pose{};
				foot_pose.setZero();

				foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
				foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
				foot_pose.row(3) << 0, 0, 0, 1;

				return foot_pose;
			};

			virtual void getFootJacobian(const JointState &q,
										 const std::shared_ptr<LimbBase> leg,
										 Jacobian &footJac)
			{
				footJac.setZero();
			};

			virtual void getFootJacobian(const JointDataMapPair<double> &q,
										 const std::shared_ptr<LimbBase> leg,
										 Jacobian &footJac)
			{
				footJac.setZero();
			};

			LegDataMap<std::shared_ptr<Frame>> getFeet() override
			{
				auto feet = this->makeLegDataMap<std::shared_ptr<Frame>>();

				for (auto &foot : feet)
					foot = std::make_shared<Link>("link");

				return feet;
			};

			void forwardKinematics(const JointState &joint_position,
								   const JointState &joint_velocity,
								   const JointState &joint_acceleration,
								   LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_position,
								   LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
								   LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) override
			{
				std::cout << "Forward Kinematics 1" << std::endl;
			};

			void forwardKinematics(const Eigen::Vector3d &joint_position,
								   const Eigen::Vector3d &joint_velocity,
								   const Eigen::Vector3d &joint_acceleration,
								   Eigen::Vector3d &end_effector_position,
								   Eigen::Vector3d &end_effector_velocity,
								   Eigen::Vector3d &end_effector_acceleration,
								   const std::shared_ptr<Frame> end_effector) override
			{
				std::cout << "Forward Kinematics 2" << std::endl;
			};

			void inverseKinematics(const Eigen::Vector3d &end_effector_position,
								   const Eigen::Vector3d &end_effector_velocity,
								   const Eigen::Vector3d &end_effector_acceleration,
								   Eigen::Vector3d &joint_position,
								   Eigen::Vector3d &joint_velocity,
								   Eigen::Vector3d &joint_acceleration,
								   const std::shared_ptr<Frame> end_effector) override
			{
				std::cout << "Inverse Kinematics 1" << std::endl;
			};

			void inverseKinematics(const LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_position,
								   const LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
								   const LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
								   JointState &joint_position,
								   JointState &joint_velocity,
								   JointState &joint_acceleration) override
			{
				std::cout << "Inverse Kinematics 2" << std::endl;
			};
		};
	} //namespace robotlib
} //namespace dls

std::shared_ptr<dls::robotlib::DummyLeg> makeLeg(const std::string &legName)
{
	std::shared_ptr<dls::robotlib::Joint> haa = std::make_shared<dls::robotlib::Joint>(legName + "_haa");
	std::shared_ptr<dls::robotlib::Link> assembly = std::make_shared<dls::robotlib::Link>(legName + "_assembly");
	std::shared_ptr<dls::robotlib::Joint> hfe = std::make_shared<dls::robotlib::Joint>(legName + "_hfe");

	std::shared_ptr<dls::robotlib::Link> upperleg = std::make_shared<dls::robotlib::Link>(legName + "_upperleg");
	std::shared_ptr<dls::robotlib::Joint> kfe = std::make_shared<dls::robotlib::Joint>(legName + "_kfe");
	std::shared_ptr<dls::robotlib::Link> lowerleg = std::make_shared<dls::robotlib::Link>(legName + "_lowerleg");

	return std::make_shared<dls::robotlib::DummyLeg>(legName,
													 std::array<std::shared_ptr<dls::robotlib::Joint>, dls::robotlib::NJOINTS>({haa, hfe, kfe}),
													 std::array<std::shared_ptr<dls::robotlib::Link>, dls::robotlib::NLINKS>({assembly, upperleg, lowerleg}));
}

extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
{
	const std::shared_ptr<dls::robotlib::Trunk> trunk = std::make_shared<dls::robotlib::Trunk>("trunk");
	const std::array<std::shared_ptr<dls::robotlib::LimbBase>, dls::robotlib::NLEGS> legs(
		{makeLeg("LF"),
		 makeLeg("RF"),
		 makeLeg("LH"),
		 makeLeg("RH")});
	const std::array<std::shared_ptr<dls::robotlib::LimbBase>, dls::robotlib::NARMS> arms({});

	return std::make_shared<dls::robotlib::DummyQuadruped>(trunk, legs, arms);
}

extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
{
}