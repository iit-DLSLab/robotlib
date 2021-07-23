#include "robot.hpp"
#include "limb.hpp"
#include <map>
namespace dls
{
	namespace robotlib
	{
		const int NJOINTS = 3;
		const int NLINKS = 3;

		class DummyLeg : public Limb<NJOINTS, NLINKS>
		{
		public:
			DummyLeg(std::string name, const std::array<std::shared_ptr<dls::robotlib::Joint>, NJOINTS> joints,
					 const std::array<std::shared_ptr<dls::robotlib::Link>, NLINKS> links) : Limb<NJOINTS, NLINKS>(name, joints, links)
			{
				// Iterate over the joints and set the corresponding child
				for (auto joint : *joints_)
				{
					const std::string child_name = jointToChildName(joint);
					setChildOfJoint(joint, getLinkFromName(child_name));
				}
			}
			virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint)
			{
				const std::string jointName = joint->getName();
				const std::string childName = jointMap[jointName].second;
				return childName;
			};
			virtual const std::shared_ptr<Link> getLinkFromName(const std::string &name)
			{
				//Iterate over the array of links to find the link
				for (auto link : links_)
				{
					if (link->getName().compare(name) == 0)
					{
						return link;
					}
				}

				std::cout << "LINK NOT FOUD FROM THE INPUT NAME " << name << std::endl;
				return std::shared_ptr<Link>(nullptr);
			};
			virtual const std::shared_ptr<Joint> getJointFromName(const std::string &name)
			{
				//Iterate over the array of links to find the link
				for (auto joint : *joints_)
				{
					if (joint->getName().compare(name))
					{
						return joint;
					}
				}

				std::cout << "JOINT NOT FOUD FROM THE INPUT NAME " << name << std::endl;
				return std::shared_ptr<Joint>(nullptr);
			};

			std::map<std::string, std::pair<std::string, std::string>> jointMap{
				//joint name, parent name, child name
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
				{"RH_kfe", std::make_pair("RH_upperleg", "RH_lowerleg")},
			};
		};

		const int NJOINTS_TOT = 12;
		const int NLINKS_TOT = 8;
		const int NLEGS = 4;
		class DummyQuadruped : public Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS>
		{
		public:
			DummyQuadruped(const std::array<std::shared_ptr<dls::robotlib::LimbBase>, NLEGS> legs)
				: Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS>(
					  "Quadruped",
					  std::make_shared<dls::robotlib::Trunk>("trunk"),
					  legs){};

			Eigen::Vector3d getFramePosition(const JointState &q,
											 const Frame &origin,
											 const Frame &destination) override
			{
				return Eigen::Vector3d().setZero();
			};

			Eigen::Matrix3d getFrameOrientation(const JointState &q,
												const Frame &origin,
												const Frame &destination) override
			{
				return Eigen::Matrix3d().setZero();
			};

			Eigen::Matrix4d getFramePose(const JointState &q,
										 const Frame &origin,
										 const Frame &destination) override
			{
				Eigen::Matrix4d frame_pose{};
				frame_pose.setZero();

				frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
				frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
				frame_pose.row(3) << 0, 0, 0, 1;

				return frame_pose;
			};

			Eigen::Vector3d getFootPosition(const JointState &q,
											const Frame &foot) override
			{
				return this->getFramePosition(q, this->getLink("TRUNK"), foot);
			};

			Eigen::Matrix3d getFootOrientation(const JointState &q,
											   const Frame &foot) override
			{
				return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
			};

			Eigen::Matrix4d getFootPose(const JointState &q,
										const Frame &foot) override
			{
				Eigen::Matrix4d foot_pose{};
				foot_pose.setZero();

				foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
				foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
				foot_pose.row(3) << 0, 0, 0, 1;

				return foot_pose;
			};

			// TO DO: mapping between name and id
			Link getLink(const std::string &name) override
			{
				return *std::static_pointer_cast<Link>(this->getLeg(0)->getLink(0));
			};

			Joint getJoint(const std::string &name) override
			{
				return Joint("joint", nullptr);
			};

			LegDataMap<std::shared_ptr<Frame>> getFeet()
			{
				auto feet = this->makeLegDataMap<std::shared_ptr<Frame>>();

				for (auto &foot : feet)
					foot = std::make_shared<Link>("link");

				return feet;
			};

			//JointState getJointsConfiguration(const Frame &end_effector,
			//								  const Eigen::Matrix4d &end_effector_pose,
			//								  JointState &q) override{};
			//
			//JointState getJointsVelocities(const Frame &end_effector,
			//							   const Eigen::Matrix4d &end_effector_velocity,
			//							   const JointState &q,
			//							   JointState &q_d) override{};
		};

	} //namespace robotlib
} //namespace dls

std::shared_ptr<dls::robotlib::DummyLeg> makeLeg(const std::string &legName)
{
	//std::shared_ptr<dls::robotlib::Joint> haa = std::make_shared<dls::robotlib::Joint>(legName + "_haa", trunk);
	std::shared_ptr<dls::robotlib::Link> assembly = std::make_shared<dls::robotlib::Link>(legName + "_assembly");
	std::shared_ptr<dls::robotlib::Joint> hfe = std::make_shared<dls::robotlib::Joint>(legName + "_hfe", assembly);

	std::shared_ptr<dls::robotlib::Link> upperleg = std::make_shared<dls::robotlib::Link>(legName + "_upperleg");
	std::shared_ptr<dls::robotlib::Joint> kfe = std::make_shared<dls::robotlib::Joint>(legName + "_kfe", upperleg);
	std::shared_ptr<dls::robotlib::Link> lowerleg = std::make_shared<dls::robotlib::Link>(legName + "_lowerleg");

	return std::make_shared<dls::robotlib::DummyLeg>(legName,
													 std::array<std::shared_ptr<dls::robotlib::Joint>, dls::robotlib::NJOINTS>({hfe, hfe, kfe}),
													 std::array<std::shared_ptr<dls::robotlib::Link>, dls::robotlib::NLINKS>({assembly, upperleg, lowerleg}));
}

extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
{
	const std::array<std::shared_ptr<dls::robotlib::LimbBase>, dls::robotlib::NLEGS> legs({makeLeg("LF"), makeLeg("RF"), makeLeg("LH"), makeLeg("RH")});
	return std::make_shared<dls::robotlib::DummyQuadruped>(legs);
}

extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
{
}