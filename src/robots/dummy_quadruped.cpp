#include "robot.hpp"
#include "limb.hpp"

namespace dls
{
	namespace robotlib
	{
		class DummyLeg : public Limb<3, 2>
		{
		public:
			DummyLeg(std::string name) : Limb<3, 2>(
											 name,
											 std::array<std::shared_ptr<Joint>, 3>({std::make_shared<Joint>(nullptr, nullptr, name + "_joint1"),
																					std::make_shared<Joint>(nullptr, nullptr, name + "_joint2"),
																					std::make_shared<Joint>(nullptr, nullptr, name + "_joint3")}),
											 std::array<std::shared_ptr<Link>, 2>({std::make_shared<Link>(name + "_link1"),
																				   std::make_shared<Link>(name + "_link2")})) {}
		};

		class DummyQuadruped : public Robot<12, 8, 4>
		{
		public:
			DummyQuadruped() : Robot<12, 8, 4>(
								   "Quadruped",
								   std::array<std::shared_ptr<LimbBase>, 4>({std::make_shared<DummyLeg>("leg1"),
																			 std::make_shared<DummyLeg>("leg2"),
																			 std::make_shared<DummyLeg>("leg3"),
																			 std::make_shared<DummyLeg>("leg4")})){};

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
				Eigen::Matrix4d transform{};

				transform.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
				transform.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
				transform.row(3) << 0, 0, 0, 1;

				return transform;
			};

			Link getLink(const std::string &name) override
			{
				return Link("link");
			};

			Joint getJoint(const std::string &name) override
			{
				return Joint(nullptr, nullptr, "joint");
			};

			LegDataMap<std::shared_ptr<Frame>> getFeet()
			{
				return this->makeLegDataMap<std::shared_ptr<Frame>>();
			};
		};

		extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
		{
			return std::make_shared<DummyQuadruped>();
		}

		extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
		{
		}
	} //namespace robotlib
} //namespace dls
