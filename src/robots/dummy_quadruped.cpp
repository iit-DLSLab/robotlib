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

			Eigen::Matrix4d getTransform(JointState &q, Frame &origin, Frame &destination) override
			{
				return Eigen::Matrix4d().setZero();
			};

			Link getLink(std::string &name) override
			{
				return Link("link");
			};

			Joint getJoint(std::string &name) override
			{
				return Joint(nullptr, nullptr, "joint");
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
