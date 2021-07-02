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

		class DummyHexapod : public Robot<18, 12, 6>
		{
		public:
			DummyHexapod() : Robot<18, 12, 6>(
								 "Hexapod",
								 std::array<std::shared_ptr<LimbBase>, 6>({std::make_shared<DummyLeg>("leg1"),
																		   std::make_shared<DummyLeg>("leg2"),
																		   std::make_shared<DummyLeg>("leg3"),
																		   std::make_shared<DummyLeg>("leg4"),
																		   std::make_shared<DummyLeg>("leg5"),
																		   std::make_shared<DummyLeg>("leg6")})){};

			Eigen::Matrix4d getTransform(JointState &q, Frame &origin, Frame &destination) override
			{
				return Eigen::Matrix4d().setZero();
			};
		};

		extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
		{
			return std::make_shared<DummyHexapod>();
		}

		extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
		{
		}
	} //namespace robotlib
} //namespace dls
