#include "robot.hpp"
#include "limb.hpp"

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

	class DummyOctopod : public Robot<24, 16, 8>
	{
	public:
		DummyOctopod() : Robot<24, 16, 8>(
							 "Octopod",
							 std::array<std::shared_ptr<LimbBase>, 8>({std::make_shared<DummyLeg>("leg1"),
																	   std::make_shared<DummyLeg>("leg2"),
																	   std::make_shared<DummyLeg>("leg3"),
																	   std::make_shared<DummyLeg>("leg4"),
																	   std::make_shared<DummyLeg>("leg5"),
																	   std::make_shared<DummyLeg>("leg6"),
																	   std::make_shared<DummyLeg>("leg7"),
																	   std::make_shared<DummyLeg>("leg8")})){};

		Eigen::Matrix4d getTransform(JointState &q, Frame &origin, Frame &destination) override
		{
			return Eigen::Matrix4d().setZero();
		};
	};

	extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t()
	{
		return std::make_shared<DummyOctopod>();
	}

	extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot)
	{
	}
} // namespace robotlib