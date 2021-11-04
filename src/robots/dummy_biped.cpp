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

	class DummyBiped : public Robot<6, 4, 2>
	{
	public:
		DummyBiped() : Robot<6, 4, 2>("Biped", std::array<std::shared_ptr<LimbBase>, 2>({std::make_shared<DummyLeg>("leg1"), std::make_shared<DummyLeg>("leg2")})){};

		Eigen::Matrix4d getTransform(JointState &q, Frame &origin, Frame &destination) override
		{
			return Eigen::Matrix4d().setZero();
		};
	};

	extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t()
	{
		return std::make_shared<DummyBiped>();
	}

	extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot)
	{
	}
} // namespace robotlib