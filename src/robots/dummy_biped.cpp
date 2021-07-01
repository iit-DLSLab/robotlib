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
											 std::array<std::shared_ptr<Joint>, 3>({std::make_shared<Joint>(nullptr, name + "_joint1"),
																					std::make_shared<Joint>(nullptr, name + "_joint2"),
																					std::make_shared<Joint>(nullptr, name + "_joint3")}),
											 std::array<std::shared_ptr<Link>, 2>({std::make_shared<Link>(nullptr, name + "_link1"),
																				   std::make_shared<Link>(nullptr, name + "_link2")})) {}
		};

		class DummyBiped : public Robot<6, 4, 2>
		{
		public:
			DummyBiped() : Robot<6, 4, 2>("Biped", std::array<std::shared_ptr<LimbBase>, 2>({std::make_shared<DummyLeg>("leg1"), std::make_shared<DummyLeg>("leg2")})) {}
		};

		extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
		{

			return std::make_shared<DummyBiped>();
		}

		extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
		{
		}

	} //namespace robotlib
} //namespace dls
