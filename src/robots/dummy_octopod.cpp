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
																		   std::make_shared<DummyLeg>("leg8")})) {}
		};

		extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
		{

			return std::make_shared<DummyOctopod>();
		}

		extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
		{
		}

	} //namespace robotlib
} //namespace dls
