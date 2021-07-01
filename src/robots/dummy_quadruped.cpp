#include "robot.hpp"
#include "forward_kinematics_base.hpp"
#include "limb.hpp"

namespace dls {
namespace robotlib {

	class DummyLeg : public Limb<3,2> {
	public:
		DummyLeg(std::string name) : Limb<3,2>(
			name,
			std::array<std::shared_ptr<Joint>,3>({
				std::make_shared<Joint>(nullptr,name+"_joint1"),
				std::make_shared<Joint>(nullptr,name+"_joint2"),
				std::make_shared<Joint>(nullptr,name+"_joint3")
			}),
			std::array<std::shared_ptr<Link>,2>({
				std::make_shared<Link>(nullptr,name+"_link1"),
				std::make_shared<Link>(nullptr,name+"_link2")
			})
		) {}
	};
	
	class DummyQuadruped : public Robot<12,8,4> {
	public:
		DummyQuadruped() : Robot<12,8,4>(
			"Quadruped",
			std::array<std::shared_ptr<LimbBase>,4>({
				std::make_shared<DummyLeg>("leg1"),
				std::make_shared<DummyLeg>("leg2"),
				std::make_shared<DummyLeg>("leg3"),
				std::make_shared<DummyLeg>("leg4")
			})
		) {}
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
