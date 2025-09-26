#include "dummy_robot/dummy_robot_creator.hpp"
#include "dummy_robot/dummy_robot.hpp"
#include "robot.hpp"

namespace robotlib
{
    std::shared_ptr<RobotBase> DummyRobotCreator
        ::createDummyRobot(const std::string& name, const std::vector<std::map<std::string,std::vector<std::string>>>& limbs_string_type)
    {
        const std::string robot_name{name};

        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia

        std::vector<LimbPtr> limbs;
        for (auto const& limb : limbs_string_type)
        {
            limbs.push_back(std::make_shared<Limb>(limb.at("name")[0], limb.at("links"), limb.at("joints"), limb.at("type")[0]));
        }      

        return std::make_shared<DummyRobot>(robot_name, trunk_dyn_params, limbs);
    }
} // namespace robotlib