/**
 * @file dummy_robot_creator.tpp
 *
 * @brief DummyRobotCreator class and functions implementation
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "dummy_robot_creator.hpp"

namespace robotlib
{
    template <unsigned int NLEGS, unsigned int NLJLEG, unsigned int NARMS, unsigned int NLJARM>
    std::shared_ptr<RobotBase> DummyRobotCreator<NLEGS, NLJLEG, NARMS, NLJARM>
        ::createDummyRobot(const std::array<std::string, (1 + NLEGS + 2*NLEGS*NLJLEG + NARMS + 2*NARMS*NLJARM)>& components_names)
    {
        const std::string robot_name{components_names[0]};

        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
         
        Container<robotlib::LimbBase, NLEGS+NARMS> limbs;
        
        std::array<std::array<std::shared_ptr<Link>, NLJLEG>,  NLEGS> leg_links;
        std::array<std::array<std::shared_ptr<Joint>, NLJLEG>,  NLEGS> leg_joints;

        for(unsigned int i{0}; i < NLEGS; i++)
        {   
            std::shared_ptr<Link> parent_link = nullptr;
            for(unsigned int j{0}; j < NLJLEG; j++)
            {
                leg_joints[i][j] = std::make_shared<Joint>(components_names[(i*NLJLEG)+(j + 1 + NLEGS)], parent_link.get());

                leg_links[i][j] = std::make_shared<Link>(components_names[(i*NLJLEG)+(j + 1 + NLEGS + NLJLEG*NLEGS)], leg_joints[i][j].get());
                parent_link = leg_links[i][j];
            }

            limbs.at(i) = std::make_shared<DummyLeg<NLJLEG>>(components_names[i+1], leg_links[i], leg_joints[i]); 
        }

        std::array<std::array<std::shared_ptr<Link>, NLJARM>, NARMS> arm_links;
        std::array<std::array<std::shared_ptr<Joint>, NLJARM>, NARMS> arm_joints;

        for(unsigned int i{0}; i < NARMS; i++)
        {
            std::shared_ptr<Link> parent_link = nullptr;
            for(unsigned int j{0}; j < NLJARM; j++)
            {
                arm_joints[i][j] = std::make_shared<Joint>(components_names[(i*NLJARM) + (j + 1 + NLEGS + 2*NLJLEG*NLEGS + NARMS)], parent_link.get());

                arm_links[i][j] = std::make_shared<Link>(components_names[(i*NLJARM) + (j + 1 + NLEGS + 2*NLJLEG*NLEGS + NARMS + NLJARM*NARMS)], arm_joints[i][j].get());
                parent_link = arm_links[i][j];                
            }

            limbs.at(NLEGS + i) = std::make_shared<DummyArm<NLJARM>>(components_names[i + 1 + NLEGS + 2*NLJLEG*NLEGS], arm_links[i], arm_joints[i]);
        }

        return std::make_shared<DummyRobot<NARMS+NLEGS, NARMS*NLJARM+NLEGS*NLJLEG, NARMS*NLJARM+NLEGS*NLJLEG>>(robot_name, trunk_dyn_params, limbs);
    }
} // namespace robotlib