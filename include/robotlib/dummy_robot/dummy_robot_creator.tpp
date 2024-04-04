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

#include <span>

namespace robotlib
{
    template <unsigned int NLEGS, unsigned int NLJLEG, unsigned int NARMS, unsigned int NLJARM>
    std::shared_ptr<RobotBase> DummyRobotCreator<NLEGS, NLJLEG, NARMS, NLJARM>
        ::createDummyRobot(const std::array<std::string, (2 + NLEGS + 2*NLEGS*NLJLEG + NARMS + 2*NARMS*NLJARM)>& components_names)
    {
        const std::string name{components_names[0]};

        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
         
        Trunk trunk(components_names[1], trunk_dyn_params);

        Container<robotlib::LimbBase, NLEGS+NARMS> limbs;
        
        std::array<std::array<std::shared_ptr<Link>, NLJLEG>,  NLEGS> leg_links;
        std::array<std::array<std::shared_ptr<Joint>, NLJLEG>,  NLEGS> leg_joints;

        for(unsigned int i{0}; i < NLEGS; i++)
        {   
            Link* parent_link = &trunk;
            for(unsigned int j{0}; j < NLJLEG; j++)
            {
                leg_joints[i][j] = std::make_shared<Joint>(components_names[(i*NLJLEG)+(j+2+NLEGS)], parent_link);

                leg_links[i][j] = std::make_shared<Link>(components_names[(i*NLJLEG)+(j+2+NLEGS+(NLJLEG*NLEGS))], leg_joints[i][j]);
                parent_link = &leg_links[i][j];
            }

            limbs.at(i) = std::make_shared<DummyLeg<NLJLEG>>(components_names[i+2], leg_links[i], leg_joints[i]); 
        }

        // std::array<std::array<std::shared_ptr<Link>, NLINKSARM>, NARMS> arm_links;
        // std::array<std::array<std::shared_ptr<Joint>, NJOINTSARM>, NARMS> arm_joints;

        // for(unsigned int i{0}; i < NARMS; i++)
        // {
        //     for(unsigned int j{0}; j < NLINKSARM; j++)
        //     {
        //         arm_links[i][j] = std::make_shared<Link>(components_names[(i*NLINKSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS+(NJOINTSARM*NARMS))]);
        //     }
            
        //     for(unsigned int j{0}; j < NJOINTSARM; j++)
        //     {
        //         arm_joints[i][j] = std::make_shared<Joint>(components_names[(i*NJOINTSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS)]);
        //     }

        //     limbs.at(NLEGS + i) = std::make_shared<DummyArm<NLINKSARM, NJOINTSARM>>(components_names[i+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)], arm_links[i], arm_joints[i]); 
        // }
   
        return std::make_shared<DummyRobot<NARMS+NLEGS, NLJARM+NLJLEG, NLJARM+NLJLEG>>(name, trunk, limbs);
    }
} // namespace robotlib