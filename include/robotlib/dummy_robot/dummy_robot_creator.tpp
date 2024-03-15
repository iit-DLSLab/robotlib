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

    template<typename T, typename Iter, std::size_t... Is>
    constexpr auto to_array(Iter& iter, std::index_sequence<Is...>)
    -> std::array<T, sizeof...(Is)> {
        return {{ ((void)Is, *iter++)... }};
    }

    template<std::size_t N, typename Iter,
            typename T = typename std::iterator_traits<Iter>::value_type>
    constexpr auto to_array(Iter iter)
    -> std::array<T, N> {
        return to_array<T>(iter, std::make_index_sequence<N>{});
    }

    template <unsigned int NLEGS, unsigned int NLINKSLEG, unsigned int NJOINTSLEG, unsigned int NARMS, unsigned int NLINKSARM, unsigned int NJOINTSARM>
    std::shared_ptr<RobotBase> DummyRobotCreator<NLEGS, NLINKSLEG, NJOINTSLEG, NARMS, NLINKSARM, NJOINTSARM>
        ::createDummyRobot(const std::array<std::string, (2 + NLEGS + NLEGS*NLINKSLEG + NLEGS*NJOINTSLEG + NARMS + NARMS*NLINKSARM + NARMS*NJOINTSARM)>& components_names)
    {
        const std::string name{components_names[0]};

        const DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
         
        Trunk trunk(components_names[1], trunk_dyn_params);

        Container<LimbBase, NLEGS+NARMS> limbs;
        
        Container<Link, NLEGS*NLINKSLEG> leg_links;
        Container<Joint, NLEGS*NJOINTSLEG> leg_joints;

        for(unsigned int i{0}; i < NLEGS; i++)
        {
            for(unsigned int j{0}; j < NLINKSLEG; j++)
            {
                leg_links.at(i*NLINKSLEG + j) = new Link(components_names[(i*NLINKSLEG)+(j+2+NLEGS+(NJOINTSLEG*NLEGS))]);
            }

            for(unsigned int j{0}; j < NJOINTSLEG; j++)
            {
                leg_joints.at(i*NJOINTSLEG + j) = new Joint(components_names[(i*NJOINTSLEG)+(j+2+NLEGS)]);
            }

            limbs.at(i) = new DummyLeg<NLINKSLEG, NJOINTSLEG>(components_names[i+2], leg_links, leg_joints); 
        }

        Container<Link, NARMS*NLINKSARM> arm_links;
        Container<Joint, NARMS*NJOINTSARM> arm_joints;

        for(unsigned int i{0}; i < NARMS; i++)
        {
            for(unsigned int j{0}; j < NLINKSARM; j++)
            {
                arm_links.at(i*NLINKSARM + j) = new Link(components_names[(i*NLINKSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS+(NJOINTSARM*NARMS))]);
            }
            
            for(unsigned int j{0}; j < NJOINTSARM; j++)
            {
                arm_joints.at(i*NJOINTSARM + j) = new Joint(components_names[(i*NJOINTSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS)]);
            }

            limbs.at(NLEGS + i) = new DummyArm<NLINKSARM, NJOINTSARM>(components_names[i+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)], arm_links, arm_joints); 
        }

        return std::make_shared<DummyRobot<NARMS+NLEGS, NLINKSARM+NLINKSLEG, NJOINTSARM+NJOINTSLEG>>(name, trunk, limbs);
    }
} // namespace robotlib