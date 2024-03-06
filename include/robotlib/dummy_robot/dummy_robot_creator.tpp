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

        std::vector<std::shared_ptr<LimbBase>> limbs_vec;
        std::vector<std::shared_ptr<Joint>> joints;
        std::vector<std::shared_ptr<Link>> links;

        for(unsigned int i{0}; i < NLEGS; i++)
        {
            joints.clear();
            for(unsigned int j{0}; j < NJOINTSLEG; j++)
            {
                joints.push_back(std::make_shared<Joint>(components_names[(i*NJOINTSLEG)+(j+2+NLEGS)]));
            }

            links.clear();
            for(unsigned int j{0}; j < NLINKSLEG; j++)
            {
                links.push_back(std::make_shared<Link>(components_names[(i*NLINKSLEG)+(j+2+NLEGS+(NJOINTSLEG*NLEGS))]));
            }

            limbs_vec.push_back(std::make_shared<DummyLeg<NLINKSLEG, NJOINTSLEG>>(components_names[i+2], to_array<NLINKSLEG>(links.begin()), to_array<NJOINTSLEG>(joints.begin()))); 
        }

        for(unsigned int i{0}; i < NARMS; i++)
        {
            joints.clear();
            for(unsigned int j{0}; j < NJOINTSARM; j++)
            {
                joints.push_back(std::make_shared<Joint>(components_names[(i*NJOINTSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS)]));
            }

            links.clear();
            for(unsigned int j{0}; j<NLINKSARM; j++)
            {
                links.push_back(std::make_shared<Link>(components_names[(i*NLINKSARM)+(j+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)+NARMS+(NJOINTSARM*NARMS))]));
            }

            limbs_vec.push_back(std::make_shared<DummyArm<NLINKSARM, NJOINTSARM>>(components_names[i+2+NLEGS+(NJOINTSLEG*NLEGS)+(NLINKSLEG*NLEGS)], to_array<NLINKSARM>(links.begin()), to_array<NJOINTSARM>(joints.begin()))); 
        }

        return std::make_shared<DummyRobot<NARMS+NLEGS, NLINKSARM+NLINKSLEG, NJOINTSARM+NJOINTSLEG>>(name, trunk, to_array<NARMS+NLEGS>(limbs_vec.begin()));
    }
} // namespace robotlib