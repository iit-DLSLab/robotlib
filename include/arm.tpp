/**
 * @file arm.cpp
 *
 * @brief Arm class and functions implementation
 *
 * @authors Authors in alphabetic order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "arm.hpp"

namespace robotlib
{
    template <unsigned int NJOINTS, unsigned int NLINKS>
    Arm<NJOINTS, NLINKS>::Arm(const std::string &name,
                              const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
                              const std::array<std::shared_ptr<Link>, NLINKS> &links)
        : Limb<NJOINTS, NLINKS>(name, joints, links){};

    template <unsigned int NJOINTS, unsigned int NLINKS>
    Arm<NJOINTS, NLINKS>::~Arm(){};
} // namespace robotlib