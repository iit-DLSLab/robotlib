/**
 * @file leg.cpp
 *
 * @brief Leg class and functions implementation
 *
 * @authors Authors in alphabetic order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "leg.hpp"

namespace robotlib
{
    template <unsigned int NJOINTS, unsigned int NLINKS>
    Leg<NJOINTS, NLINKS>::Leg(const std::string &name,
                              const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
                              const std::array<std::shared_ptr<Link>, NLINKS> &links)
        : Limb<NJOINTS, NLINKS>(name, joints, links){};

    template <unsigned int NJOINTS, unsigned int NLINKS>
    Leg<NJOINTS, NLINKS>::~Leg(){};
} // namespace robotlib