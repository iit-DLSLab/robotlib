/**
 * @file arm.cpp
 *
 * @brief Arm class and functions implementation
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

#include "arm.hpp"

namespace robotlib
{
    template <unsigned int NLINKS, unsigned int NJOINTS>
    Arm<NLINKS, NJOINTS>::Arm(const std::string &name,
                              const std::array<std::shared_ptr<Link>, NLINKS>& links,
                              const std::array<std::shared_ptr<Joint>, NJOINTS>& joints)
        : Limb<NLINKS, NJOINTS>(name, links, joints){};

    template <unsigned int NLINKS, unsigned int NJOINTS>
    Arm<NLINKS, NJOINTS>::~Arm(){};

    template <unsigned int NLINKS, unsigned int NJOINTS>
    std::string Arm<NLINKS, NJOINTS>::type() const
    {
        return "arm";
    }
} // namespace robotlib