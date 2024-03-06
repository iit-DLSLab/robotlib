/**
 * @file leg.cpp
 *
 * @brief Leg class and functions implementation
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

#include "leg.hpp"

namespace robotlib
{
    template <unsigned int NLINKS, unsigned int NJOINTS>
    Leg<NLINKS, NJOINTS >::Leg(const std::string& name,
                               const std::array<std::shared_ptr<Link>, NLINKS>& links,
                               const std::array<std::shared_ptr<Joint>, NJOINTS>& joints)
        : Limb<NLINKS, NJOINTS >(name, links, joints){};

    template <unsigned int NLINKS, unsigned int NJOINTS>
    std::string Leg<NLINKS, NJOINTS >::type() const
    {
        return "arm";
    }
} // namespace robotlib