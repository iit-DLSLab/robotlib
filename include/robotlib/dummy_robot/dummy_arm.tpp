/**
 * @file dummy_arm.tpp
 *
 * @brief Dummy arm class and functions implementation
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

#include "dummy_arm.hpp"

namespace robotlib
{
    template <unsigned int NLINKS, unsigned int NJOINTS>
    DummyArm<NLINKS, NJOINTS>::DummyArm::DummyArm(const std::string& name,
                                                  const std::array<Link, NLINKS>& links,
                                                  const std::array<Joint, NJOINTS>& joints)
        : Arm<NLINKS, NJOINTS>(name, links, joints){}
} // namespace robotlib