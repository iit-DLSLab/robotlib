/**
 * @file dummy_leg.tpp
 *
 * @brief DummyLeg class and functions implementation
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

#include "dummy_leg.hpp"

namespace robotlib
{
    template <unsigned int NLINKS, unsigned int NJOINTS>
    DummyLeg<NLINKS, NJOINTS>::DummyLeg::DummyLeg(const std::string& name,
                                                  const Container<Link, NLINKS>& links,
                                                  const Container<Joint, NJOINTS>& joints)
        : Leg<NLINKS, NJOINTS>(name, links, joints){}
} // namespace robotlib