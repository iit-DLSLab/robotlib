/**
 * @file limb.cpp
 *
 * @brief Limb class and functions implementation
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

#include "limb.hpp"

namespace robotlib
{
    template <unsigned int NLINKS, unsigned int NJOINTS>
    Limb<NLINKS, NJOINTS>::Limb(const std::string& name,
                                const Container<Link, NLINKS>& links,
                                const Container<Joint, NJOINTS>& joints)
          : LimbBase(name)
          , joints_(joints)
          , links_(links)
    {};

    template <unsigned int NLINKS, unsigned int NJOINTS>
    unsigned int Limb<NLINKS, NJOINTS>::getNJoints() const { return joints_.length(); };

    template <unsigned int NLINKS, unsigned int NJOINTS>
    unsigned int Limb<NLINKS, NJOINTS>::getNLinks() const { return links_.length(); };

    template <unsigned int NLINKS, unsigned int NJOINTS>
    const Joint& Limb<NLINKS, NJOINTS>::getJoint(const std::string &name) const
    {
        //Iterate over the array of joints to find the joint
        for (auto& joint : joints_)
        {
            if (joint.getName().compare(name) == 0)
            {
                return joint;
            }
        }
        throw std::range_error("key not found");
    };

    template <unsigned int NLINKS, unsigned int NJOINTS>
    const Link& Limb<NLINKS, NJOINTS>::getLink(const std::string &name) const
    {
        //Iterate over the array of links to find the link
        for (auto& link : links_)
        {
            if (link.getName().compare(name) == 0)
            {
                return link;
            }
        }
        throw std::range_error("key not found");
    };

    template <unsigned int NLINKS, unsigned int NJOINTS>
    const Link& Limb<NLINKS, NJOINTS>::getEndEffector() const
    {
        return *(--links_.end());
    };

    template <unsigned int NLINKS, unsigned int NJOINTS>
    const ContainerBase<Joint> Limb<NLINKS, NJOINTS>::getJoints() const
    { 
        // std::cout << "##### getJoints" << std::endl;

        // for(auto& joint : joints_)
        // {
        //     std::cout << "#####" << joint.getName() << std::endl;
        // }
        return &joints_; 
    }

    template <unsigned int NLINKS, unsigned int NJOINTS>
    ContainerBase<Joint> Limb<NLINKS, NJOINTS>::getJoints()
    { 
        // std::cout << "##### LIMBS getJoints #####" << std::endl;
        // for(auto& joint : joints_)
        // {
        //     std::cout << "#####" << joint.getName() << std::endl;
        // }

        return &joints_; 
    }

    template <unsigned int NLINKS, unsigned int NJOINTS>
    const ContainerBase<Link> Limb<NLINKS, NJOINTS>::getLinks() const
    { 
        return &links_; 
    };

    template <unsigned int NLINKS, unsigned int NJOINTS>
    ContainerBase<Link> Limb<NLINKS, NJOINTS>::getLinks()
    { 
        return &links_; 
    };

} // namespace robotlib