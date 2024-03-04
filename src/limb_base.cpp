/**
 * @file limb_base.cpp
 *
 * @brief LiimbBase class and functions implementation
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

#include "limb_base.hpp"

namespace robotlib
{
    LimbBase::LimbBase(const std::string &name) : name_(name){}

    const std::string& LimbBase::getName() const { return name_; }

    unsigned int LimbBase::getNJoints() const
    {
        throw std::range_error("You need to implement getNJoints from LimbBase");
    }

    unsigned int LimbBase::getNLinks() const
    {
        throw std::range_error("You need to implement getNLinks from LimbBase");
    }

    const Joint& LimbBase::getJoint(const std::string &name) const
    {
        throw std::range_error("You need to implement getJoint from LimbBase");
    }

    const Link& LimbBase::getLink(const std::string &name) const
    {
        throw std::range_error("You need to implement getLink from LimbBase");
    }

    const Link& LimbBase::getEndEffector() const
    {
        throw std::range_error("You need to implement getEndEffector from LimbBase");
    }

    const ContainerBase<Joint>& LimbBase::getJoints() const
    {
        throw std::range_error("You need to implement getJoints from LimbBase");
    }

    ContainerBase<Joint>& LimbBase::getJoints()
    {
        throw std::range_error("You need to implement protected getJoints from LimbBase");
    }

    const ContainerBase<Link>& LimbBase::getLinks() const
    {
        throw std::range_error("You need to implement getLinks from LimbBase");
    }

    ContainerBase<Link>& LimbBase::getLinks()
    {
        throw std::range_error("You need to implement protected getLinks from LimbBase");
    }

    bool LimbBase::operator==(const LimbBase& rhs) const
    {
        return this->name_.compare(rhs.name_) == 0;
    }

    std::string LimbBase::type() const
    { 
        return "generic"; 
    };
} // namespace robotlib