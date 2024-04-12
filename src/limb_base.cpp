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
    LimbBase::LimbBase(const std::string &name) 
        : name_(name){}

    const std::string& LimbBase::getName() const 
    { 
        return name_; 
    }

    bool LimbBase::operator==(const LimbBase& rhs) const
    {
        return this == &rhs;
    }

    std::string LimbBase::type() const
    { 
        return "generic"; 
    };
} // namespace robotlib