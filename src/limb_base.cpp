/**
 * @file limb_base.cpp
 *
 * @brief LiimbBase class and functions implementation
 *
 * @authors Authors in alphabetic order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "limb_base.hpp"

namespace robotlib
{
    LimbBase::LimbBase(const std::string &name) : name_(name){}

    LimbBase::~LimbBase(){}

    std::string LimbBase::getName() const { return name_; }
} // namespace robotlib