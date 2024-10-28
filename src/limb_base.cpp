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
    LimbBase::LimbBase(const std::string &name) : name_(name), id(-1){} //-1 means not set

    LimbBase::~LimbBase(){}

    std::string LimbBase::getName() const { return name_; }

    void LimbBase::setID(int id) { this->id = id; }

    int LimbBase::getID() const { return id; }
} // namespace robotlib