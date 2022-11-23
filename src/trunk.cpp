/**
 * @file trunk.cpp
 *
 * @brief Trunk class and functions implementation
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "trunk.hpp"

namespace robotlib
{

	Trunk::Trunk(const std::string &name) : Link(name), dynamic_parameters_(DynParams(Eigen::Vector3d::Zero(), 0, Eigen::Matrix3d::Zero())){}

	Trunk::Trunk(const std::string &name, const DynParams &dynamic_parameters) : Link(name), dynamic_parameters_(dynamic_parameters){}

	Trunk::~Trunk(){}

	std::string Trunk::getName() const { return name_; }

	Eigen::Vector3d Trunk::getCoM() const {return dynamic_parameters_.getCoM(); }

	double Trunk::getMass() const { return dynamic_parameters_.getMass(); }
	
	Eigen::Matrix3d Trunk::getInertia() const {return dynamic_parameters_.getInertia(); }

	DynParams Trunk::getDynParams() const { return dynamic_parameters_; }

} // namespace robotlib