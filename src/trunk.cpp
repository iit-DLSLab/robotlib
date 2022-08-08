/**
 * @file trunk.cpp
 *
 * @brief Trunk class and functions implementation
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "trunk.hpp"

namespace robotlib
{

	Trunk::Trunk(const std::string &name) : Link(name), dynamic_parameters_(DynParams(Eigen::Vector3d::Zero(), 0, Eigen::Matrix3d::Zero())){}

	Trunk::Trunk(const std::string &name, const DynParams &dynamic_parameters) : Link(name), dynamic_parameters_(dynamic_parameters){}

	Trunk::~Trunk(){}

	const std::string Trunk::getName() const { return name_; }

	const Eigen::Vector3d & Trunk::getCoM() const {return dynamic_parameters_.getCoM(); }

	const double Trunk::getMass() const { return dynamic_parameters_.getMass(); }
	
	const Eigen::Matrix3d &Trunk::getInertia() const {return dynamic_parameters_.getInertia(); }

	const DynParams &Trunk::getDynParams() const { return dynamic_parameters_; }

} // namespace robotlib