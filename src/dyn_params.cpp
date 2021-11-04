/**
 * @file dyn_params.cpp
 *
 * @brief DynParams class and functions implementation
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "dyn_params.hpp"

namespace robotlib
{
	DynParams::DynParams(const Eigen::Vector3d &com,
						 const double mass,
						 const Eigen::Matrix3d &inertia)
		: com_(com),
		  mass_(mass),
		  inertia_(inertia){};

	DynParams::~DynParams(){};

	const Eigen::Vector3d &DynParams::getCoM() { return com_; };
	const double DynParams::getMass() { return mass_; };
	const Eigen::Matrix3d &DynParams::getInertia() { return inertia_; };
} // namespace robotlib