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

namespace dls
{
	namespace robotlib
	{
		Trunk::Trunk(const std::string &name) : Link(name){};
		Trunk::~Trunk(){};

		const std::string Trunk::getName() const { return name_; }
	} // namespace robotlib
} // namespace dls