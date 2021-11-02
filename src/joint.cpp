/**
 * @file joint.cpp
 *
 * @brief Joint class and functions implementation
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "joint.hpp"

namespace dls
{
	namespace robotlib
	{
		Joint::Joint(const std::string &name) : Frame(name){};

		Joint::~Joint(){};

		const std::string Joint::getName() const { return name_; }
		const std::shared_ptr<Link> Joint::getParent() const { return parent_; }
		const std::shared_ptr<Link> Joint::getChild() const { return child_; }

		void Joint::setParent(const std::shared_ptr<Link> parent) { parent_ = parent; }
		void Joint::setChild(const std::shared_ptr<Link> child) { child_ = child; }
	} // namespace robotlib
} // namespace dls