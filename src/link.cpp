/**
 * @file link.cpp
 *
 * @brief Link class and functions implementation
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "link.hpp"

namespace robotlib
{
	Link::Link(const std::string &name) : Frame(name){};

	Link::~Link(){};

	const std::string Link::getName() const { return name_; }
	const std::shared_ptr<Joint> Link::getParent() const { return parent_; }
	const std::shared_ptr<Joint> Link::getChild() const { return child_; }

	void Link::setParent(const std::shared_ptr<Joint> parent) { parent_ = parent; }
	void Link::setChild(const std::shared_ptr<Joint> child) { child_ = child; }
	void Link::setChildren(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children) { children_ = children; }
} // namespace robotlib