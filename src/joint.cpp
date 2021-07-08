#include "joint.hpp"

namespace dls
{
	namespace robotlib
	{
		Joint::Joint(Link *parent, Link *child, const std::string &name) : Frame(name), parent_(parent){};

		Joint::~Joint(){};

		const std::string Joint::getName() { return name_; }
		const Link *Joint::getParent() const { return parent_; };
		const Link *Joint::getChild() const { return child_; };
	} // namespace robotlib
} // namespace dls