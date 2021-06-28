#include "link.hpp"

namespace dls
{
	namespace robotlib
	{
		Link::Link(LimbBase *parent, const std::string &name)
		{
			parent_ = parent;
			name_ = name;
		}

		Link::~Link(){};

		const std::string Link::getName() { return name_; }
		const LimbBase *Link::getParent() const { return parent_; };
	} // namespace robotlib
} // namespace dls