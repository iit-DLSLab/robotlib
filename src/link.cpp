#include "link.hpp"

namespace dls
{
	namespace robotlib
	{
		Link::Link(const std::string &name)
		{
			name_ = name;
		}

		Link::~Link(){};

		const std::string Link::getName() { return name_; }
	} // namespace robotlib
} // namespace dls