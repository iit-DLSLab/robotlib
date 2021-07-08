#include "link.hpp"

namespace dls
{
	namespace robotlib
	{
		Link::Link(const LimbBase *parentLimb, const std::string &name) : Frame(name), parentLimb_(parentLimb){};

		Link::~Link(){};

		const std::string Link::getName() { return name_; }
		const LimbBase *Link::getParentLimb() const { return parentLimb_; };

	} // namespace robotlib
} // namespace dls