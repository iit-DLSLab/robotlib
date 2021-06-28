#include "joint.hpp"

namespace dls
{
	namespace robotlib
	{
		Joint::Joint(LimbBase *parent, const std::string &name)
		{
			parent_ = parent;
			name_ = name;
		};

		Joint::~Joint(){};

		const std::string Joint::getName() { return name_; }
		const LimbBase *Joint::getParent() const { return parent_; };
	} // namespace robotlib
} // namespace dls