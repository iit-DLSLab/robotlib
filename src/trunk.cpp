#include "trunk.hpp"

namespace dls
{
	namespace robotlib
	{
		Trunk::Trunk(const std::string &name) : Link(name){};
		Trunk::~Trunk(){};

		const std::string Trunk::getName() const
		{
			return name_;
		}
	} // namespace robotlib
} // namespace dls