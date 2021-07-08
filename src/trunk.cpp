#include "trunk.hpp"

namespace dls
{
	namespace robotlib
	{
		Trunk::Trunk(const std::string &name) : Frame(name){};

		Trunk::~Trunk(){};

		const std::string Trunk::getName() { return name_; };
	} // namespace robotlib
} // namespace dls