#include "trunk.hpp"

namespace dls
{
	namespace robotlib
	{
		Trunk::Trunk(const std::string &name) : Frame(name){};
		Trunk::~Trunk(){};

		const std::string Trunk::getName()
		{
			return name_;
		}
		void Trunk::setChildren(const std::shared_ptr<ContainerBase<Joint>> children)
		{
			children_ = children;
		};
	} // namespace robotlib
} // namespace dls