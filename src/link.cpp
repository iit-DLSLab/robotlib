#include "link.hpp"

namespace dls
{
	namespace robotlib
	{
		Link::Link(const std::string &name) : Frame(name){};

		Link::~Link(){};

		const std::string Link::getName() const { return name_; };

		void Link::setChildren(const std::shared_ptr<ContainerBase<Joint>> children)
		{
			children_ = children;
		};

		const std::shared_ptr<Joint> Link::getParent() const { return parent_; };
		const std::shared_ptr<Joint> Link::getChild() const { return child_; };

		void Link::setChild(const std::shared_ptr<Joint> child) { child_ = child; }
		void Link::setParent(const std::shared_ptr<Joint> parent) { parent_ = parent; }
	} // namespace robotlib
} // namespace dls