#include "link.hpp"

namespace dls
{
	namespace robotlib
	{
		Link::Link(const std::string &name) : Frame(name){};

		Link::~Link(){};

		const std::string Link::getName() { return name_; }
		const std::shared_ptr<Joint> Link::getParent() const { return parent_; }
		const std::shared_ptr<Joint> Link::getChild() const { return child_; }

		void Link::setChild(const std::shared_ptr<Joint> child) { child_ = child; }
		void Link::setParent(const std::shared_ptr<Joint> parent) { parent_ = parent; }
		void Link::setChildren(const std::shared_ptr<ContainerBase<Joint>> children) { children_ = children; }
	} // namespace robotlib
} // namespace dls