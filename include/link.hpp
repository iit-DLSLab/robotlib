#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"
#include "utils.hpp"
#include <memory>

#include <iostream>
namespace dls
{
	namespace robotlib
	{
		class Joint;
		template <unsigned int NJOINTS, unsigned int NLINKS>
		class Limb;
		class Link : public Frame
		{
		public:
			Link(const std::string &name);

			virtual ~Link();

			virtual const std::string getName() override;

			template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
			friend class Robot;

			const std::shared_ptr<Joint> getParent() const;
			const std::shared_ptr<Joint> getChild() const;

		protected:
			std::shared_ptr<ContainerBase<Joint>> children_;
			std::shared_ptr<Joint> parent_;
			std::shared_ptr<Joint> child_;

			void setChildren(const std::shared_ptr<ContainerBase<Joint>> children);
			void setParent(const std::shared_ptr<Joint> parent);
			void setChild(const std::shared_ptr<Joint> child);
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LINK_HPP_
