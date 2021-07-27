#ifndef _ROBOTLIB_JOINT_HPP_
#define _ROBOTLIB_JOINT_HPP_

#include "frame.hpp"
#include <memory>

namespace dls
{
	namespace robotlib
	{

		class Link;
		template <unsigned int NJOINTS, unsigned int NLINKS>
		class Limb;

		class Joint : public Frame
		{
		public:
			Joint(const std::string &name);

			virtual ~Joint();

			virtual const std::string getName() override;
			const std::shared_ptr<Link> getParent() const;
			const std::shared_ptr<Link> getChild() const;

			template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
			friend class Robot;

		protected:
			std::shared_ptr<Link>
				parent_;
			std::shared_ptr<Link> child_;

		private:
			void setParent(const std::shared_ptr<Link> parent);
			void setChild(const std::shared_ptr<Link> child);
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_JOINT_HPP_
