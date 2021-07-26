#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

#include "frame.hpp"
#include "utils.hpp"
#include "joint.hpp"

namespace dls
{
	namespace robotlib
	{
		class Trunk : public Frame
		{
		public:
			Trunk(const std::string &name);

			~Trunk();

			virtual const std::string getName() override;
			template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
			friend class Robot;

		protected:
			void setChildren(const std::shared_ptr<ContainerBase<Joint>> children);

		private:
			std::shared_ptr<ContainerBase<Joint>> children_;
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_TRUNK_HPP_
