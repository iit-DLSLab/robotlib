#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"
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

			template <unsigned int NJOINTS, unsigned int NLINKS>
			friend class Limb;

		protected:
			std::shared_ptr<Joint> child_;
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LINK_HPP_
