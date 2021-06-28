#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"

namespace dls
{
	namespace robotlib
	{
		class Link : public Frame
		{
		public:
			Link(LimbBase *parent, const std::string &name);

			virtual ~Link();

			virtual const std::string getName() override;
			virtual const LimbBase *getParent() const override;
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LINK_HPP_
