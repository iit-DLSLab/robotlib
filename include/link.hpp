#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"
#include "link.hpp"
#include "limb_base.hpp"

namespace dls
{
	namespace robotlib
	{
		class Link : public Frame
		{
		public:
			Link(const LimbBase *parentLimb, const std::string &name);

			virtual ~Link();

			virtual const std::string getName() override;
			const LimbBase *getParentLimb() const; // MM: I'm not sure if returning a LimbBase* is completely safe...

			//protected:
			const LimbBase *parentLimb_{};
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LINK_HPP_
