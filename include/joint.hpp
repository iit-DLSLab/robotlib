#ifndef _ROBOTLIB_JOINT_HPP_
#define _ROBOTLIB_JOINT_HPP_

#include "frame.hpp"
#include "link.hpp"

namespace dls
{
	namespace robotlib
	{
		class Joint : public Frame
		{
		public:
			Joint(Link *parent, Link *child, const std::string &name);

			virtual ~Joint();

			virtual const std::string getName() override;
			const Link *getParent() const;
			const Link *getChild() const;

		protected:
			const Link *parent_{};
			const Link *child_{};
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_JOINT_HPP_
