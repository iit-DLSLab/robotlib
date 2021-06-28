#ifndef _ROBOTLIB_JOINT_HPP_
#define _ROBOTLIB_JOINT_HPP_

#include "frame.hpp"

namespace dls
{
	namespace robotlib
	{
		class Joint : public Frame
		{
		public:
			Joint(LimbBase *parent, const std::string &name);

			virtual ~Joint();

			virtual const std::string getName() override;
			virtual const LimbBase *getParent() const override;
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_JOINT_HPP_
