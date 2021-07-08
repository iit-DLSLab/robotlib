#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

#include "frame.hpp"

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
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_TRUNK_HPP_
