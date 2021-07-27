#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

#include "link.hpp"
#include "utils.hpp"
#include "joint.hpp"

namespace dls
{
	namespace robotlib
	{
		class Trunk : public Link
		{
		public:
			Trunk(const std::string &name);

			~Trunk();

			virtual const std::string getName() override;
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_TRUNK_HPP_
