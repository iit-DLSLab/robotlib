#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

#include "pose.hpp"
#include "dyn_params.hpp"

namespace dls
{
	namespace robotlib
	{
		class Trunk
		{
		public:
			Trunk(const Pose &origin, const DynParams &dparams);

			~Trunk();

			const Pose &getOrigin();
			const DynParams &getDynParams();

		private:
			const Pose origin_;			 //! Pose of the trunk
			const DynParams dyn_params_; //! Dynamic parameter of the trunk
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_TRUNK_HPP_
