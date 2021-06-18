#ifndef _ROBOTLIB_JOINT_STATE_BASE_HPP_
#define _ROBOTLIB_JOINT_STATE_BASE_HPP_

namespace dls
{
	namespace robotlib
	{
		/**
		 * A joint class for robots.
		 */
		class JointStateBase
		{
		public:
			JointStateBase(){};
			//Joint (const std::string& name, const DynParams& dparams ): name_(name), dyn_params_(dparams) {};

			~JointStateBase(){};

			//iterator begin()
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_JOINT_STATE_BASE_HPP_