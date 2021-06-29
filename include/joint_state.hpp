#ifndef _ROBOTLIB_JOINT_STATE_HPP_
#define _ROBOTLIB_JOINT_STATE_HPP_

namespace dls
{
	namespace robotlib
	{
		class JointState
		{
		public:
			JointState();
			//Joint (const std::string& name, const DynParams& dparams ): name_(name), dyn_params_(dparams) {};

			~JointState();

			//iterator begin()
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_JOINT_STATE_HPP_