#include "trunk.hpp"

namespace dls
{
	namespace robotlib
	{
		Trunk::Trunk(const Pose &origin, const DynParams &dparams) : origin_(origin), dyn_params_(dparams){};

		Trunk::~Trunk(){};

		// Get functions
		const Pose &Trunk::getOrigin() { return origin_; }
		const DynParams &Trunk::getDynParams() { return dyn_params_; }
	} // namespace robotlib
} // namespace dls