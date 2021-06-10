#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

// =============================================================================
// Includes
// =============================================================================
#include "pose.hpp"
#include "dyn_params.hpp"

namespace dls{
namespace robot {
/**
 * A trunk class for robots.
 */
class Trunk
{
public:

	Trunk (const Pose& origin, const DynParams& dparams ): origin_(origin), dyn_params_(dparams) {};
	
	~Trunk(){};

	// Get functions
	const Pose& getOrigin(){return origin_;}
	const DynParams& getDynParams(){return dyn_params_;}

private:
	const Pose origin_;				//! Pose of the trunk
	const DynParams dyn_params_; 		//! Dynamic parameter of the trunk
};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_TRUNK_HPP_
