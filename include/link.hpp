#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_
// =============================================================================
// Includes
// =============================================================================

#include "pose.hpp"
#include "dyn_params.hpp"

namespace dls{
namespace robot {
/**
 * A link class for robots.
 */
class Link
{
public:

    Link (LimbBase* parent, const std::string& name): parent_(parent), name_(name){};
	
	//Link (const std::string& name, const Pose& pose, const DynParams& dparams ): name_(name), pose_(pose), dyn_params_(dparams) {};

	~Link(){};
	
	// Get functions
	const std::string getName() const {return name_;}
    const LimbBase *getParent() const { return parent_; }
    
	//const Pose& getPose(){return pose_;}
	//const DynParams& getDynParams(){return dyn_params_;}

private:

	const std::string name_; 		             //! Name of the link
    const LimbBase *parent_;					//! Pointer to parent limb
	// const Pose pose_;		                //! Pose of the link w.r.t. parent link
	// const DynParams dyn_params_; 		    //! Dynamic parameter of the link
};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LINK_HPP_
