/*******************************************************************************
*                                                       ,----,                 *
*                                                     .'   .' \                *
*                                                   ,----,'    |               *
*               ________  ___       ________        |    :  .  ;               *
*              |\   ___ \|\  \     |\   ____\       ;    |.'  /                *
*              \ \  \_|\ \ \  \    \ \  \___|_      `----'/  ;                 *
*               \ \  \ \\ \ \  \    \ \_____  \       /  ;  /                  *
*                \ \  \_\\ \ \  \____\|____|\  \     ;  /  /-,                 *
*                 \ \_______\ \_______\____\_\  \   /  /  /.`|                 *
*                  \|_______|\|_______|\_________\./__;      :                 *
*                                     \|_________||   :    .'                  *
*                                                 ;   | .'                     *
*                                                 `---'                        *
********************************************************************************
* Author:            Legacy Code                                               *
* Maintainer:        Marco Marchitto                                           *
* Maintainer email:  marco.marchitto@iit.it                                    *
*******************************************************************************/
#ifndef ROBOTLIB_LINK_HPP
#define ROBOTLIB_LINK_HPP

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

    Link (const std::string& name): name_(name){};
	//Link (const std::string& name, const Pose& pose, const DynParams& dparams ): name_(name), pose_(pose), dyn_params_(dparams) {};

	~Link(){};
	
	// Get functions
	const std::string getName(){return name_;}
    
	//const Pose& getPose(){return pose_;}
	//const DynParams& getDynParams(){return dyn_params_;}

private:

	const std::string name_; 		             //! Name of the link
	// const Pose pose_;		                //! Pose of the link w.r.t. parent link
	// const DynParams dyn_params_; 		    //! Dynamic parameter of the link
};

} // namespace dog
} // namespace robot

#endif
