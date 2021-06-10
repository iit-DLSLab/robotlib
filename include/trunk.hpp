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
#ifndef ROBOTLIB_TRUNK_HPP
#define ROBOTLIB_TRUNK_HPP

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

} // namespace dog
} // namespace robot

#endif
