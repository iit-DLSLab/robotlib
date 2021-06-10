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
#ifndef ROBOTLIB_JOINT_HPP
#define ROBOTLIB_JOINT_HPP

// =============================================================================
// Includes
// =============================================================================
#include "robotlib/dyn_params.hpp"

namespace dls{
namespace robot {
/**
 * A joint class for robots.
 */
class Joint
{
public:

    Joint (const std::string& name): name_(name){};
	//Joint (const std::string& name, const DynParams& dparams ): name_(name), dyn_params_(dparams) {};

	~Joint (){};

	// Get functions
	const std::string getName(){return name_;}
	//const DynParams& getDynParams(){return dyn_params_;}

private:
	const std::string name_;               //! Name of the joint
	//const DynParams dyn_params_;	        //! Dynamic parameter of the joint

	//joint limits TODO
};

} // namespace dog
} // namespace robot

#endif
