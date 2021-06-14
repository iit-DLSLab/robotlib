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
* Author:            Marco Marchitto                                           *
* Maintainer:        Marco Marchitto                                           *
* Maintainer email:  marco.marchitto@iit.it                                    *
*******************************************************************************/
#ifndef ROBOTLIB_ARMBASE_HPP
#define ROBOTLIB_ARMBASE_HPP

// =============================================================================
// Includes
// =============================================================================

#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"

namespace dls{
namespace robot {
/**
 * An arm interface for robots.
 */

class ArmBase : public LimbBase
{
public:
    ArmBase (const std::string& name) :LimbBase(name){};
    
	~ArmBase(){};

    // Functions for arms (and not for legs) TO DO

};

} // namespace robot
} // namespace dls

#endif
