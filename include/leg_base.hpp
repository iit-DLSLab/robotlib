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
#ifndef ROBOTLIB_LEGBASE_HPP
#define ROBOTLIB_LEGBASE_HPP

// =============================================================================
// Includes
// =============================================================================

#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"

namespace dls{
namespace robot {
/**
 * A leg interface for robots.
 */

class LegBase : public LimbBase
{
public:
    LegBase (const std::string& name):LimbBase(name) {};
    
	~LegBase(){};

    // Functions for legs (and not for arms) TO DO

};

} // namespace robot
} // namespace dls

#endif
