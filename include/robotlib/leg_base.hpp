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

#include "robotlib/limb.hpp"

namespace dls{
namespace robot {
/**
 * A leg interface for robots.
 */

template <unsigned int NJOINTS, unsigned int NLINKS>
class LegBase : public Limb<NJOINTS, NLINKS>
{
public:
    LegBase (const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :Limb<NJOINTS,NLINKS>(name, joints, links) {};
    
	~LegBase(){};

    // Functions for legs (and not for arms) TO DO

};

} // namespace robot
} // namespace dls

#endif
