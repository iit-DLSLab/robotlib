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

#include "robotlib/limb.hpp"

namespace dls{
namespace robot {
/**
 * An arm interface for robots.
 */

template <unsigned int NJOINTS, unsigned int NLINKS>
class ArmBase : public Limb<NJOINTS, NLINKS>
{
public:
    ArmBase (const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :Limb<NJOINTS,NLINKS>(name, joints, links) {};
    
	~ArmBase(){};

    // Functions for arms (and not for legs) TO DO

};

} // namespace robot
} // namespace dls

#endif
