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
#ifndef ROBOTLIB_LIMBBASE_HPP
#define ROBOTLIB_LIMBBASE_HPP

// =============================================================================
// Includes
// =============================================================================
#include "robotlib/link.hpp"
#include "robotlib/joint.hpp"

namespace dls{
namespace robot {
/**
 * A limb interface for robots.
 */

class LimbBase
{
public:
    LimbBase (const std::string& name): name_(name) {};
    
	~LimbBase(){};

    // Get functions
    const std::string getName() {return name_;};
    virtual const int getNumLinks() = 0;
    virtual const int getNumJoints() = 0;
    virtual const std::shared_ptr<Link>& getLink(const int linkId) = 0;
    virtual const std::shared_ptr<Joint>& getJoint(const int jointId) = 0;    

private:
    const std::string name_;                                   //! Limb name

};

} // namespace robot
} // namespace dls

#endif
