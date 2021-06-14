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

//#include "link.hpp"
//#include "joint.hpp"

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
    const std::string getName() const {return name_;};
    virtual const int getNumLinks() = 0;
    virtual const int getNumJoints() = 0;
    virtual std::shared_ptr<void> getLink(const int linkId) = 0;
    virtual std::shared_ptr<void> getJoint(const int jointId) = 0;    

private:
    const std::string name_;                                   //! Limb name

};

} // namespace robot
} // namespace dls

#endif
