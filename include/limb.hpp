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
#ifndef ROBOTLIB_LIMB_HPP
#define ROBOTLIB_LIMB_HPP

// =============================================================================
// Includes
// =============================================================================

#include "limb_base.hpp"
#include "joint.hpp"
#include "link.hpp"

namespace dls{
namespace robot {
/**
 * A limb class for robots. 
 */

template <unsigned int NJOINTS, unsigned int NLINKS>
class Limb : public LimbBase
{
public:
    Limb (const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links) 
        : LimbBase(name), joints_(joints), links_(links) {};

	~Limb(){};
    
    // Get functions
    //std::array<std::shared_ptr<Link>, NLINKS> getLinks(){return links_;};
    //std::array<std::shared_ptr<Joint>, NJOINTS> getJoints(){return joints_;};

    virtual std::shared_ptr<void> getLink(const int linkId)  override {return links_[linkId];};
    virtual std::shared_ptr<void> getJoint(const int jointId) override {return joints_[jointId];};    
    virtual const int getNumLinks() override {return links_.size();};
    virtual const int getNumJoints() override {return joints_.size();};

    //forward kinematics

private:   

    const std::array<std::shared_ptr<Joint>, NJOINTS> joints_;        //! Array of joints
    const std::array<std::shared_ptr<Link>, NLINKS> links_;           //! Array of links
};

} // namespace robot
} // namespace dls

#endif
