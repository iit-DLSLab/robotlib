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
#ifndef ROBOTLIB_ARM_HPP
#define ROBOTLIB_ARM_HPP

// =============================================================================
// Includes
// =============================================================================

#include "arm_base.hpp"

namespace dls{
namespace robot {
/**
 * An arm class for robots.
 */
template<unsigned int NJOINTS, unsigned int NLINKS>
class Arm : public ArmBase
{
public:

	Arm(const std::string& name, const std::array<std::shared_ptr<Joint>, NJOINTS>& joints, const std::array<std::shared_ptr<Link>, NLINKS>& links)
            :ArmBase(name), joints_(joints), links_(links) {};

	~Arm(){};

    virtual std::shared_ptr<void> getLink(const int linkId)  override {return links_[linkId];};
    virtual std::shared_ptr<void> getJoint(const int jointId) override {return joints_[jointId];};    
    virtual const int getNumLinks() override {return links_.size();};
    virtual const int getNumJoints() override {return joints_.size();};

    //forward kinematics

private:   

    const std::array<std::shared_ptr<Joint>, NJOINTS> joints_;        //! Array of joints
    const std::array<std::shared_ptr<Link>, NLINKS> links_;           //! Array of links

} // namespace robot
} // namespace dls

#endif

