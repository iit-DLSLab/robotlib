#ifndef _ROBOTLIB_LIMB_HPP_
#define _ROBOTLIB_LIMB_HPP_

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

    virtual const std::shared_ptr<Link>& getLink(const int linkId)  override {return links_[linkId];};
    virtual const std::shared_ptr<Joint>& getJoint(const int jointId) override {return joints_[jointId];};    
    virtual const int getNumLinks() override {return links_.size();};
    virtual const int getNumJoints() override {return joints_.size();};

    //forward kinematics

private:   

    const std::array<std::shared_ptr<Joint>, NJOINTS> joints_;        //! Array of joints
    const std::array<std::shared_ptr<Link>, NLINKS> links_;           //! Array of links
};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LIMB_HPP_
