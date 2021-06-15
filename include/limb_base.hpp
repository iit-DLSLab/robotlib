#ifndef _ROBOTLIB_LIMB_BASE_HPP_
#define _ROBOTLIB_LIMB_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================

//#include "link.hpp"
//#include "joint.hpp"

#include <string>
#include <memory>

namespace dls{
namespace robot {
/**
 * A limb interface for robots.
 */

class LimbBase
{
public:
    LimbBase (const std::string& name): name_(name) {};
    
	virtual ~LimbBase(){};
    
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

#endif // _ROBOTLIB_LIMB_BASE_HPP_
