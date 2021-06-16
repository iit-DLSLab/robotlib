#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

// stdlib
#include <fstream>

#include <memory>

#include "limb_base.hpp"
#include "leg.hpp"
#include "link.hpp"
#include "joint.hpp"
#include "trunk.hpp"

#include "utils.hpp"


namespace dls
{
namespace robot
{

// =============================================================================
// Class Interface
// =============================================================================

class RobotBase {
private:
    template<class Data>
    class LegDataMap{
    public:
        LegDataMap(const int nLegs): nLegs_(nLegs){    
            Data *p = new Data[nLegs_];
            std::shared_ptr<Data> pshrd(p);
            data_ = pshrd;
        }
        ~LegDataMap(){};
        
        Iterator<Data> begin() { return Iterator<Data>(&data_.get()[0]); }
        Iterator<Data> end() { return Iterator<Data>(&data_.get()[nLegs_]); }
    private:
        std::shared_ptr<Data> data_;
        
        const int nLegs_;
    };



public:

    virtual Iterator<const std::shared_ptr<LimbBase>> begin() {};
    virtual Iterator<const std::shared_ptr<LimbBase>> end() {}; 

    // Get functions
    virtual const int getNLEGS() = 0;
    virtual const int getNJOINTS() = 0;
    virtual const int getNLINKS() = 0;

    virtual const std::shared_ptr<LimbBase> getLeg(const int id) = 0;

	// Plugin typedefs
	typedef std::shared_ptr<RobotBase> createRobot_t();
	typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

    
    // Create a leg data map
    template<class Data> LegDataMap<Data> makeLegDataMap(){return LegDataMap<Data>(this->getNLEGS());}

    // Create a joint data map
    template<class Data> LegDataMap<Data> makeJointDataMap(){return LegDataMap<Data>(this->getNJOINTS());}

    // Create a link data map
    template<class Data> LegDataMap<Data> makeLinkDataMap(){return LegDataMap<Data>(this->getNLINKS());}

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LIMB_HPP_
