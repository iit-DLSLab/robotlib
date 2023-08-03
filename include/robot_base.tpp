#ifndef _ROBOTLIB_ROBOT_BASE_TPP_
#define _ROBOTLIB_ROBOT_BASE_TPP_

#include "robot_base.hpp"

namespace robotlib
{

    template <class Data>
    LegDataMap<Data> RobotBase::makeLegDataMap() const
    { 
        return LegDataMap<Data>(this->getLegs()); 
    } // NRT

    // Create a leg data map pair
    template <class Data>
    LegDataMap<Data> RobotBase::makeLegDataMap(const Data &data) const
    { 
        return LegDataMap<Data>(this->getLegs(), data); 
    }

    // Create a link data map pair
    template <class Data>
    LinkDataMap<Data> RobotBase::makeLinkDataMap() 
    { 
        return LinkDataMap<Data>(this); 
    } // NRT

    template <class Data>
    LinkDataMap<Data> RobotBase::makeLinkDataMap(const Data &data) 
    { 
        return LinkDataMap<Data>(this, data); 
    }

    // Create a joint data map pair
    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap() 
    { 
        return JointDataMap<Data>(this); 
    } // NRT

    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap(const Data &data) 
    { 
        return JointDataMap<Data>(this, data); 
    } // NRT

    // Create a joint data map pair
    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg) 
    { 
        return JointDataMap<Data>(leg); 
    } // NRT

    // Create a joint data map pair
    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data &data) 
    { 
        return JointDataMap<Data>(leg, data); 
    } // NRT
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_TPP_