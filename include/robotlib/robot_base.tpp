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
    LegDataMap<Data> RobotBase::makeLegDataMap(const std::shared_ptr<Data> data) const
    { 
        return LegDataMap<Data>(this->getLegs(), data); 
    } // NRT

    template <class Data>
    LegDataMap<Data> RobotBase::makeLegDataMap(const Data &data) const
    { 
        return LegDataMap<Data>(this->getLegs(), std::make_shared<Data>(data)); 
    }

    // Create a link data map pair
    template <class Data>
    LinkDataMap<Data> RobotBase::makeLinkDataMap() 
    { 
        return LinkDataMap<Data>(std::shared_ptr<RobotBase>(this)); 
    } // NRT

    template <class Data>
    LinkDataMap<Data> RobotBase::makeLinkDataMap(const std::shared_ptr<Data> data) 
    { 
        return LinkDataMap<Data>(std::shared_ptr<RobotBase>(this), data); 
    }

    // Create a joint data map pair
    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap() 
    { 
        return JointDataMap<Data>(std::shared_ptr<RobotBase>(this)); 
    } // NRT

    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap(const std::shared_ptr<Data> data) 
    { 
        return JointDataMap<Data>(std::shared_ptr<RobotBase>(this), data); 
    } // NRT

    // Create a joint data map pair
    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg) 
    { 
        return JointDataMap<Data>(leg); 
    } // NRT

    // Create a joint data map pair
    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const std::shared_ptr<Data> data) 
    { 
        return JointDataMap<Data>(leg, data); 
    } // NRT
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_TPP_