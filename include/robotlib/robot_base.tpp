#ifndef _ROBOTLIB_ROBOT_BASE_TPP_
#define _ROBOTLIB_ROBOT_BASE_TPP_

#include "robot_base.hpp"

namespace robotlib
{
    // Create a llimb data map pair
    template <class Data>
    LimbDataMap<Data> RobotBase::makeLimbDataMap(const Data& data) const
    { 
        LimbDataMap<Data> data_map;
        for(auto limb : this->getLimbs()){
            data_map[limb] = data;
        }
        return data_map;
    }

    template <class Data>
    LimbDataMap<Data> RobotBase::makeLimbDataMap(const std::vector<Data>& data_vector) const
    {        
        LimbDataMap<Data> data_map;
        for(auto limb : this->getLimbs()){
            data_map[limb] = data_vector[limb->id];
        }
        return data_map;
    }

    template <class Data>
    LinkDataMap<Data> RobotBase::makeLinkDataMap(const Data& data) const
    { 
        LinkDataMap<Data> data_map;
        for(auto link : this->getLinks()){
            data_map[link] = data;
        }
        return data_map;
    }

    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap(const std::vector<Data>& data) const
    {
        JointDataMap<Data> data_map;
        for(auto joint : this->getJoints()){
            data_map[joint] = data[joint->id];
        }
        return data_map;
    }

    template <class Data>
    JointDataMap<Data> RobotBase::makeJointDataMap(const Data& data) const
    { 
        JointDataMap<Data> data_map;
        for(auto joint : this->getJoints()){
            data_map[joint] = data;
        }
        return data_map;
    }
} // namespace robotlib

#include "robot_base.tpp"
#endif // _ROBOTLIB_ROBOT_BASE_TPP_