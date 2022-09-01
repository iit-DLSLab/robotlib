#ifndef _ROBOTLIB_JOINT_DATA_MAP_TPP_
#define _ROBOTLIB_JOINT_DATA_MAP_TPP_

#include "joint_data_map.hpp"
#include "data_map.hpp"

namespace robotlib
{
    template <class Data>
    JointDataMap<Data>::~JointDataMap(){}

    template <class Data>
    JointDataMap<Data>::JointDataMap(std::shared_ptr<RobotBase> robot) 
        : DataMap<Joint, Data>(DataHelper::getRobotNumJoints(robot))
    {
        int count_data = 0;
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getJoints()))
            {
                this->data_array_[count_data++] = this->createPair(key, nullptr);
            }
        }
    }
     
    template <class Data> 
    JointDataMap<Data>::JointDataMap(std::shared_ptr<RobotBase> robot, const std::shared_ptr<Data> data) 
        : DataMap<Joint, Data>(DataHelper::getRobotNumJoints(robot))
    {
        int count_data = 0;
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getJoints()))
            {
                this->data_array_[count_data++] = this->createPair(key, data);
            }
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg) : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;
        for (auto key : *(leg->getJoints()))
        { 
            this->data_array_[count_data++] = this->createPair(key, nullptr);
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg, const std::shared_ptr<Data> data) : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;
        for (auto key : *(leg->getJoints()))
        {
            this->data_array_[count_data++] = this->createPair(key, data);
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const JointDataMap<Data> &other) 
        : DataMap<Joint, Data>(other.getSize())
    {
        int count_data = 0;
        for (auto pair : other)
        {
            this->data_array_[count_data++] = this->createPair(pair.key_, nullptr);
        }
    }

    // template <class Data> 
    // JointDataMap<Data>::JointDataMap() : DataMap<Joint, Data>() //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!
    // {
    // }
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_TPP_