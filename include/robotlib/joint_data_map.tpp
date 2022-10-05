#ifndef _ROBOTLIB_JOINT_DATA_MAP_TPP_
#define _ROBOTLIB_JOINT_DATA_MAP_TPP_

#include "joint_data_map.hpp"
#include "data_map.hpp"

namespace robotlib
{
    template <class Data>
    JointDataMap<Data>::~JointDataMap(){}

    template <class Data>
    JointDataMap<Data>::JointDataMap(RobotBase *robot) 
        : DataMap<Joint, Data>(DataHelper::getRobotNumJoints(robot))
    {
        int count_data = 0;
        for (auto &leg : *DataHelper::getLegs(robot))
        {
            for (auto &key : *(leg->getJoints()))
            {
                this->data_array_[count_data++] = this->createPair(key);
            }
        }
    }
     
    template <class Data> 
    JointDataMap<Data>::JointDataMap(RobotBase *robot, const Data& data) 
        : DataMap<Joint, Data>(DataHelper::getRobotNumJoints(robot))
    {
        int count_data = 0;
        for (auto &leg : *DataHelper::getLegs(robot))
        {
            for (auto &key : *(leg->getJoints()))
            {
                this->data_array_[count_data++] = this->createPair(key, data);
            }
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg) 
        : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;
        for (auto &key : *(leg->getJoints()))
        { 
            this->data_array_[count_data++] = this->createPair(key);
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg, const Data &data) 
        : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;
        for (auto &key : *(leg->getJoints()))
        {
            this->data_array_[count_data++] = this->createPair(key, data);
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const JointDataMap<Data> &other) 
        : DataMap<Joint, Data>(other.getSize())
    {
        int count_data = 0;
        for (auto &pair : other)
        {
            this->data_array_[count_data++] = this->createPair(pair);
        }
    }

    template <class Data>
    JointDataMap<Data> &JointDataMap<Data>::operator=(const JointDataMap<Data> &other)
    {
        int count_data = 0;
        for (auto &pair: other)
        {
            this->data_array_[count_data].key_ = pair.key_;
            if (this->data_array_[count_data].data_ == nullptr)
            {
                this->data_array_[count_data].data_ = std::shared_ptr<Data>(new Data(*pair.data_));
            }
            else{
                *this->data_array_[count_data++].data_ = *pair.data_;
            }
        }
        return *this;
    }
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_TPP_