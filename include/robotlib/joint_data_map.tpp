#ifndef _ROBOTLIB_JOINT_DATA_MAP_TPP_
#define _ROBOTLIB_JOINT_DATA_MAP_TPP_

#include "joint_data_map.hpp"

namespace robotlib
{
    template <class Data>
    JointDataMap<Data>::~JointDataMap(){};

    template <class Data>
    JointDataMap<Data>::JointDataMap(RobotBase *robot) : DataMap<Joint, Data>(robot->getNJOINTS())
    {
        int count_data = 0;

        for (auto leg : *(robot->getLegs()))
        {
            for (auto key : *(leg->getJoints()))
            {
                this->data_[count_data] = this->createPair(key, Data());
                count_data++;
            }
        }
    }
     
    template <class Data> 
    JointDataMap<Data>::JointDataMap(RobotBase *robot, const Data &data) : DataMap<Joint, Data>(robot->getNJOINTS())
    {
        int count_data = 0;

        for (auto leg : *(robot->getLegs()))
        {
            for (auto key : *(leg->getJoints()))
            {
                this->data_[count_data] = this->createPair(key, data);
                count_data++;
            }
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg) : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;

        for (auto key : *(leg->getJoints()))
        {
            this->data_[count_data] = this->createPair(key, Data());
            count_data++;
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg, const Data &data) : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;

        for (auto key : *(leg->getJoints()))
        {
            this->data_[count_data] = this->createPair(key, data);
            count_data++;
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap() : DataMap<Joint, Data>() //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!
    {
    }
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_TPP_