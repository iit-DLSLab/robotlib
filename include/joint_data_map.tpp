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
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getJoints()))
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
        for (auto leg : *DataHelper::getLegs(robot))
        {
            for (auto key : *(leg->getJoints()))
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
        for (auto key : *(leg->getJoints()))
        { 
            this->data_array_[count_data++] = this->createPair(key);
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const std::shared_ptr<LimbBase> leg, const Data &data) 
        : DataMap<Joint, Data>(leg->getNJoints())
    {
        int count_data = 0;
        for (auto key : *(leg->getJoints()))
        {
            this->data_array_[count_data++] = this->createPair(key, data);
        }
    }

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const JointDataMap<Data>& other) 
        : DataMap<Joint, Data>(other.getSize())
    {
        int count_data = 0;
        for (auto &pair : other)
        {
            this->data_array_[count_data++] = this->createPair(pair);
        }
    }

    template <class Data>
    JointDataMap<Data> &JointDataMap<Data>::operator=(const JointDataMap<Data>& other)
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

    template <class Data>
    JointDataMap<Data> JointDataMap<Data>::operator-(const JointDataMap<Data>& other)
    {
        JointDataMap<Data> out(*this);

        for(auto &joint_pair: out)
        {   
            out[joint_pair.key_] = *(joint_pair.data_) - other[joint_pair.key_];
        }
        return out;
    }

    template <class Data>
    JointDataMap<Data> &JointDataMap<Data>::operator-=(const JointDataMap<Data>& other)
    {
        for(auto &joint_pair: *this)
        {   
            *(joint_pair.data_) -= other[joint_pair.key_];
        }
        return *this;
    }

    template <class Data>
    JointDataMap<Data> JointDataMap<Data>::operator+(const JointDataMap<Data>& other)
    {
        JointDataMap<Data> out(*this);

        for(auto &joint_pair: out)
        {   
            out[joint_pair.key_] = *(joint_pair.data_) + other[joint_pair.key_];
        }
        return out;
    }

    template <class Data>
    JointDataMap<Data> &JointDataMap<Data>::operator+=(const JointDataMap<Data>& other)
    {
        for(auto &joint_pair: *this)
        {   
            *(joint_pair.data_) += other[joint_pair.key_];
        }
        return *this;
    }

    template <class Data>
    JointDataMap<Data> JointDataMap<Data>::operator*(const JointDataMap<Data>& other)
    {
        JointDataMap<Data> out(*this);

        for(auto &joint_pair: out)
        {   
            out[joint_pair.key_] = *(joint_pair.data_) * other[joint_pair.key_];
        }
        return out;
    }

    template <class Data>
    JointDataMap<Data> &JointDataMap<Data>::operator*=(const JointDataMap<Data>& other)
    {
        for(auto &joint_pair: *this)
        {   
            *(joint_pair.data_) *= other[joint_pair.key_];
        }
        return *this;
    }
}

template <class Data>
robotlib::JointDataMap<Data> operator*(const Eigen::VectorXd &vec, const JointDataMap<Data>& state)
{
    assert (vec.size() == state.size());

    robotlib::JointDataMap<Data> out(state);

    int i = 0;

    for(auto &joint_pair: state)
    {   
        out[joint_pair.key_] = vec[i++] * *(joint_pair.data_);
    }
    return out;
}
template <class Data>
robotlib::JointDataMap<Data> operator*(const double &esc, const JointDataMap<Data>& state)
{
    robotlib::JointDataMap<Data> out(state);

    for(auto &joint_pair: state)
    {   
        out[joint_pair.key_] = esc * *(joint_pair.data_);
    }
    return out;
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_TPP_