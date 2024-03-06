#ifndef _ROBOTLIB_JOINT_DATA_MAP_TPP_
#define _ROBOTLIB_JOINT_DATA_MAP_TPP_

#include "joint_data_map.hpp"

namespace robotlib
{
    template <class Data> 
    JointDataMap<Data>::JointDataMap(const  ContainerBase<Joint>& joints, const Data& data) 
        : DataMap<Joint, Data>(joints, data)
    {}

    template <class Data>
    JointDataMap<Data>::JointDataMap(const ContainerBase<Joint>& joints, const std::vector<Data>& data) 
        : DataMap<Joint, Data>(joints, data)
    {}

    template <class Data> 
    JointDataMap<Data>::JointDataMap(const JointDataMap<Data>& other) 
        : DataMap<Joint, Data>(other)
    {}

    template <class Data> 
    JointDataMap<Data>& JointDataMap<Data>::operator=(const JointDataMap<Data>& other) 
    {
        this->DataMap<Joint, Data>::operator=(other);
        
        return *this;
    }

    template <class Data> 
    JointDataMap<Data>& JointDataMap<Data>::operator=(const std::vector<Data>& other) 
    {
        unsigned int data_count{0};
        for(auto& joint_pair : *this)
            joint_pair = other[data_count++];
        
        return *this;
    }

    // template <class Data>
    // JointDataMap<Data> JointDataMap<Data>::operator-(const JointDataMap<Data>& other)
    // {
    //     JointDataMap<Data> out(*this);

    //     for(auto &joint_pair: out)
    //     {   
    //         joint_pair -= other[joint_pair];
    //     }
    //     return out;
    // }

    template <class Data>
    JointDataMap<Data> JointDataMap<Data>::operator-(const JointDataMap<Data>& other) const
    {
        JointDataMap<Data> out(*this);

        for(auto& joint_pair: out)
        {   
            joint_pair -= other[joint_pair];
        }
        return out;
    }

    template <class Data>
    JointDataMap<Data>& JointDataMap<Data>::operator-=(const JointDataMap<Data>& other)
    {
        for(auto& joint_pair: *this)
        {   
            joint_pair -= other[joint_pair];
        }
        return *this;
    }

    // template <class Data>
    // JointDataMap<Data> JointDataMap<Data>::operator+(const JointDataMap<Data>& other)
    // {
    //     JointDataMap<Data> out(*this);

    //     for(auto &joint_pair: out)
    //     {   
    //         out[joint_pair.key_] = *(joint_pair.data_) + other[joint_pair.key_];
    //     }
    //     return out;
    // }

    template <class Data>
    JointDataMap<Data> JointDataMap<Data>::operator+(const JointDataMap<Data>& other) const
    {
        JointDataMap<Data> out(*this);

        for(auto& joint_pair: out)
        {   
            joint_pair += other[joint_pair];
        }
        return out;
    }

    template <class Data>
    JointDataMap<Data> &JointDataMap<Data>::operator+=(const JointDataMap<Data>& other)
    {
        for(auto& joint_pair: *this)
        {   
            joint_pair += other[joint_pair];
        }
        return *this;
    }

    // template <class Data>
    // JointDataMap<Data> JointDataMap<Data>::operator*(const JointDataMap<Data>& other)
    // {
    //     JointDataMap<Data> out(*this);

    //     for(auto &joint_pair: out)
    //     {   
    //         out[joint_pair.key_] = *(joint_pair.data_) * other[joint_pair.key_];
    //     }
    //     return out;
    // }

    template <class Data>
    JointDataMap<Data> JointDataMap<Data>::operator*(const JointDataMap<Data>& other) const
    {
        JointDataMap<Data> out(*this);

        for(auto& joint_pair: out)
        {   
            joint_pair *= other[joint_pair];
        }
        return out;
    }

    template <class Data>
    JointDataMap<Data>& JointDataMap<Data>::operator*=(const JointDataMap<Data>& other)
    {
        for(auto& joint_pair: *this)
        {   
            joint_pair *= other[joint_pair];
        }
        return *this;
    }
}

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_TPP_