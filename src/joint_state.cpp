#ifndef _ROBOTLIB_JOINT_STATE_CPP_
#define _ROBOTLIB_JOINT_STATE_CPP_

#include "joint_state.hpp"
#include <iostream> 

namespace robotlib
{

    double& JointState::operator[](const std::shared_ptr<Joint> joint)
    {
        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
            {
                if (joint_pair.key_->getName().compare(joint->getName()) == 0)
                {
                    return joint_pair.data_;
                }
            }
        }
        throw std::range_error("key not found");
    }

    const double& JointState::operator[](const std::shared_ptr<Joint> joint) const
    {
        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
            {
                if (joint_pair.key_->getName().compare(joint->getName()) == 0)
                {
                    return joint_pair.data_;
                }
            }
        }
        throw std::range_error("key not found");
    }

    JointState& JointState::operator=(const double data)
    {
        for (auto leg_pair : *this)
        {
            (*leg_pair.data_).assignAll(data);
        }
        return *this;
    }

    JointState& JointState::operator=(const JointState &other)
    {
        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                joint_pair.data_ = other[joint_pair.key_];
            }
        }
        return *this;
    }

    void JointState::setZero() 
    { 
        *this = 0; 
    }

    int JointState::size() const
    {
        auto size{0};

        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
            {
                size++;
            }
        }

        return size;
    }

    /**
     * @brief Return the max value of the joint state
     * @return double
     */
    double JointState::max()
    {
        double max_value{0};
        bool first_val{true};

        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
            {
                double value = joint_pair.data_;
                if (first_val==true)
                {
                    max_value = value;
                    first_val=false;
                }
                else
                {
                    if(value>max_value)
                    {
                        max_value = value;
                    }
                }
            }
        }
        return max_value;
    }


    /**
     * @brief Return the min value of the joint state
     * @return double
     */
    double JointState::min()
    {
        double min_value{0};
        bool first_val{true};

        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
            {
                double value = joint_pair.data_;
                if (first_val==true)
                {
                    min_value = value;
                    first_val=false;
                }
                else
                {
                    if(value<min_value)
                    {
                        min_value = value;
                    }
                }
            }
        }
        return min_value;
    }

    void JointState::print()
    {
        std::cout << "JointState [Name - Value]" << std::endl;
        std::cout << "-------------------------" << std::endl;

        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_)
            {
                std::cout << joint_pair.key_->getName() << " - " << joint_pair.data_ << std::endl;
            }
        }
    }

    std::shared_ptr<JointDataMap<double>>& JointState::getLegJointState(const std::shared_ptr<LimbBase> leg) 
    { 
        return (*this)[leg->getName()]; 
    }

    const std::shared_ptr<JointDataMap<double>>& JointState::getLegJointState(const std::shared_ptr<LimbBase> leg) const 
    { 
        return (*this)[leg->getName()]; 
    }

    JointState::~JointState(){}

    JointState::JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs) : LegDataMap<std::shared_ptr<JointDataMap<double>>>(legs){}
}

#endif //_ROBOTLIB_JOINT_STATE_CPP_