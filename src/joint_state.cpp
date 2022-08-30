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

    JointState& JointState::operator=(const std::vector<double> data)
    {
        int i = 0;
        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                joint_pair.data_ = data[i++];
            }
        }
        return *this;
    }

    // JointState& JointState::operator=(const JointState &other)
    // {
    //     for (auto &leg_pair : *this)
    //     {
    //         for(auto &joint_pair: *leg_pair.data_)
    //         {   
    //             joint_pair.data_ = other[joint_pair.key_];
    //         }
    //     }
    //     return *this;
    // }

    JointState JointState::operator-(const JointState &other)
    {
        JointState out(*this);

        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                out[joint_pair.key_] = joint_pair.data_ - other[joint_pair.key_];
            }
        }
        return out;
    }

    JointState &JointState::operator-=(const JointState &other)
    {
        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                joint_pair.data_ -= other[joint_pair.key_];
            }
        }
        return *this;
    }

    JointState JointState::operator+(const JointState &other)
    {
        JointState out(*this);

        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                out[joint_pair.key_] = joint_pair.data_ + other[joint_pair.key_];
            }
        }
        return out;
    }

    JointState&JointState::operator+=(const JointState &other)
    {
        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                joint_pair.data_ += other[joint_pair.key_];
            }
        }
        return *this;
    }

    JointState& JointState::operator<< (double val)
    {
        Iterator<DataMap<LimbBase, std::shared_ptr<JointDataMap<double>>>::Pair> legIt = this->begin();
        jointIt = legIt->data_->begin();

        jointIt->data_ = val;

        return *this;
    }

    JointState& JointState::operator, (double val)
    {
        for (auto leg_pair = this->begin(); leg_pair !=  this->end(); leg_pair++)
        {
            for(auto joint_pair = leg_pair->data_->begin(); joint_pair != leg_pair->data_->end(); joint_pair++)
            {                   
                if(joint_pair == jointIt)
                {
                    joint_pair++;
                    if (joint_pair == leg_pair->data_->end())
                    {
                        leg_pair++;
                        if(leg_pair != this->end())
                        {
                            joint_pair = leg_pair->data_->begin();
                        }
                        else
                        {
                            throw std::range_error("Number of values bigger than the data map");
                        }
                    }
                    jointIt = joint_pair;
                    jointIt->data_ = val;

                    return *this;
                }
            }
        }
        throw std::range_error("Number of values bigger than the data map");
    }

    Eigen::VectorXd JointState::vec_()
    {
        Eigen::VectorXd out(this->size());

        int i = 0;
        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                out[i] = joint_pair.data_;
            }
        }
        return out;
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

    JointState::JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> legs) 
        : LegDataMap<std::shared_ptr<JointDataMap<double>>>(legs)
        , jointIt(nullptr)
    {}

    JointState::JointState(std::shared_ptr<JointState> dataMap) 
        : LegDataMap<std::shared_ptr<JointDataMap<double>>>(dataMap)
        , jointIt(nullptr)
    {
        for (auto &leg_pair : *this)
        {
            for(auto &joint_pair: *leg_pair.data_)
            {   
                joint_pair.data_ = (*dataMap)[joint_pair.key_];
            }
        }
    }

    JointState::operator std::vector<double>() const
    {
        std::vector<double> msg;

        msg.resize(this->size());

        int i = 0;
        for (auto &leg_pair : *this)
        {
            for (auto &joint_pair : *leg_pair.data_)
            {
                msg[i++] = joint_pair.data_;
            }
        }

        return msg;
    }
}

robotlib::JointState operator*(const double &esc, const robotlib::JointState &vec)
{
    robotlib::JointState out(vec);

    for (auto leg_pair : vec)
    {
        for(auto &joint_pair: *leg_pair.data_)
        {   
            out[joint_pair.key_] = esc * joint_pair.data_;
        }
    }
    return out;
}

#endif //_ROBOTLIB_JOINT_STATE_CPP_