#ifndef _ROBOTLIB_JOINT_STATE_CPP_
#define _ROBOTLIB_JOINT_STATE_CPP_

#include "joint_state.hpp"
#include <iostream> 
#include "utils/container.hpp"

namespace robotlib
{
    JointState::JointState(const ContainerBase<LimbBase>& limbs, const double& val) 
        : total_size(0)
        , LimbDataMap<JointDataMap<double>>(limbs, 
            [&]() -> std::vector<JointDataMap<double>>
            {
                std::vector<JointDataMap<double>> out;

                for (auto& limb : limbs)
                {
                    out.push_back(JointDataMap<double>(limb.getJoints(), val));
                }
                return out;
            }()
        )
        // , attribIt(nullptr)
    {
        for (auto& limb_pair : *this)
            this->total_size += limb_pair.getKey().getNJoints();
    }
    
    
    JointState::JointState(const JointState &other)
        : LimbDataMap<JointDataMap<double>>(other)
        // , attribIt(nullptr)
    {
        // for (auto &leg_pair : *this)
        // {
        //     leg_pair.data_ = std::shared_ptr<JointDataMap<double>>(new JointDataMap<double>(leg_pair.key_));
        //     for (auto &joint_pair : *leg_pair.data_)
        //     {
        //         joint_pair.data_ = std::shared_ptr<double>(new double(other[joint_pair.key_]));
        //     }
        // }
    }

    const std::vector<Joint*> JointState::getJoints() const
    {
        std::vector<Joint*> out;

         for (auto& limb_pair : *this)
        {
            for (auto& joint : limb_pair.getKey().getJoints())
            {
                out.push_back(&joint);
            }
        }
        return out;
    }

    double& JointState::operator[](const Joint& rhs)
    {
        for (auto& limb_pair : *this)
        {
            for (auto& joint_pair : limb_pair.getData())
            {
                if (joint_pair.getKey() == rhs)
                {
                    return joint_pair.getData();
                }
            }
        }
        throw std::range_error("key not found");
    }

    const double& JointState::operator[](const Joint& joint) const
    {
        return this->operator[](joint);
    }

    double& JointState::operator[](const std::shared_ptr<Joint>& joint)
    {
        return this->operator[](*joint);
    }

    const double& JointState::operator[](const std::shared_ptr<Joint>& joint) const
    {
        return this->operator[](*joint);
    }

    JointState& JointState::operator=(const JointState& rhs)
    {
        if(*this != rhs)
            throw std::range_error("Joint state operands are not for the same robot architecture");


        for (auto& limb_pair : *this)
        {
            limb_pair = rhs[limb_pair];
        }

        return *this;
    }

    JointState& JointState::operator=(const double& data)
    {
        for (auto& limb_pair : *this)
        {
            for (auto &joint_pair : limb_pair.getData()) //iterate over the joints of each limb
            {
                joint_pair = data;
            }
        }
        return *this;
    }

    JointState& JointState::operator=(const std::vector<double>& data)
    {
        assert (this->size() == data.size()); //verify if the size of this is equal to the number of items in vector

        unsigned int data_count = 0;
        for (auto& limb_pair : *this)
        {
            for(auto& joint_pair: limb_pair.getData())
            {   
                joint_pair = data[data_count++];
            }
        }
        return *this;
    }

    // JointState& JointState::operator<< (double val)
    // {
    //     auto limbIt = this->begin();
    //     this->attribIt = limbIt->getData().begin();

    //     *this->attribIt = val;

    //     return *this;
    // }

    // JointState& JointState::operator, (double val)
    // {
    //     for (auto limbIt = this->begin(); limbIt !=  this->end(); limbIt++)
    //     {
    //         for(auto jointIt = limbIt->getData().begin(); jointIt != limbIt->getData().end(); jointIt++)
    //         {                   
    //             if(jointIt == attribIt)
    //             {
    //                 jointIt++;
    //                 if (jointIt == limbIt->getData().end())
    //                 {
    //                     limbIt++;
    //                     if(limbIt != this->end())
    //                     {
    //                         jointIt = limbIt->getData().begin();
    //                     }
    //                     else
    //                     {
    //                         throw std::range_error("Number of values bigger than the data map");
    //                     }
    //                 }
    //                 this->attribIt = jointIt;
    //                 *this->attribIt = val;

    //                 return *this;
    //             }
    //         }
    //     }
    //     throw std::range_error("Number of values bigger than the data map");
    // }

    Eigen::VectorXd JointState::toeig_() const
    {
        Eigen::VectorXd out(this->size());

        unsigned int data_count{0};
        for (auto& limb_pair : *this)
        {
            for(auto& joint_pair: limb_pair.getData())
            {   
                out[data_count++] = joint_pair.getData();
            }
        }
        return out;
    }

    Eigen::VectorXd JointState::toeig_(const std::shared_ptr<LimbBase>& limb) const
    {
        Eigen::VectorXd out(limb->getNJoints());

        unsigned int data_count{0};
        for(auto& joint_pair: (*this)[limb])
        {   
            out[data_count++] = joint_pair.getData();
        }
        return out;
    }


    std::vector<double> JointState::tovec_() const
    {
        std::vector<double> out;

        for (auto& limb_pair : *this)
        {
            for(auto& joint_pair: limb_pair.getData())
            {  
                out.push_back(joint_pair.getData());
            }
        }
        return out;
    }

    void JointState::setZero() 
    { 
        *this = 0; 
    }

    long unsigned int JointState::size() const
    {
        return this->total_size;
    }

    double JointState::max()
    {
        double* max_value = nullptr;

        for (auto& limb_pair : *this)
        {
            for (auto& joint_pair : limb_pair.getData()) //iterate over the joints
            {
                if(max_value == nullptr || *max_value > joint_pair.getData())
                    max_value = &joint_pair.getData();
            }
        }
        return *max_value;
    }

    double JointState::min()
    {
        double* min_value = nullptr;

        for (auto& limb_pair : *this)
        {
            for (auto& joint_pair : limb_pair.getData()) //iterate over the joints
            {
                if(min_value == nullptr || *min_value < joint_pair.getData())
                    min_value = &joint_pair.getData();
            }
        }
        return *min_value;
    }

    bool JointState::operator==(const JointState& rhs) const
    {
        if(this->size() != rhs.size())
            return false;
        
        for (auto& limb : *this)
        {
            limb.getData() != rhs[limb];
                return false;
        }

        return true;
    }

    JointDataMap<double>& JointState::getLimbJointState(const std::shared_ptr<LimbBase> limb) 
    { 
        return (*this)[limb]; 
    }

    const JointDataMap<double>& JointState::getLimbJointState(const std::shared_ptr<LimbBase> limb) const 
    { 
        return (*this)[limb]; 
    }

}

robotlib::JointState operator*(const double& esc, const robotlib::JointState& state)
{
    robotlib::JointState out(state);

    for (auto& limb_pair : state)
    {
        for(auto& joint_pair : limb_pair.getData())
        {
            out[limb_pair][joint_pair] = esc * joint_pair.getData();
        }   
    }
    return out;
}

robotlib::JointState operator*(const Eigen::VectorXd& vec, const robotlib::JointState& state)
{

    assert (vec.size() == state.size());

    robotlib::JointState out(state);

    unsigned int data_count{0};
    for (auto& limb_pair : state)
    {
        for(auto& joint_pair : limb_pair.getData())
        {
            out[limb_pair][joint_pair] = vec[data_count++] * joint_pair.getData();
        }  
    }
    return out;
}

Eigen::Quaterniond operator+=(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2)
{
    Eigen::Quaterniond out;
    out.w() = q1.w() + q2.w(); //Add the scalar portion
    out.vec() = q1.vec() + q2.vec(); 

    return out;
}

Eigen::Quaterniond operator-=(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2)
{
    Eigen::Quaterniond out;
    out.w() = q1.w() - q2.w(); //Add the scalar portion
    out.vec() = q1.vec() - q2.vec(); 

    return out;
}

#endif //_ROBOTLIB_JOINT_STATE_CPP_