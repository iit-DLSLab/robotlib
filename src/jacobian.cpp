#ifndef _ROBOTLIB_JACOBIAN_CPP_
#define _ROBOTLIB_JACOBIAN_CPP_

#include "jacobian.hpp"
#include <iostream>

namespace robotlib
{
    
    

    Jacobian::Jacobian(const Jacobian& jacobian) 
        : data_(new double[6 * jacobian.nJoints_])
        , nJoints_(jacobian.nJoints_)
        , Map(data_, 6, nJoints_)
    {
        for (int i = 0; i < 6 * jacobian.nJoints_; ++i)
        {
            data_[i] = jacobian.data_[i];
        }
    }

    Jacobian::Jacobian(const int nJoints, const double& data) 
        : data_(new double[6 * nJoints](data))
        , nJoints_(nJoints)
        , Map(data_, 6, nJoints_)
    { }

    Jacobian::Jacobian(const std::vector<double>& data) 
        : data_(new double[data.size()])
        , nJoints_(data.size()/6)
        , Map(data_, 6, nJoints_)
    {
        if(data.size()%6 != 0)
            throw std::invalid_argument("INITIALIZATION VECTOR SHOULD HAVE MODULUS 6.");

        auto data_count{0};
        for (int i = 0; i < data.size(); ++i)
        {
            data_[i] = data[data_count++];
        }
    }
    
    Jacobian::~Jacobian()
    {
        if (data_ != nullptr)
            delete[] data_;
    }

    Map Jacobian::getLinearJacobian() 
    {
        return Map(this->data(), 3, nJoints_);
    };

    Map Jacobian::getAngularJacobian()
    {
        int linear_jacobian_size{3 * nJoints_};
        ///This operation returns a new Map of angular Jacobian and also updates the angular part of the complete Jacobian
        return Map(this->data() + linear_jacobian_size, 3, nJoints_);
    };

    Jacobian& Jacobian::operator=(const Jacobian &other)
    {
        if (nJoints_ != other.nJoints_)
        {
            throw std::invalid_argument("CANNOT USE = OPERATOR FOR JACOBIANS WITH DIFFERENT SIZE. First size: 6x" 
                                    + std::to_string(nJoints_) +", second size: 6x" + std::to_string(other.nJoints_));
        }

        if (other.data_ == nullptr)
        {
            throw std::invalid_argument("CANNOT USE = OPERATOR FOR JACOBIANS WITH NULL DATA.");
        }
      
        for (int i = 0; i < 6 * nJoints_; ++i)
        {
            data_[i] = other.data_[i];
        }
      
        return *this;
    }

    // void Jacobian::print()
    // {
    //     std::cout << "Jacobian [Linear]" << std::endl;
    //     std::cout << "-----------------" << std::endl;

    //     std::cout << this->getLinearJacobian() << std::endl;

    //     std::cout << "\nJacobian [Angular]" << std::endl;
    //     std::cout << "-----------------" << std::endl;

    //     std::cout << this->getAngularJacobian() << std::endl;
    // }

    // void Jacobian::init(const int nJoints, const double init_value)
    // {
    //     nJoints_ = nJoints;

    //     // linear_jacobian_.setZero(3, nJoints_);
    //     // angular_jacobian_.setZero(3, nJoints_);

    //     data_ = new double[6 * nJoints_];
    //     for (int i = 0; i < 6 * nJoints_; ++i)
    //     {
    //         data_[i] = init_value;
    //     }
    //     new (this) Map(data_, 6, nJoints_);
    // }
}

#endif //_ROBOTLIB_JACOBIAN_CPP_