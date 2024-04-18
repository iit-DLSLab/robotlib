#ifndef _ROBOTLIB_JACOBIAN_CPP_
#define _ROBOTLIB_JACOBIAN_CPP_

#include "jacobian.hpp"
#include <iostream>

namespace robotlib
{
    Jacobian::Jacobian(const Jacobian& jacobian) 
        : Map(
            [&]() -> double*
            {
                this->data_ =
                    [&]() -> double*
                    {
                        double* data_temp = new double[6 * jacobian.nJoints_];
                        for(auto i{0}; i < 6 * jacobian.nJoints_; i++)
                            data_temp[i] = jacobian.data_[i];
                        return data_temp;
                    }();
                return this->data_;
            }()
            , 6, jacobian.nJoints_)
        , nJoints_(jacobian.nJoints_)   
    {}

    Jacobian::Jacobian(const int nJoints, const double& data) 
        : Map(
            [&]() -> double*
            {
                this->data_ =
                    [&]() -> double*
                    {
                        double* data_temp = new double[6 * nJoints];
                        for(auto i{0}; i < 6 * nJoints; i++)
                            data_temp[i] = data;
                        return data_temp;
                    }();
                return this->data_;
            }()
            , 6, nJoints)
        , nJoints_(nJoints)
    {}

    Jacobian::Jacobian(const std::vector<double>& data) 
        : Map(
            [&]() -> double*
            {
                if(data.size()%6 != 0)
                    throw std::invalid_argument("INITIALIZATION VECTOR SHOULD HAVE MODULUS 6.");
                
                this->data_ =
                    [&]() -> double*
                    {
                        double* data_temp = new double[data.size()];
                        for(auto i{0}; i < data.size(); i++)
                            data_temp[i] = data[i];
                        return data_temp;
                    }();
                return this->data_;
            }()
            , 6, data.size()/6)
        , nJoints_(data.size()/6)
        
    {}
    
    Jacobian::~Jacobian()
    {
        if (data_ != nullptr)
            delete[] data_;
    }

    Map Jacobian::getLinearJacobian() 
    {
        return Map(this->data_, 3, nJoints_);
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

    // Jacobian Jacobian::operator*(std::vector<double>& vec)
    // {

        
    //     // auto temp = this->Map::operator*(Map(vec.data(), 6, 1));
    //     // return this->Map::operator*(Map(vec.data(), 6, 1));


    //     // auto temp = Map(data_, 6, nJoints_) *  Map(vec.data(), nJoints_, 1)

        

    //     return *this * Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(vec.data(), vec.size(), 1);
    //     // Map(vec.data(), nJoints_, 1);
    // }


}

#endif //_ROBOTLIB_JACOBIAN_CPP_