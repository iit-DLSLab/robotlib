#ifndef _ROBOTLIB_JACOBIAN_CPP_
#define _ROBOTLIB_JACOBIAN_CPP_

#include "jacobian.hpp"
#include <iostream>

namespace robotlib
{
    Jacobian::Jacobian(const Jacobian& jacobian) 
        : Matrix(6,jacobian.cols())
    {
        *this = jacobian;
    }

    Jacobian::Jacobian() 
        : Matrix(6, 0)
    {}

    Jacobian::Jacobian(const int nJoints, const double& data) 
        : Matrix(6,nJoints)
    {
        this->setConstant(data);        
    }

    Jacobian::~Jacobian()
    {
    }

    Matrix Jacobian::getLinearJacobian() 
    {
        return this->block(0, 0, 3, this->cols());
    };

    Matrix Jacobian::getAngularJacobian()
    {
        return this->block(3, 0, 3, this->cols());
    };

}

#endif //_ROBOTLIB_JACOBIAN_CPP_