#ifndef _ROBOTLIB_JOINT_DATA_MAP_HPP_
#define _ROBOTLIB_JOINT_DATA_MAP_HPP_

#include "joint.hpp"
#include <map>
namespace robotlib
{
    template <class Data>
    using JointDataMap =  std::map<JointPtr, Data>;
}

// template <class Data>
// robotlib::JointDataMap<Data> operator*(const Eigen::VectorXd &vec, const robotlib::JointDataMap<Data> &state);

// template <class Data>
// robotlib::JointDataMap<Data> operator*(const double &esc, const robotlib::JointDataMap<Data> &state);

// #include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_HPP_