#ifndef _ROBOTLIB_JOINT_STATE_HPP_
#define _ROBOTLIB_JOINT_STATE_HPP_

#include <memory>
#include <vector>

#include "limb_data_map.hpp"
#include "joint_data_map.hpp"
#include "joint.hpp"
#include <Eigen/Dense>

namespace robotlib
{

    using JointState = Eigen::VectorXd;

} //namespace robotlib

#endif //_ROBOTLIB_JOINT_STATE_HPP_