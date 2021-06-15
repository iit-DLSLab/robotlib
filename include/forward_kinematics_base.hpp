#ifndef _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_
#define _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_

#include "leg_data_map.hpp"
#include <Eigen/Dense>

namespace dls{
namespace robot {

/// --> TODO: Remove
static const int fbjointsCount = 18;
typedef Eigen::Matrix<double, fbjointsCount, 1> Column18d;
typedef Column18d FloatingBaseJointState;

typedef Eigen::Matrix<double, 12, 1> Column12d;
typedef Column12d JointState;
typedef Eigen::Array<bool, 3, 1> LegJointBool;
/// <--

class ForwardKinematicsBase
{
    public:
        virtual ~ForwardKinematicsBase(){}

        // TODO: Consider the possibility of using an enum for the LegID
        virtual Eigen::Vector3d getFootPos(const JointState& q, const int leg) = 0;
};

}
}

#endif // _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_