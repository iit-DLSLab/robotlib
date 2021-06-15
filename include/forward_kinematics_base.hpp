#ifndef _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_
#define _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_

#include "leg_data_map.hpp"
#include <Eigen/Dense>

namespace dls{
namespace robot {

class ForwardKinematicsBase
{
    public:
        virtual ~ForwardKinematicsBase(){}

        // TODO: Consider the possibility of using an enum for the LegID
        virtual Eigen::Vector3d getFootPos(const JointState& q, const int LegID& leg) = 0;
};

}
}

#endif // _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_