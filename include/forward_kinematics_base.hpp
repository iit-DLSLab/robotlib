#ifndef _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_
#define _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_

#include <Eigen/Dense>
#include "declarations.hpp"

namespace dls
{
    namespace robotlib
    {
        class ForwardKinematicsBase
        {
        public:
            ForwardKinematicsBase(){};
            virtual ~ForwardKinematicsBase(){};

            // TODO: Consider the possibility of using an enum for the LegID
            virtual Eigen::Vector3d getFootPosition(const JointState &q, const int leg) = 0;
            virtual Eigen::Matrix3d getFootOrientation(const JointState &q, const int leg) = 0;

            virtual Eigen::Vector3d getLinkPosition(const JointState &q, const int leg) = 0;
            virtual Eigen::Matrix3d getLinkOrientation(const JointState &q, const int leg) = 0;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_FORWARD_KINEMATICS_BASE_HPP_