#include "utils.hpp"

namespace robotlib
{
    Eigen::Matrix4d get_eigen_matrix4d_from_urdf_pose(urdf::Pose urdf_pose)
    {
        Eigen::Matrix4d eigen_pose{Eigen::Matrix4d::Zero()};
        eigen_pose(3,3) = 1; //homogeneous definition
        eigen_pose(0,3) = urdf_pose.position.x;
        eigen_pose(1,3) = urdf_pose.position.y;
        eigen_pose(2,3) = urdf_pose.position.z;
        Eigen::Quaterniond urdf_quaternion{urdf_pose.rotation.w, urdf_pose.rotation.x, urdf_pose.rotation.y, urdf_pose.rotation.z};
        eigen_pose.block<3,3>(0,0) = quatToRotMat(urdf_quaternion).transpose();
        return eigen_pose;
    }
} // namespace robotlib