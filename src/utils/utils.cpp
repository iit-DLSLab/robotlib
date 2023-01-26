#include "utils/utils.hpp"

namespace robotlib
{
    namespace utils
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

        Eigen::Matrix<double, 6,1> computeGravity_B(const Eigen::Matrix3d& b_R_w, const double g)
        {
            Eigen::Matrix<double, 6,1> g_w = Eigen::Matrix<double, 6,1>::Zero();
            Eigen::Matrix<double, 6,1> g_b = Eigen::Matrix<double, 6,1>::Zero();

            // Gravity vector
            g_w(5) = -g;

            // Transform gravity from world to base frame
            g_b.segment(3, 3) = b_R_w*g_w.segment(3, 3);

            return g_b;
        }
    } // namespace utils
} // namespace robotlib