#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

#include "robotlib/limb_data_map.hpp"

#include <memory>
#include <urdf_model/pose.h>
#include <Eigen/Dense>

namespace robotlib
{
namespace utils
    {
        /*!
        *@brief Get Eigen matrix from urdf pose.
        *@param[in] urdf_pose pose of type urdf::Pose.
        *@return 4x4 eigen matrix.
        */
        Eigen::Matrix4d get_eigen_matrix4d_from_urdf_pose(urdf::Pose urdf_pose);

        /**
         * @brief converts a quaternion \f$\mathbf{q}\f$ into a rotation matrix
         * \f$R_q(\mathbf{q})\f$. The rotation matrix maps a vector
         * \f$\mathbf{z}\in\mathbb{R}^{3\times1}\f$ (expressed in global coordinates)
         * into a vector \f$ \mathbf{z}' \in \mathbb{R}^{3\times1}\f$ (expressed in
         * local coordinates), such that \f$ \mathbf{z}' = R_q(\mathbf{q})\mathbf{z}\f$.
         *
         * @param[in] q structure containing the quaternion
         * @return the 3 by 3 rotation matrix \f$R_q(\mathbf{q})\f$
         * @remark the function uses the formula (125) from <a href="https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf">"Representing Attitude: Euler
         *  Angles, Unit Quaternions, and Rotation Vectors"</a> by James Diebel.
         * @date July 2005
         */
        inline Eigen::Matrix3d quatToRotMat(const Eigen::Quaterniond & q) {
            Eigen::Matrix3d R;
            R(0, 0) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.x() * q.x());
            R(1, 1) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.y() * q.y());
            R(2, 2) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.z() * q.z());
            R(0, 1) = 2.0 * (q.x() * q.y() + q.w() * q.z());
            R(0, 2) = 2.0 * (q.x() * q.z() - q.w() * q.y());
            R(1, 0) = 2.0 * (q.x() * q.y() - q.w() * q.z());
            R(1, 2) = 2.0 * (q.y() * q.z() + q.w() * q.x());
            R(2, 0) = 2.0 * (q.x() * q.z() + q.w() * q.y());
            R(2, 1) = 2.0 * (q.y() * q.z() - q.w() * q.x());

            return R;
        }

        /**
         * @brief Function to compute the rotation matrix which expresses a vector of
         * the fixed frame A into the rotated frame B according to the ZYX convention
         * (subsequent rotation) considering right hand coordinate systems (clockwise convention)
         * \f[
         * {}_B R_A = \begin{bmatrix}
         * \cos(\psi)\cos(\theta) & \cos(\theta)\sin(\psi) & -\sin(\theta) \\
         * \cos(\psi)\sin(\phi)\sin(\theta) - \cos(\phi)\sin(\psi) & \cos(\phi)\cos(\psi) + \sin(\phi)\sin(\psi)\sin(\theta) & \cos(\theta)\sin(\phi) \\
         * \sin(\phi)\sin(\psi) + \cos(\phi)\cos(\psi)\sin(\theta) & \cos(\phi)\sin(\psi)\sin(\theta) - \cos(\psi)\sin(\phi) & \cos(\phi)\cos(\theta)
         * \end{bmatrix}
         * \f]
         * the transpose of this matrix has as director cosines (columns) the axis of
         * the rotated frame expressed in the fixed frame which will be multiplied for
         * the component of the vector in the rotated frame B to get the components in
         * the fixed frame A
         * @param[in] rpy vector containing roll \f$ \phi\f$, pitch \f$ \theta \f$ and yaw \f$\psi\f$
         * @return the matrix \f${}_B R_A\f$
         *
        */
        Eigen::Matrix3d inline rpyToRot(const Eigen::Vector3d & rpy){

            Eigen::Matrix3d Rx, Ry, Rz;
            double roll, pitch, yaw;

            roll = rpy(0);
            pitch = rpy(1);
            yaw = rpy(2);

            Rx <<	1   ,    0     	  ,  	  0,
                    0   ,    cos(roll) ,  sin(roll),
                    0   ,    -sin(roll),  cos(roll);


            Ry << cos(pitch) 	,	 0  ,   -sin(pitch),
                    0       ,    1  ,   0,
                    sin(pitch) 	,	0   ,  cos(pitch);

            Rz << cos(yaw)  ,  sin(yaw) ,		0,
                    -sin(yaw) ,  cos(yaw) ,  		0,
                    0      ,     0     ,       1;


            return Rx*Ry*Rz;

        }

        /*!
        * @brief Compute gravity term in base frame
        * @param b_R_w orientation matrix expressing the orientation of world frame w.r.t the base frame in base coordinates
        * @param g gravity acceleration
        * @return gravity term in base frame
        */ 
        Eigen::Matrix<double, 6,1> computeGravity_B(const Eigen::Matrix3d& b_R_w, const double g);
    } // namespace utils
} // namespace robotlib

#endif // _ROBOTLIB_UTILS_HPP_