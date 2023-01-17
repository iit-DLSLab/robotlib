#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

#include "leg_data_map.hpp"

#include <memory>
#include <urdf_model/pose.h>
#include <Eigen/Dense>

namespace robotlib
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

    // /*! 
    // * @brief Compute the number of legs in stance
    // * @param stance_legs 
    // * @return
    // */ 
    // int compute_stance_legs(const LegDataMap<bool>& stance_legs);

    // /*! 
    // * @brief Computing robot proprio height
    // * @details
    // * If none of the legs are in contact with the ground, the function returns the actual proprio height taken in input.
    // * @param[in] w_rpy_b orientation of the base frame in world coordinates
    // * @param[in] stance_legs variable identifying the stance status of each leg
    // * @param[in] actual_foot_position actual foot position
    // * @param[in] actual_proprio_height actual proprio height
    // * @return robot proprio height
    // */
    // double computeProprioHeight(const Eigen::Vector3d w_rpy_b, const LegDataMap<bool>& stance_legs, const robotlib::LegDataMap<Eigen::Vector3d>& actual_foot_position, const double actual_proprio_height);

    // /*!
    // * @brief Compute gravity term in base frame
    // * @param b_R_w orientation matrix expressing the orientation of world frame w.r.t the base frame in base coordinates 
    // * @return gravity term in base frame
    // */ 
    // Eigen::Vector6d computeGravity(const Eigen::Matrix3d& b_R_w);

} // namespace robotlib

#include "utils.tpp"

#endif // _ROBOTLIB_UTILS_HPP_