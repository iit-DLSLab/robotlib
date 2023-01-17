#include "utils/utils.hpp"

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

    // int compute_stance_legs(const robotlib::LegDataMap<bool>& stance_legs)
    // {
    //     int leg_count{0};
    //     for(auto leg_pair: stance_legs)
    //     {
    //         if (stance_legs[leg_pair.key_])
    //             leg_count++;
    //     }
    //     return leg_count;
    // }

    // double computeProprioHeight(const Eigen::Vector3d w_rpy_b, const LegDataMap<bool>& stance_legs, const LegDataMap<Eigen::Vector3d>& actual_foot_position, const double actual_proprio_height)
    // {
    //     double proprio_height = actual_proprio_height; 
    //     int n_stance_legs{compute_stance_legs(stance_legs)};
    //     if(n_stance_legs>0)
    //     {
    //         proprio_height = 0.0;

    //         //Compute foot position in horizontal frame
    //         auto actual_foot_pos_HF = this->pRobot->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
    //         auto HF_R_b = dls::math::rpyToRot(Eigen::Vector3d(w_rpy_b[0], w_rpy_b[1], 0.0)).transpose();
    //         for(auto leg_pair : actual_foot_pos_HF)
    //         {
    //             actual_foot_pos_HF[leg_pair.key_] = HF_R_b*actual_foot_position[leg_pair.key_];
    //         }
    //         // Compute proprio height
    //         for(auto leg_pair : stance_legs)
    //         {
    //             proprio_height += (-actual_foot_pos_HF[leg_pair.key_](2) * (stance_legs[leg_pair.key_]))/n_stance_legs;
    //         }
    //     }
    //     return proprio_height;
    // }

    // Eigen::Vector6d TrunkController::computeGravity(const Eigen::Matrix3d& b_R_w)
    // {
    //     Eigen::Vector6d g_w = Eigen::Vector6d::Zero();
    //     Eigen::Vector6d g_b = Eigen::Vector6d::Zero();

    //     // Gravity vector
    //     g_w(LZ) = -g;

    //     // Transform gravity from world to base frame
    //     g_b.segment(LX, 3) = b_R_w*g_w.segment(LX, 3);

    //     return g_b;
    // }
} // namespace robotlib