#include "forward_kinematics.hpp"
#include <gtest/gtest.h>
#include <iostream>

typedef Eigen::Matrix<double, 12, 1> Column12d;
typedef Column12d JointState;

TEST(ForwardKinematicsTest, fkTest1)
{
    std::cout << "TEST: Forward Kinematics" << std::endl;

    dls::robotlib::ForwardKinematics forward_kinematics;
    dls::robotlib::JointState joint_state;
    joint_state << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;

    forward_kinematics.getFootPosition(joint_state, 0);
}