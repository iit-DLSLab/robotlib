#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

TEST(RobotBaseUnitTests, getTransform)
{
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto q = dummy_quadruped->makeJointState();

    Eigen::Matrix4d transform = dummy_quadruped->getFramePose(q, dummy_quadruped->getLink("TRUNK"), dummy_quadruped->getLink("LF_HAA"));

    Eigen::Matrix4d ground_truth;
    ground_truth.setZero();
    ground_truth(3, 3) = 1;

    ASSERT_EQ(transform, ground_truth);
}