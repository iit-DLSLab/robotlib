#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

TEST(RobotBaseUnitTests, getFramePose)
{
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto q = dummy_quadruped->makeJointState();

    Eigen::Matrix4d frame_pose = dummy_quadruped->getFramePose(q, dummy_quadruped->getLink("TRUNK"), dummy_quadruped->getLink("LF_HAA"));

    Eigen::Matrix4d ground_truth;
    ground_truth.setZero();
    ground_truth(3, 3) = 1;

    ASSERT_EQ(frame_pose, ground_truth);
}

TEST(RobotBaseUnitTests, getFootPose)
{
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto q = dummy_quadruped->makeJointState();

    Eigen::Matrix4d foot_pose = dummy_quadruped->getFootPose(q, dummy_quadruped->getLink("LF_FOOT"));

    Eigen::Matrix4d ground_truth;
    ground_truth.setZero();
    ground_truth(3, 3) = 1;

    ASSERT_EQ(foot_pose, ground_truth);
}

TEST(RobotBaseUnitTests, getFeet)
{
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto feet = dummy_quadruped->getFeet();
}