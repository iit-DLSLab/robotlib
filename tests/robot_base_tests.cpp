#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

TEST(RobotBaseUnitTests, getFramePosition)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Vector3d frame_position = dummy_quadruped->getFramePosition(joint_state,
                                                                       dummy_quadruped->getLink("TRUNK"),
                                                                       dummy_quadruped->getLink("LF_FOOT"));

    /// Assert conditions
    ASSERT_EQ(frame_position, Eigen::Vector3d().setZero());
}

TEST(RobotBaseUnitTests, getFrameOrientation)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix3d frame_orientation = dummy_quadruped->getFrameOrientation(joint_state,
                                                                             dummy_quadruped->getLink("TRUNK"),
                                                                             dummy_quadruped->getLink("LF_FOOT"));

    /// Assert conditions
    ASSERT_EQ(frame_orientation, Eigen::Matrix3d().setZero());
}

TEST(RobotBaseUnitTests, getFramePose)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix4d frame_pose_dq = dummy_quadruped->getFramePose(joint_state,
                                                                  dummy_quadruped->getLink("TRUNK"),
                                                                  dummy_quadruped->getLink("LF_HAA"));
    /// Ground truth
    Eigen::Matrix4d frame_pose_gt;
    frame_pose_gt.setZero();
    frame_pose_gt(3, 3) = 1;

    /// Assert conditions
    ASSERT_EQ(frame_pose_dq, frame_pose_gt);
    ASSERT_EQ(typeid(frame_pose_dq).name(), typeid(frame_pose_gt).name());
}

TEST(RobotBaseUnitTests, getFramePose_getFeet)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    auto feet_dq = dummy_quadruped->getFeet();

    /// Ground truth
    Eigen::Matrix4d foot_pose_gt;
    foot_pose_gt.setZero();
    foot_pose_gt(3, 3) = 1;

    for (auto foot : feet_dq)
    {
        Eigen::Matrix4d foot_pose_dq;
        foot_pose_dq.setZero();

        foot_pose_dq = dummy_quadruped->getFootPose(joint_state, *foot);

        /// Assert conditions
        ASSERT_EQ(foot_pose_dq, foot_pose_gt);
        ASSERT_EQ(typeid(foot_pose_dq).name(), typeid(foot_pose_gt).name());
    }
}

TEST(RobotBaseUnitTests, getFootPosition)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Vector3d foot_position = dummy_quadruped->getFootPosition(joint_state,
                                                                     dummy_quadruped->getLink("LF_FOOT"));

    /// Assert conditions
    ASSERT_EQ(foot_position, Eigen::Vector3d().setZero());
}

TEST(RobotBaseUnitTests, getFootOrientation)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix3d foot_orientation = dummy_quadruped->getFootOrientation(joint_state,
                                                                           dummy_quadruped->getLink("LF_FOOT"));

    /// Assert conditions
    ASSERT_EQ(foot_orientation, Eigen::Matrix3d().setZero());
}

TEST(RobotBaseUnitTests, getFootPose)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix4d foot_pose_dq = dummy_quadruped->getFootPose(joint_state,
                                                                dummy_quadruped->getLink("LF_FOOT"));

    /// Ground truth
    Eigen::Matrix4d foot_pose_gt;
    foot_pose_gt.setZero();
    foot_pose_gt(3, 3) = 1;

    /// Assert conditions
    ASSERT_EQ(foot_pose_dq, foot_pose_gt);
    ASSERT_EQ(typeid(foot_pose_dq).name(), typeid(foot_pose_gt).name());
}

TEST(RobotBaseUnitTests, getLink)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto link_dq = dummy_quadruped->getLink("link");

    /// Ground truth
    dls::robotlib::Link link_gt("link");

    /// Assert conditions
    ASSERT_EQ(link_dq.getName(), link_gt.getName());
    ASSERT_EQ(typeid(link_dq).name(), typeid(link_gt).name());
}

TEST(RobotBaseUnitTests, getJoint)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_dq = dummy_quadruped->getJoint("joint");

    /// Ground truth
    dls::robotlib::Joint joint_gt(nullptr, nullptr, "joint");

    /// Assert conditions
    ASSERT_EQ(joint_dq.getName(), joint_gt.getName());
    ASSERT_EQ(typeid(joint_dq).name(), typeid(joint_gt).name());
}