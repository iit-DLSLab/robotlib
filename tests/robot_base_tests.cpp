#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

TEST(RobotBaseUnitTests, getNLegs)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto n_legs_dq{dummy_quadruped->getNLEGS()};

    /// Ground truth
    int n_legs_gt{4};

    /// Assert conditions
    ASSERT_EQ(n_legs_dq, n_legs_gt);
    ASSERT_EQ(typeid(n_legs_dq).name(), typeid(n_legs_gt).name());
}

TEST(RobotBaseUnitTests, getNJoints)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto n_joints_dq{dummy_quadruped->getNJOINTS()};

    /// Ground truth
    int n_joints_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_joints_dq, n_joints_gt);
    ASSERT_EQ(typeid(n_joints_dq).name(), typeid(n_joints_gt).name());
}

TEST(RobotBaseUnitTests, getNLinks)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto n_links_dq{dummy_quadruped->getNLINKS()};

    /// Ground truth
    int n_links_gt{8};

    /// Assert conditions
    ASSERT_EQ(n_links_dq, n_links_gt);
    ASSERT_EQ(typeid(n_links_dq).name(), typeid(n_links_gt).name());
}

TEST(RobotBaseUnitTests, getLeg)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    /// Ground truth
    std::array<std::string, 4> legs_gt{{"leg1", "leg2", "leg3", "leg4"}};

    for (int i{0}; i < dummy_quadruped->getNLEGS(); i++)
    {
        auto legs_dq{dummy_quadruped->getLeg(i)};

        /// Assert conditions
        ASSERT_EQ(legs_dq->getName(), legs_gt.at(i));
        ASSERT_EQ(typeid(legs_dq->getName()).name(), typeid(legs_gt.at(i)).name());
    }
}

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

TEST(RobotBaseUnitTests, getFeet)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto joint_state = dummy_quadruped->makeJointState();

    auto feet_dq = dummy_quadruped->getFeet();

    /// Ground truth
    Eigen::Matrix4d foot_pose_gt{};
    foot_pose_gt.setZero();
    foot_pose_gt(3, 3) = 1;

    for (const auto foot : feet_dq)
    {
        Eigen::Matrix4d foot_pose_dq{};
        foot_pose_dq.setZero();
        foot_pose_dq = dummy_quadruped->getFootPose(joint_state, *foot);

        /// Assert conditions
        ASSERT_EQ(foot_pose_dq, foot_pose_gt);
        ASSERT_EQ(typeid(foot_pose_dq).name(), typeid(foot_pose_gt).name());
    }
}

//TEST(RobotBaseUnitTests, getJointsConfiguration)
//{
//    /// Dummy quadruped
//    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();
//
//    /// Ground truth
//
//    /// Assert conditions
//    ASSERT_EQ(1, 1);
//    ASSERT_EQ(typeid(1).name(), typeid(1).name());
//}
//
//TEST(RobotBaseUnitTests, getJointsVelocities)
//{
//    /// Dummy quadruped
//    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();
//
//    /// Ground truth
//
//    /// Assert conditions
//    ASSERT_EQ(1, 1);
//    ASSERT_EQ(typeid(1).name(), typeid(1).name());
//}

TEST(RobotBaseUnitTests, getName)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto name_dq = dummy_quadruped->getName();

    /// Ground truth
    std::string name_gt{"Quadruped"};

    /// Assert conditions
    ASSERT_EQ(name_dq, name_gt);
    ASSERT_EQ(typeid(name_dq).name(), typeid(name_gt).name());
}