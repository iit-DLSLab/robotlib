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
    const dls::robotlib::LimbBase *l;             //TODO: set right parentLimb
    dls::robotlib::Link link_gt(l, "leg1_link1"); //TODO : generalize the test for all the links

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

TEST(RobotBaseUnitTests, getParentLimb)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    auto link_dq = dummy_quadruped->getLink("leg1_link1");

    auto parentLimb_link_dq = link_dq.getParentLimb();
    /// Ground truth
    dls::robotlib::LimbBase *parentLimb_link_dq_gt(dummy_quadruped->getLeg(0).get()); //TODO: extend to all dummy structure

    /// Assert conditions
    ASSERT_EQ(parentLimb_link_dq->getName(), parentLimb_link_dq_gt->getName());
    //ASSERT_EQ(typeid(parentLimb_link_dq).name(), typeid(parentLimb_link_dq_gt).name()); // they are raw pointers so you may not use this assert
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

TEST(RobotBaseUnitTests, makeJacobian)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    dls::robotlib::Link foot = dummy_quadruped->getLink("Leg1_link1");

    auto jacobian = dummy_quadruped->makeFootJacobian(foot);

    const int nJoints = foot.getParentLimb()->getNJoints();

    int count = 0;
    for (int i = 0; i < jacobian.rows(); ++i)
    {
        for (int j = 0; j < jacobian.cols(); ++j)
        {
            jacobian(i, j) = count;
            count++;
        }
    }

    /// Ground truth
    double *data_gt = new double[6 * nJoints];
    for (int i = 0; i < 6 * nJoints; ++i)
    {
        data_gt[i] = 0;
    }

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian_gt(data_gt, 6, nJoints);
    count = 0;
    for (int i = 0; i < jacobian_gt.rows(); ++i)
    {
        for (int j = 0; j < jacobian_gt.cols(); ++j)
        {
            jacobian_gt(i, j) = count;
            count++;
        }
    }

    auto linearJacobian = jacobian.getLinearJacobian();
    auto angularJacobian = jacobian.getAngularJacobian();

    /// Assert conditions
    ASSERT_EQ(jacobian, jacobian_gt);
    ASSERT_EQ(jacobian.getLinearJacobian(), jacobian_gt.block(0, 0, 3, nJoints));
    ASSERT_EQ(jacobian.getAngularJacobian(), jacobian_gt.block(3, 0, 3, nJoints));
    //ASSERT_EQ(typeid(jacobian).name(), typeid(jacobian_gt).name());
}

TEST(RobotBaseUnitTests, makeFootJacobian)
{
    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();

    dls::robotlib::Link foot = dummy_quadruped->getLink("Leg1_link1");

    dummy_quadruped->makeJacobian(foot, foot);
}
