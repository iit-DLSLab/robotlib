/**
 * @file robot_base_unit_tests.cpp
 * 
 * @brief Unit tests for RobotBase class
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "dummy_robot/dummy_robot_creator.hpp"


/**
 * @test Dummy robot
 */
robotlib::DummyRobotCreator dummy_robot_creator;
std::vector<std::map<std::string,std::vector<std::string>>> limbs {
    {
        {"name", {"LF"}},
        {"joints", {"LF_HAA", "LF_HFE", "LF_KFE"}},
        {"links",  {"LF_ASSEMBLY", "LF_UPPERLEG", "LF_LOWERLEG"}},
        {"type", {"leg"}}
    },
    {
        {"name", {"RF"}},
        {"joints", {"RF_HAA", "RF_HFE", "RF_KFE"}},
        {"links",  {"RF_ASSEMBLY", "RF_UPPERLEG", "RF_LOWERLEG"}},
        {"type", {"leg"}}
    },
    {
        {"name", {"LH"}},
        {"joints", {"LH_HAA", "LH_HFE", "LH_KFE"}},
        {"links",  {"LH_ASSEMBLY", "LH_UPPERLEG", "LH_LOWERLEG"}},
        {"type", {"leg"}}
    },
    {
        {"name", {"RH"}},
        {"joints", {"RH_HAA", "RH_HFE", "RH_KFE"}},
        {"links",  {"RH_ASSEMBLY", "RH_UPPERLEG", "RH_LOWERLEG"}},
        {"type", {"leg"}}
    }
};

const std::string robot_name{"Dummy Quadruped"};



TEST(RobotBaseUnitTests, getNLegs)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto n_legs_dq{dummy_robot->getNLEGS()};

    /// Ground truth
    unsigned int n_legs_gt{4};

    /// Assert conditions
    ASSERT_EQ(n_legs_dq, n_legs_gt);
    ASSERT_EQ(typeid(n_legs_dq), typeid(n_legs_gt));
}

TEST(RobotBaseUnitTests, getNJoints)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto n_joints_dq{dummy_robot->getNJOINTS()};

    /// Ground truth
    unsigned int n_joints_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_joints_dq, n_joints_gt);
    ASSERT_EQ(typeid(n_joints_dq), typeid(n_joints_gt));
}

TEST(RobotBaseUnitTests, getNLinks)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
    
    auto n_links_dq{dummy_robot->getNLINKS()};

    /// Ground truth
    unsigned int n_links_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_links_dq, n_links_gt);
    ASSERT_EQ(typeid(n_links_dq), typeid(n_links_gt));
}

TEST(RobotBaseUnitTests, getLeg)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    /// Ground truth
    std::array<std::string, 4> legs_gt{{"LF", "RF", "LH", "RH"}};

    int count_legs {0};
    for (auto& legs_dq : dummy_robot->getLegs())
    {
        /// Assert conditions
        ASSERT_EQ(legs_dq->getName(), legs_gt.at(count_legs));
        ASSERT_EQ(typeid(legs_dq->getName()), typeid(legs_gt.at(count_legs)));
        count_legs++;
    }
}

TEST(RobotBaseUnitTests, computeFramePosition)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Vector3d frame_position = dummy_robot->computeFramePosition(joint_state,
                                                                   dummy_robot->getTrunk(),
                                                                   dummy_robot->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(frame_position, Eigen::Vector3d().Zero());
}

TEST(RobotBaseUnitTests, computeFrameOrientation)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Matrix3d frame_orientation = dummy_robot->computeFrameOrientation(joint_state,
                                                                         dummy_robot->getTrunk(),
                                                                         dummy_robot->getLink("LF_LOWERLEG"));

    Eigen::Matrix3d rotation_matrix_gt = Eigen::Matrix3d().Zero();
    //set diagonal to 1 using eigen method
    rotation_matrix_gt.diagonal().setOnes();

    /// Assert conditions
    ASSERT_EQ(frame_orientation, rotation_matrix_gt);
}

TEST(RobotBaseUnitTests, getLink)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto link = dummy_robot->getLink("LF_ASSEMBLY");

    /// Ground truth
    robotlib::Link link_gt("LF_ASSEMBLY"); //TODO : generalize the test for all the links

    /// Assert conditions
    ASSERT_EQ(link->getName(), link_gt.getName());
    ASSERT_EQ(typeid(*link), typeid(link_gt));
}

TEST(RobotBaseUnitTests, getJoint)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
    
    auto joint_dq = dummy_robot->getJoint("LF_HFE");

    /// Ground truth
    robotlib::Joint joint_gt("LF_HFE");

    /// Assert conditions
    ASSERT_EQ(joint_dq->getName(), joint_gt.getName());
    ASSERT_EQ(typeid(*joint_dq), typeid(joint_gt));
}

TEST(RobotBaseUnitTests, getName)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto name_dq = dummy_robot->getName();

    /// Ground truth
    std::string name_gt{"Dummy Quadruped"};

    /// Assert conditions
    ASSERT_EQ(name_dq, name_gt);
    ASSERT_EQ(typeid(name_dq), typeid(name_gt));
}

TEST(RobotBaseUnitTests, limb_getEndEffector)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    for (auto& leg : dummy_robot->getLegs())
    {
        ASSERT_TRUE(leg->getEndEffector()->getName().contains("LOWERLEG"));   
    }
}


TEST(RobotBaseUnitTests, jointStateOperators)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
    auto q = dummy_robot->makeJointState();
    double value_gt {10};
    
    /// Operator[]
    for (auto& joint : dummy_robot->getJoints())
    {
        q[joint->id] = value_gt;
    }
}

TEST(RobotBaseUnitTests, getJointLimits)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
    
    // Ground truth for the dummy quadruped
    const double q_min_gt {0};
    const double q_max_gt {90};
    const double qd_max_gt {3};
    const double tau_max_gt {5};
    // Set the joint limits for each joint in the dummy robot
    for (auto& joint : dummy_robot->getJoints())
    {
        joint->setJointLimits(q_min_gt, q_max_gt, qd_max_gt, tau_max_gt);
    }

    auto q_min = dummy_robot->makeJointState();
    auto q_max = dummy_robot->makeJointState();
    auto qd_max = dummy_robot->makeJointState();
    auto tau_max = dummy_robot->makeJointState();

    // compare joint limits of dummy robot joints and q_min
    dummy_robot->getMinJointAngle(q_min);
    dummy_robot->getMaxJointAngle(q_max);
    dummy_robot->getMaxJointVelocity(qd_max);
    dummy_robot->getMaxJointEffort(tau_max);

    // TODO: override operator == for dataMap
    for (auto& joint : dummy_robot->getJoints())
    {
        EXPECT_EQ(q_min_gt, q_min[joint->id]);
        EXPECT_EQ(q_max_gt, q_max[joint->id]);
        EXPECT_EQ(qd_max_gt, qd_max[joint->id]);
        EXPECT_EQ(tau_max_gt, tau_max[joint->id]);
    }
}


TEST(RobotBaseUnitTests, minJointValue)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto q = dummy_robot->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_min_gt {-10};
    q[dummy_robot->getJoint("LF_HAA")->id] = q_min_gt;
    q[dummy_robot->getJoint("LF_HFE")->id] = 10;
    q[dummy_robot->getJoint("LH_HAA")->id] = 5;
    
    EXPECT_EQ(q.minCoeff(), q_min_gt);
}


TEST(RobotBaseUnitTests, maxJointValue)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto q = dummy_robot->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_max_gt = 10;
    q[dummy_robot->getJoint("LF_HAA")->id] = q_max_gt;
    q[dummy_robot->getJoint("LF_HFE")->id] = -10;
    q[dummy_robot->getJoint("LH_HAA")->id] = 5;
    
    EXPECT_EQ(q.maxCoeff(), q_max_gt);
}
