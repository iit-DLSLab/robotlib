/**
 * @file joint_data_map_unit_tests.cpp
 * 
 * @brief Unit tests for JointDataMap classes
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
 * @test Dummy robot created with the following structure:
 * 4 leg
 * 3 joints/links per leg
 */
robotlib::DummyRobotCreator<4, 3> dummy_robot_creator;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 29> components_names{"Dummy Quadruped",
                                            "LF", "RF", "LH", "RH",
                                            //joints
                                            "LF_HAA", "LF_HFE", "LF_KFE",
                                            "RF_HAA", "RF_HFE", "RF_KFE",
                                            "LH_HAA", "LH_HFE", "LH_KFE",
                                            "RH_HAA", "RH_HFE", "RH_KFE",
                                            //links
                                            "LF_ASSEMBLY", "LF_UPPERLEG", "LF_LOWERLEG",
                                            "RF_ASSEMBLY", "RF_UPPERLEG", "RF_LOWERLEG",
                                            "LH_ASSEMBLY", "LH_UPPERLEG", "LH_LOWERLEG",
                                            "RH_ASSEMBLY", "RH_UPPERLEG", "RH_LOWERLEG"};

TEST(JointDataMapUnitTests, Zero)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_map = dummy_robot->makeJointDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    for (auto& joint_pair : joint_map)
    {
        EXPECT_EQ(joint_pair.getData(),  Eigen::Vector3d::Zero());
    }
}


TEST(JointDataMapUnitTests, NonZero)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    Eigen::Vector3d p(1, 2, 3);
    auto joint_map = dummy_robot->makeJointDataMap<Eigen::Vector3d>(p);

    for (auto& joint : dummy_robot->getJoints())
    {
        EXPECT_EQ(joint_map[joint],  p);
    }
}

TEST(JointDataMapUnitTests, Attribution)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_map = dummy_robot->makeJointDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    Eigen::Vector3d p;
    p.setZero();
    for (auto& joint : dummy_robot->getJoints())
    {
        p[0]++;
        p[1]++;
        p[2]++;
        joint_map[joint] = p;

        EXPECT_EQ(joint_map[joint],  p);
    }

    for (auto& joint_pair : joint_map)
    {
        p[0]++;
        p[1]++;
        p[2]++;
        joint_pair = p;

        EXPECT_EQ(joint_pair.getData(),  p);
    }

    p[0]++;
    p[1]++;
    p[2]++;
    joint_map = p;
    for (auto& joint_pair : joint_map)
    {
        EXPECT_EQ(joint_pair.getData(),  p);
    }
}

TEST(JointDataMapUnitTests, Iteration)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_map = dummy_robot->makeJointDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    auto i{0};
    for (auto& elem : joint_map)
    {
        EXPECT_EQ(elem.getKey().getName(), components_names[5+i++]);
    }
}


TEST(JointDataMapUnitTests, Size)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_map = dummy_robot->makeJointDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    auto joint_count{0};
    for (auto& elem : joint_map)
    {
        joint_count++;
    }

    EXPECT_EQ(joint_count,  12);
    EXPECT_EQ(joint_map.size(),  12);
}