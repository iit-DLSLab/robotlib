/**
 * @file limb_data_map_unit_tests.cpp
 * 
 * @brief Unit tests for LimbDataMap classes
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


TEST(LimbDataMapUnitTests, Zero)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto limb_map = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    for (auto& limb : dummy_robot->getLimbs())
    {
        EXPECT_EQ(limb_map[limb],  Eigen::Vector3d::Zero());
    }
}

TEST(LimbDataMapUnitTests, NonZero)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    Eigen::Vector3d p(1, 2, 3);
    auto limb_map = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(p);

    for (auto& limb : dummy_robot->getLimbs())
    {
        EXPECT_EQ(limb_map[limb],  p);
    }
}

TEST(LimbDataMapUnitTests, Attribution)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto limb_map = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    Eigen::Vector3d p;
    p.setZero();
    for (auto& limb : dummy_robot->getLimbs())
    {
        p[0]++;
        p[1]++;
        p[2]++;
        limb_map[limb] = p;

        EXPECT_EQ(limb_map[limb],  p);
    }
}


TEST(LimbDataMapUnitTests, Iteration)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto limb_map = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    auto i{0};
    for (auto& elem : limb_map)
    {
        EXPECT_EQ(elem.getKey().getName(), components_names[1+i++]);
    }
}


TEST(LimbDataMapUnitTests, Size)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto limb_map = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    auto limb_count{0};
    for (auto& elem : limb_map)
    {
        limb_count++;
    }

    EXPECT_EQ(limb_count,  4);
    EXPECT_EQ(limb_map.size(),  4);
}