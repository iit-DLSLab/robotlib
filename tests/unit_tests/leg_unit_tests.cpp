/**
 * @file leg_unit_tests.cpp
 * 
 * @brief Unit tests for Leg class
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
 * 1 leg
 * 2 joints/links per leg
 */
robotlib::DummyRobotCreator<1, 2> dummy_robot_creator;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 6> components_names{"Dummy Robot",
                                            "Leg",
                                            "Leg_joint_1",
                                            "Leg_joint_2",
                                            "Leg_link_1",
                                            "Leg_link_2"};

/**
 * @test Dummy robot created with the following structure:
 * 1 leg
 * 1 joints/links per leg
 */
robotlib::DummyRobotCreator<1, 1> dummy_robot_creator_2;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 4> components_names_2{"Dummy Robot",
                                              "Robot leg",
                                              "Leg_joint_1",
                                              "Leg_link_1"};

/**
 * @brief Set of unit tests for Leg::getName function (inherited from LimbBase)
 */
TEST(LegUnitTests, getName)
{
    /**
     * @test Get the leg name (case with a complete string)
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getName(), components_names[1]);
        }
    }

    /**
     * @test Get the leg name (case with a two separate strings)
     */
    {
        auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getName(), components_names_2[1]);
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getNJoints function (inherited from LimbBase)
 */
TEST(LegUnitTests, getNJoints)
{
    /**
 * @test Get the number of joints for a leg (case with two joints)
 */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getNJoints(), 2);
        }
    }

    /**
 * @test Get the number of joints for a leg (case with one joint)
 */
    {
        auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getNJoints(), 1);
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getNLinks function (inherited from LimbBase)
 */
TEST(LegUnitTests, getNLinks)
{
    /**
 * @test Get the number of links for a leg (case with two links)
 */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getNLinks(), 2);
        }
    }

    /**
 * @test Get the number of links for a leg (case with one link)
 */
    {
        auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getNLinks(), 1);
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getJoint function (inherited from LimbBase)
 */
TEST(LegUnitTests, getJoint)
{
    /**
     * @test Get the two leg joints and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getJoint(components_names[2]).getName(), components_names[2]);
            EXPECT_EQ(leg->getJoint(components_names[3]).getName(), components_names[3]);
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getLink function (inherited from LimbBase)
 */
TEST(LegUnitTests, getLink)
{
    /**
     * @test Get the two leg links and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getLink(components_names[4]).getName(), components_names[4]);
            EXPECT_EQ(leg->getLink(components_names[5]).getName(), components_names[5]);
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getEndEffector function (inherited from LimbBase)
 */
TEST(LegUnitTests, getEndEffector)
{
    /**
     * @test Get the leg end-effector (last link) and check its name
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        for (auto& leg : dummy_robot->getLegs())
        {
            EXPECT_EQ(leg->getEndEffector().getName(), components_names[5]);
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getJoints function (inherited from LimbBase)
 */
TEST(LegUnitTests, getJoints)
{
    /**
     * @test Iterate over the two leg joints and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        unsigned int i{0};
        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint.getName(), components_names[2+i]);
            i++;
        }
    }
}

/**
 * @brief Set of unit tests for Leg::getLinks function (inherited from LimbBase)
 */
TEST(LegUnitTests, getLinks)
{
    /**
     * @test Iterate over the two leg links and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        unsigned int i{0};
        for (auto& link : dummy_robot->getLinks())
        {
            EXPECT_EQ(link.getName(), components_names[4+i]);
            i++;
        }
    }
}