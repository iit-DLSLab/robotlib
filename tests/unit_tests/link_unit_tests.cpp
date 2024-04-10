/**
 * @file link_unit_tests.cpp
 * 
 * @brief Unit tests for Link class
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
 * 2 legs
 * 1 joints/links per leg
 */
robotlib::DummyRobotCreator<2, 1> dummy_robot_creator_2;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 7> components_names_2{"Dummy Robot",
                                              "Robot leg",
                                              "Robot leg2",
                                              "Leg_joint_1",
                                              "Leg_joint_2",
                                              "Leg_link_1",
                                              "Leg_link_2"};

/**
 * @brief Set of unit tests for Link::getName function
 */
TEST(LinkUnitTests, getName)
{
    /**
     * @test Link name with a complete string
     */
    {
        robotlib::Link link{"link_test"};
        EXPECT_EQ(link.getName(), "link_test");
    }

    /**
     * @test Link name with two separate words
     */
    {
        robotlib::Link link{"link test"};
        EXPECT_EQ(link.getName(), "link test");
    }

    /**
     * @test Link name with an empty string
     */
    {
        robotlib::Link link{""};
        EXPECT_EQ(link.getName(), "");
    }

    /**
     * @test Link name with a single space character
     */
    {
        robotlib::Link link{" "};
        EXPECT_EQ(link.getName(), " ");
    }

    /**
     * @test Dummy robot links names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        unsigned int i {0};
        for (auto& link : dummy_robot->getLinks())
        {
            EXPECT_EQ(link.getName(), components_names[4+i]);
            i++;
        }
    }
}

/**
 * @brief Set of unit tests for Link::getParent function
 */
TEST(LinkUnitTests, getParent)
{
    /**
     * @test Get the two links parents and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        unsigned int i {0};
        for (auto& link : dummy_robot->getLinks())
        {
            EXPECT_EQ((link.getParent())->getName(), components_names[2+i]);
            i++;
        }
    }
}

/**
 * @brief Set of unit tests for Link::getChildren function
 */
TEST(LinkUnitTests, getChildren)
{
    /**
     * @test Iterate over the links children (one child for each link)
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        for (auto& link : dummy_robot->getLinks())
        {
            if((link.getName()).compare(components_names[5]) != 0)
                    EXPECT_EQ(link.getChildren().size(), 1);
            else
                    EXPECT_EQ(link.getChildren().size(), 0);
        }
    }

    /**
     * @test Iterate over the link (trunk) children (case with one child)
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        auto trunk_link{dummy_robot->getTrunk()};

        EXPECT_EQ((trunk_link.getChildren().size()), 1);

        for (auto& trunk_child : trunk_link.getChildren())
        {
            EXPECT_EQ(trunk_child->getName(), components_names[2]);
        }
    }

    /**
     * @test Iterate over the link (trunk) children (case with two children)
     */
    {
        auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

        auto trunk_link{dummy_robot->getTrunk()};

        EXPECT_EQ((trunk_link.getChildren().size()), 2);

        unsigned int i{0};
        for (auto& trunk_child : trunk_link.getChildren())
        {
            EXPECT_EQ(trunk_child->getName(), components_names_2[3+i]);
            i++;
        }
    }
}