/**
 * @file joint_unit_tests.cpp
 * 
 * @brief Unit tests for Joint class
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
 * 2 joints per leg
 * 2 links per leg
 */
robotlib::DummyRobotCreator<1, 2, 2> dummy_robot_creator;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 7> components_names{"Dummy Robot",
                                            "TRUNK",
                                            "Leg",
                                            "Leg_joint_1", "Leg_joint_2",
                                            "Leg_link_1", "Leg_link_2"};

/**
 * @brief Set of unit tests for Joint::getName function
 */
TEST(JointUnitTests, getName)
{
    /**
     * @test Joint name with a complete string
     */
    {
        robotlib::Joint joint{"joint_test"};
        EXPECT_EQ(joint.getName(), "joint_test");
    }

    /**
     * @test Joint name with two separate words
     */
    {
        robotlib::Joint joint{"joint test"};
        EXPECT_EQ(joint.getName(), "joint test");
    }

    /**
     * @test Joint name with an empty string
     */
    {
        robotlib::Joint joint{""};
        EXPECT_EQ(joint.getName(), "");
    }

    /**
     * @test Joint name with a single space character
     */
    {
        robotlib::Joint joint{" "};
        EXPECT_EQ(joint.getName(), " ");
    }

    /**
     * @test Dummy robot joints names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        unsigned int i {0};
        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint->getName(), components_names.at(3+i));
            i++;
        }
    }
}

/**
 * @brief Set of unit tests for Joint::getParent function
 */
TEST(JointUnitTests, getParent)
{
    /**
     * @test Get the two joints parents and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        unsigned int i {0};
        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ((joint->getParent()).getName(), components_names.at(1+i));
            i=i+4;
        }           
    }
}

/**
 * @brief Set of unit tests for Joint::getChild function
 */
TEST(JointUnitTests, getChild)
{
    /**
     * @test Get the two joints children and check their names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        unsigned int i {0};
        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ((joint->getChild()).getName(), components_names.at(5+i));
            i++;
        }
    }
}

/**
 * @brief Set of unit tests for Joint::getMinAngle
 */
TEST(JointUnitTests, getMinAngle)
{
    /**
     * @test Get the dummy robot joint minimum angle
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint->getMinAngle(), 0);
        }
    }
}
/**
 * @brief Set of unit tests for Joint::getMaxAngle
 */
TEST(JointUnitTests, getMaxAngle)
{
    /**
     * @test Get the dummy robot joint maximum angle
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint->getMaxAngle(), 90);
        }
    }
}
/**
 * @brief Set of unit tests for Joint::getMaxVelocity
 */
TEST(JointUnitTests, getMaxVelocity)
{
     /**
      * @test Get the dummy robot joint maximum velocity
      */
     {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint->getMaxVelocity(), 3);
        }
     }
}
/**
 * @brief Set of unit tests for Joint::getMaxEffort
 */
TEST(JointUnitTests, getMaxEffort)
{
    /**
     * @test Get the dummy robot joint maximum effort
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint->getMaxEffort(), 5);
        }
    }
}