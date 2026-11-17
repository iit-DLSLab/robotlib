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
 * 2 joints/links per leg
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        // get names of joints
        std::vector<std::string> joints_name;
        for(auto& limb : limbs)
        {
            for(const auto& joint_name : limb["joints"])
            {
                joints_name.push_back(joint_name);
            }
        }
        unsigned int i {0};
        for (auto& joint : dummy_robot->getJoints())
        {
            EXPECT_EQ(joint->getName(), joints_name[i]);
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        double angle = -90;
        for (auto& joint : dummy_robot->getJoints())
        {
            joint->setJointLimits(angle, 0, 0,0);
            EXPECT_EQ(joint->getMinAngle(), angle);
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        double angle = 90;
        for (auto& joint : dummy_robot->getJoints())
        {
            joint->setJointLimits(0, angle, 0,0);
            EXPECT_EQ(joint->getMaxAngle(), angle);
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        double velocity = 3;
        for (auto& joint : dummy_robot->getJoints())
        {
            joint->setJointLimits(0, 90, velocity,0);
            EXPECT_EQ(joint->getMaxVelocity(), velocity);
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        double effort = 10;
        for (auto& joint : dummy_robot->getJoints())
        {
            joint->setJointLimits(0, 90, 0,effort);
            EXPECT_EQ(joint->getMaxEffort(), effort);
        }
    }
}