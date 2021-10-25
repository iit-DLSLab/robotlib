/**
 * @file joint_unit_tests.cpp
 * @brief Unit tests for Joint class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "joint.hpp"

/**
 * @brief Set of unit tests for Joint::getName function
 */
TEST(JointUnitTests, getName)
{
     /**
      * @test Joint name with a complete string
      */
     dls::robotlib::Joint joint{"joint_test"};
     ASSERT_EQ(joint.getName(), "joint_test");

     /**
      * @test Joint name with an empty string
      */
     dls::robotlib::Joint joint_empty_name{""};
     ASSERT_EQ(joint_empty_name.getName(), "");

     /**
      * @test Joint name with a single space character
      */
     dls::robotlib::Joint joint_single_space_name{" "};
     ASSERT_EQ(joint_single_space_name.getName(), " ");
}

/**
 * @brief Set of unit tests for Joint::getParent function
 */
TEST(JointUnitTests, getParent)
{
}

/**
 * @brief Set of unit tests for Joint::getChild function
 */
TEST(JointUnitTests, getChild)
{
}

/**
 * @brief Set of unit tests for Joint::setChild function
 */
TEST(JointUnitTests, setChild)
{
}

/**
 * @brief Set of unit tests for Joint::setParent function
 */
TEST(JointUnitTests, setParent)
{
}