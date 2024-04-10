/**
 * @file arm_unit_tests.cpp
 * 
 * @brief Unit tests for Arm class
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
 * 1 arm
 * 2 joints/links per arm
 */
robotlib::DummyRobotCreator<0, 0, 1, 2> dummy_robot_creator;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 6> components_names{"Dummy Robot",
                                            "Arm",
                                            "Arm_joint_1",
                                            "Arm_joint_2",
                                            "Arm_link_1",
                                            "Arm_link_2"};

/**
 * @test Dummy robot created with the following structure:
 * 1 arm
 * 1 joints/links per arm
 */
robotlib::DummyRobotCreator<0, 0, 1, 1> dummy_robot_creator_2;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 4> components_names_2{"Dummy Robot",
                                              "Robot arm",
                                              "Arm_joint_1",
                                              "Arm_link_1"};

/**
 * @brief Set of unit tests for Leg::getName function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getName)
{
     /**
      * @test Get the arm name (case with a complete string)
      */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getName(), components_names[1]);
          }
     }

     /**
      * @test Get the arm name (case with a two separate strings)
      */
     {
          auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getName(), components_names_2[1]);
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getNJoints function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getNJoints)
{
     /**
     * @test Get the number of joints for a arm (case with two joints)
     */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getNJoints(), 2);
          }
     }

     /**
     * @test Get the number of joints for a arm (case with one joint)
     */
     {
          auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getNJoints(), 1);
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getNLinks function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getNLinks)
{
     /**
     * @test Get the number of links for a arm (case with two links)
     */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getNLinks(), 2);
          }
     }

     /**
     * @test Get the number of links for a arm (case with one link)
     */
     {
          auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getNLinks(), 1);
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getJoint function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getJoint)
{
     /**
      * @test Get the two arm joints and check their names
      */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getJoint(components_names[2]).getName(), components_names[2]);
               EXPECT_EQ(arm->getJoint(components_names[3]).getName(), components_names[3]);
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getLink function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getLink)
{
     /**
      * @test Get the two arm links and check their names
      */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getLink(components_names[4]).getName(), components_names[4]);
               EXPECT_EQ(arm->getLink(components_names[5]).getName(), components_names[5]);
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getEndEffector function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getEndEffector)
{
     /**
      * @test Get the arm end-effector (last link) and check its name
      */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          for (auto& arm : dummy_robot->getArms())
          {
               EXPECT_EQ(arm->getEndEffector().getName(), components_names[5]);
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getJoints function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getJoints)
{
     /**
      * @test Iterate over the two arm joints and check their names
      */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          unsigned int i{0};
          for (auto& arm : dummy_robot->getArms())
          {
               for (auto& joint : arm->getJoints())
               {
                    EXPECT_EQ(joint.getName(), components_names[2+i]);
                    i++;
               }
          }
     }
}

/**
 * @brief Set of unit tests for Leg::getLinks function (inherited from LimbBase)
 */
TEST(ArmUnitTests, getLinks)
{
     /**
      * @test Iterate over the two arm links and check their names
      */
     {
          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          unsigned int i{0};
          for (auto& arm : dummy_robot->getArms())
          {
               for (auto& link : arm->getLinks())
               {
                    EXPECT_EQ(link.getName(), components_names[4+i]);
                    i++;
               }
          }
     }
}