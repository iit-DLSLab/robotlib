/**
 * @file dummy_robot_unit_tests.cpp
 * @brief Unit tests for DummyRobot class
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
#include "dummy_robot/dummy_robot.hpp"

/**
 * @brief Set of unit tests for DummyRobot
 */
TEST(DummyRobotUnitTests, test)
{
     /**
      * @test Dummy robot creation and hierarchy like the dummy quadruped
      * 4 legs
      * 0 arms
      * 3 joints per leg
      * 3 links per leg
      * 0 joints per arm
      * 0 links per arm
      */
     {
          robotlib::DummyRobotCreator<12, 12, 4, 3, 3> dummy_robot_creator;

          /* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
          std::array<std::string, 30> components_names{"Dummy Robot",
                                                       "TRUNK",
                                                       "LF", "RF", "LH", "RH",
                                                       "LF_joint_1", "LF_joint_2", "LF_joint_3",
                                                       "RF_joint_1", "RF_joint_2", "RF_joint_3",
                                                       "LH_joint_1", "LH_joint_2", "LH_joint_3",
                                                       "RH_joint_1", "RH_joint_2", "RH_joint_3",
                                                       "LF_link_1", "LF_link_2", "LF_link_3",
                                                       "RF_link_1", "RF_link_2", "RF_link_3",
                                                       "LH_link_1", "LH_link_2", "LH_link_3",
                                                       "RH_link_1", "RH_link_2", "RH_link_3"};

          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          /*
                                   Robot hierarchy
                                        TRUNK
                                          |
                    ----------------------------------------------
                    |              |              |              |
               LF_joint_1     RF_joint_1     LH_joint_1     RH_joint_1
                    ----------------------------------------------
                    |              |              |              |
               LF_link_1      RF_link_1      LH_link_1      RH_link_1
                    ----------------------------------------------
                    |              |              |              |
               LF_joint_2     RF_joint_2     LH_joint_2     RH_joint_2
                    ----------------------------------------------
                    |              |              |              |
               LF_link_2      RF_link_2      LH_link_2      RH_link_2
                    ----------------------------------------------
                    |              |              |              |
               LF_joint_3     RF_joint_3     LH_joint_3     RH_joint_3
                    ----------------------------------------------
                    |              |              |              |
               LF_link_3      RF_link_3      LH_link_3      RH_link_3
          */

          EXPECT_EQ(dummy_robot->getNLEGS(), 4);
          EXPECT_EQ(dummy_robot->getNARMS(), 0);
          EXPECT_EQ(dummy_robot->getNJOINTS(), 12);
          EXPECT_EQ(dummy_robot->getNLINKS(), 12);

          EXPECT_EQ(dummy_robot->getName(), "Dummy Robot");

          unsigned int i{0}, j{0}, k{0};
          for (auto leg : *(dummy_robot->getLegs()))
          {
               EXPECT_EQ(leg->getName(), components_names.at(2+i));
               i++;

               for (auto joint : *(leg->getJoints()))
               {
                    EXPECT_EQ(joint->getName(), components_names.at(6+j));
                    j++;
               }

               for (auto link : *(leg->getLinks()))
               {
                    EXPECT_EQ(link->getName(), components_names.at(18+k));
                    k++;
               }
          }

          EXPECT_EQ(dummy_robot->getJoint("LF_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("LF_joint_2")->getParent()->getName(), "LF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LF_joint_3")->getParent()->getName(), "LF_link_2");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_2")->getParent()->getName(), "RF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_3")->getParent()->getName(), "RF_link_2");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_2")->getParent()->getName(), "LH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_3")->getParent()->getName(), "LH_link_2");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_2")->getParent()->getName(), "RH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_3")->getParent()->getName(), "RH_link_2");

          EXPECT_EQ(dummy_robot->getJoint("LF_joint_1")->getChild()->getName(), "LF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LF_joint_2")->getChild()->getName(), "LF_link_2");
          EXPECT_EQ(dummy_robot->getJoint("LF_joint_3")->getChild()->getName(), "LF_link_3");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_1")->getChild()->getName(), "RF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_2")->getChild()->getName(), "RF_link_2");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_3")->getChild()->getName(), "RF_link_3");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_1")->getChild()->getName(), "LH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_2")->getChild()->getName(), "LH_link_2");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_3")->getChild()->getName(), "LH_link_3");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_1")->getChild()->getName(), "RH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_2")->getChild()->getName(), "RH_link_2");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_3")->getChild()->getName(), "RH_link_3");

          EXPECT_EQ(dummy_robot->getLink("LF_link_1")->getParent()->getName(), "LF_joint_1");
          EXPECT_EQ(dummy_robot->getLink("LF_link_2")->getParent()->getName(), "LF_joint_2");
          EXPECT_EQ(dummy_robot->getLink("LF_link_3")->getParent()->getName(), "LF_joint_3");
          EXPECT_EQ(dummy_robot->getLink("RF_link_1")->getParent()->getName(), "RF_joint_1");
          EXPECT_EQ(dummy_robot->getLink("RF_link_2")->getParent()->getName(), "RF_joint_2");
          EXPECT_EQ(dummy_robot->getLink("RF_link_3")->getParent()->getName(), "RF_joint_3");
          EXPECT_EQ(dummy_robot->getLink("LH_link_1")->getParent()->getName(), "LH_joint_1");
          EXPECT_EQ(dummy_robot->getLink("LH_link_2")->getParent()->getName(), "LH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("LH_link_3")->getParent()->getName(), "LH_joint_3");
          EXPECT_EQ(dummy_robot->getLink("RH_link_1")->getParent()->getName(), "RH_joint_1");
          EXPECT_EQ(dummy_robot->getLink("RH_link_2")->getParent()->getName(), "RH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("RH_link_3")->getParent()->getName(), "RH_joint_3");

          EXPECT_EQ(dummy_robot->getLink("LF_link_1")->getChild()->getName(), "LF_joint_2");
          EXPECT_EQ(dummy_robot->getLink("LF_link_2")->getChild()->getName(), "LF_joint_3");
          EXPECT_EQ(dummy_robot->getLink("LF_link_3")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("RF_link_1")->getChild()->getName(), "RF_joint_2");
          EXPECT_EQ(dummy_robot->getLink("RF_link_2")->getChild()->getName(), "RF_joint_3");
          EXPECT_EQ(dummy_robot->getLink("RF_link_3")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("LH_link_1")->getChild()->getName(), "LH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("LH_link_2")->getChild()->getName(), "LH_joint_3");
          EXPECT_EQ(dummy_robot->getLink("LH_link_3")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("RH_link_1")->getChild()->getName(), "RH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("RH_link_2")->getChild()->getName(), "RH_joint_3");
          EXPECT_EQ(dummy_robot->getLink("RH_link_3")->getChild(), nullptr);

          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getParent(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getChild(), nullptr);
          i = 0;
          for (auto joint : *(dummy_robot->getLink("TRUNK")->getChildren()))
          {
               EXPECT_EQ(joint->getName(), components_names.at(6+i));
               i=i+3;
          }
     }
     /**
      * @test Dummy robot creation and hierarchy with:
      * 4 legs
      * 0 arms
      * 1 joint per leg
      * 1 link per leg
      * 0 joints per arm
      * 0 links per arm
      */
     {
          robotlib::DummyRobotCreator<4, 4, 4, 1, 1> dummy_robot_creator;

          /* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
          std::array<std::string, 14> components_names{"Dummy Robot",
                                                       "TRUNK",
                                                       "LF", "RF", "LH", "RH",
                                                       "LF_joint_1",
                                                       "RF_joint_1",
                                                       "LH_joint_1",
                                                       "RH_joint_1",
                                                       "LF_link_1",
                                                       "RF_link_1",
                                                       "LH_link_1",
                                                       "RH_link_1"};

          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          EXPECT_EQ(dummy_robot->getNLEGS(), 4);
          EXPECT_EQ(dummy_robot->getNARMS(), 0);
          EXPECT_EQ(dummy_robot->getNJOINTS(), 4);
          EXPECT_EQ(dummy_robot->getNLINKS(), 4);

          EXPECT_EQ(dummy_robot->getName(), "Dummy Robot");

          unsigned int i{0};
          for (auto leg : *(dummy_robot->getLegs()))
          {
               EXPECT_EQ(leg->getName(), components_names.at(2+i));

               for (auto joint : *(leg->getJoints()))
               {
                    EXPECT_EQ(joint->getName(), components_names.at(6+i));
               }

               for (auto link : *(leg->getLinks()))
               {
                    EXPECT_EQ(link->getName(), components_names.at(10+i));
               }
               i++;
          }

          EXPECT_EQ(dummy_robot->getJoint("LF_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_1")->getParent()->getName(), "TRUNK");

          EXPECT_EQ(dummy_robot->getJoint("LF_joint_1")->getChild()->getName(), "LF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RF_joint_1")->getChild()->getName(), "RF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_1")->getChild()->getName(), "LH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_1")->getChild()->getName(), "RH_link_1");

          EXPECT_EQ(dummy_robot->getLink("LF_link_1")->getParent()->getName(), "LF_joint_1");
          EXPECT_EQ(dummy_robot->getLink("RF_link_1")->getParent()->getName(), "RF_joint_1");
          EXPECT_EQ(dummy_robot->getLink("LH_link_1")->getParent()->getName(), "LH_joint_1");
          EXPECT_EQ(dummy_robot->getLink("RH_link_1")->getParent()->getName(), "RH_joint_1");

          EXPECT_EQ(dummy_robot->getLink("LF_link_1")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("RF_link_1")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("LH_link_1")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("RH_link_1")->getChild(), nullptr);

          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getParent(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getChild(), nullptr);
          i = 0;
          for (auto joint : *(dummy_robot->getLink("TRUNK")->getChildren()))
          {
               EXPECT_EQ(joint->getName(), components_names.at(6+i));
               i++;
          }
     }
     /**
      * @test Dummy robot creation and hierarchy with:
      * 1 leg
      * 0 arms
      * 1 joint
      * 1 link
      * 0 joints per arm
      * 0 links per arm
      */
     {
          robotlib::DummyRobotCreator<1, 1, 1, 1, 1> dummy_robot_creator;

          /* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
          std::array<std::string, 5> components_names{"Dummy Robot",
                                                      "TRUNK",
                                                      "CL",
                                                      "CL_joint",
                                                      "CL_link"};

          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          EXPECT_EQ(dummy_robot->getNLEGS(), 1);
          EXPECT_EQ(dummy_robot->getNARMS(), 0);
          EXPECT_EQ(dummy_robot->getNJOINTS(), 1);
          EXPECT_EQ(dummy_robot->getNLINKS(), 1);

          EXPECT_EQ(dummy_robot->getName(), "Dummy Robot");

          for (auto leg : *(dummy_robot->getLegs()))
          {
               EXPECT_EQ(leg->getName(), components_names.at(2));

               for (auto joint : *(leg->getJoints()))
               {
                    EXPECT_EQ(joint->getName(), components_names.at(3));
               }

               for (auto link : *(leg->getLinks()))
               {
                    EXPECT_EQ(link->getName(), components_names.at(4));
               }

          }

          EXPECT_EQ(dummy_robot->getJoint("CL_joint")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("CL_joint")->getChild()->getName(), "CL_link");
          EXPECT_EQ(dummy_robot->getLink("CL_link")->getParent()->getName(), "CL_joint");
          EXPECT_EQ(dummy_robot->getLink("CL_link")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getParent(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getChild()->getName(), "CL_joint");
          for (auto joint : *(dummy_robot->getLink("TRUNK")->getChildren()))
          {
               EXPECT_EQ(joint->getName(), components_names.at(3));
          }
     }
     /**
      * @test Dummy robot creation and hierarchy with:
      * 3 legs
      * 2 arm
      * 3 joints per leg
      * 3 links per leg
      * 2 joints per arm
      * 2 links per arm
      */
     {
          robotlib::DummyRobotCreator<13, 13, 3, 3, 3, 2, 2, 2> dummy_robot_creator;

          /* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
          std::array<std::string, 33> components_names{"Dummy Robot",
                                                       "TRUNK",
                                                       "LH", "CF", "RH",
                                                       "LH_joint_1", "LH_joint_2", "LH_joint_3",
                                                       "CF_joint_1", "CF_joint_2", "CF_joint_3",
                                                       "RH_joint_1", "RH_joint_2", "RH_joint_3",
                                                       "LH_link_1", "LH_link_2", "LH_link_3",
                                                       "CF_link_1", "CF_link_2", "CF_link_3",
                                                       "RH_link_1", "RH_link_2", "RH_link_3",
                                                       "LU", "RU",
                                                       "LU_joint_1", "LU_joint_2",
                                                       "RU_joint_1", "RU_joint_2",
                                                       "LU_link_1", "LU_link_2",
                                                       "RU_link_1", "RU_link_2"};

          auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

          EXPECT_EQ(dummy_robot->getNLEGS(), 3);
          EXPECT_EQ(dummy_robot->getNARMS(), 2);
          EXPECT_EQ(dummy_robot->getNJOINTS(), 13);
          EXPECT_EQ(dummy_robot->getNLINKS(), 13);

          EXPECT_EQ(dummy_robot->getName(), "Dummy Robot");

          unsigned int i{0}, j{0}, k{0};
          for (auto leg : *(dummy_robot->getLegs()))
          {
               EXPECT_EQ(leg->getName(), components_names.at(2+i));
               i++;

               for (auto joint : *(leg->getJoints()))
               {
                    EXPECT_EQ(joint->getName(), components_names.at(5+j));
                    j++;
               }

               for (auto link : *(leg->getLinks()))
               {
                    EXPECT_EQ(link->getName(), components_names.at(14+k));
                    k++;
               }
          }

          i = 0; j = 0; k = 0;
          for (auto arm : *(dummy_robot->getArms()))
          {
               EXPECT_EQ(arm->getName(), components_names.at(23+i));
               i++;

               for (auto joint : *(arm->getJoints()))
               {
                    EXPECT_EQ(joint->getName(), components_names.at(25+j));
                    j++;
               }

               for (auto link : *(arm->getLinks()))
               {
                    EXPECT_EQ(link->getName(), components_names.at(29+k));
                    k++;
               }
          }

          EXPECT_EQ(dummy_robot->getJoint("LH_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_2")->getParent()->getName(), "LH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_3")->getParent()->getName(), "LH_link_2");
          EXPECT_EQ(dummy_robot->getJoint("CF_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("CF_joint_2")->getParent()->getName(), "CF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("CF_joint_3")->getParent()->getName(), "CF_link_2");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_1")->getParent()->getName(), "TRUNK");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_2")->getParent()->getName(), "RH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_3")->getParent()->getName(), "RH_link_2");

          EXPECT_EQ(dummy_robot->getJoint("LH_joint_1")->getChild()->getName(), "LH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_2")->getChild()->getName(), "LH_link_2");
          EXPECT_EQ(dummy_robot->getJoint("LH_joint_3")->getChild()->getName(), "LH_link_3");
          EXPECT_EQ(dummy_robot->getJoint("CF_joint_1")->getChild()->getName(), "CF_link_1");
          EXPECT_EQ(dummy_robot->getJoint("CF_joint_2")->getChild()->getName(), "CF_link_2");
          EXPECT_EQ(dummy_robot->getJoint("CF_joint_3")->getChild()->getName(), "CF_link_3");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_1")->getChild()->getName(), "RH_link_1");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_2")->getChild()->getName(), "RH_link_2");
          EXPECT_EQ(dummy_robot->getJoint("RH_joint_3")->getChild()->getName(), "RH_link_3");

          EXPECT_EQ(dummy_robot->getLink("LH_link_1")->getParent()->getName(), "LH_joint_1");
          EXPECT_EQ(dummy_robot->getLink("LH_link_2")->getParent()->getName(), "LH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("LH_link_3")->getParent()->getName(), "LH_joint_3");
          EXPECT_EQ(dummy_robot->getLink("CF_link_1")->getParent()->getName(), "CF_joint_1");
          EXPECT_EQ(dummy_robot->getLink("CF_link_2")->getParent()->getName(), "CF_joint_2");
          EXPECT_EQ(dummy_robot->getLink("CF_link_3")->getParent()->getName(), "CF_joint_3");
          EXPECT_EQ(dummy_robot->getLink("RH_link_1")->getParent()->getName(), "RH_joint_1");
          EXPECT_EQ(dummy_robot->getLink("RH_link_2")->getParent()->getName(), "RH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("RH_link_3")->getParent()->getName(), "RH_joint_3");

          EXPECT_EQ(dummy_robot->getLink("LH_link_1")->getChild()->getName(), "LH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("LH_link_2")->getChild()->getName(), "LH_joint_3");
          EXPECT_EQ(dummy_robot->getLink("LH_link_3")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("CF_link_1")->getChild()->getName(), "CF_joint_2");
          EXPECT_EQ(dummy_robot->getLink("CF_link_2")->getChild()->getName(), "CF_joint_3");
          EXPECT_EQ(dummy_robot->getLink("CF_link_3")->getChild(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("RH_link_1")->getChild()->getName(), "RH_joint_2");
          EXPECT_EQ(dummy_robot->getLink("RH_link_2")->getChild()->getName(), "RH_joint_3");
          EXPECT_EQ(dummy_robot->getLink("RH_link_3")->getChild(), nullptr);

          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getParent(), nullptr);
          EXPECT_EQ(dummy_robot->getLink("TRUNK")->getChild(), nullptr);
          i = 0; j = 0;
          for (auto joint : *(dummy_robot->getLink("TRUNK")->getChildren()))
          {
               if(i <= 8)
               {
                    EXPECT_EQ(joint->getName(), components_names.at(5+i));
                    i=i+3;
               }
               else
               {
                    EXPECT_EQ(joint->getName(), components_names.at(25+j));
                    j=j+2;
               }
          }
     }
}