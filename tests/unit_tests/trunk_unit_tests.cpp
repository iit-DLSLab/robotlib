/**
 * @file trunk_unit_tests.cpp
 * 
 * @brief Unit tests for Trunk class
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
 * 1 joints per leg
 * 1 links per leg
 */
robotlib::DummyRobotCreator<1, 1, 1> dummy_robot_creator;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 5> components_names{"Dummy Robot",
                                            "TRUNK",
                                            "Leg",
                                            "Leg_joint_1",
                                            "Leg_link_1",};

/**
 * @test Dummy robot created with the following structure:
 * 2 legs
 * 1 joints per leg
 * 1 links per leg
 */
robotlib::DummyRobotCreator<2, 1, 1> dummy_robot_creator_2;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 8> components_names_2{"Dummy Robot",
                                              "TRUNK",
                                              "Leg",
                                              "Leg2",
                                              "Leg_joint_1",
                                              "Leg2_joint_2",
                                              "Leg_link_1",
                                              "Leg2_link_2"};

/**
 * @brief Set of unit tests for Trunk::getName function
 */
TEST(TrunkUnitTests, getName)
{  
    /**
     * @test Trunk name with a complete string
     */
    {
        robotlib::Trunk trunk{"trunk_test"};
        EXPECT_EQ(trunk.getName(), "trunk_test");
    }

    /**
     * @test Trunk name with two separate words
     */
    {
        robotlib::Trunk trunk{"trunk test"};
        EXPECT_EQ(trunk.getName(), "trunk test");
    }

    /**
     * @test Trunk name with an empty string
     */
    {
        robotlib::Trunk trunk{""};
        EXPECT_EQ(trunk.getName(), "");
    }

    /**
     * @test Trunk name with a single space character
     */
    {
        robotlib::Trunk trunk{" "};
        EXPECT_EQ(trunk.getName(), " ");
    }

    /**
     * @test Dummy robot trunk name
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

        // auto trunk_link{dummy_robot->getLink(components_names[1])};

        // EXPECT_EQ(trunk_link.getName(), components_names[1]);
    }
}

// /**
//  * @brief Set of unit tests for Trunk::getParent function (inherited from Link)
//  */
// TEST(LinkUnitTests, getParent)
// {
//     /**
//      * @test Get the trunk parent (trunk has generally no parent links)
//      */
//     {
//         auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//         auto trunk_link{dummy_robot->getLink(components_names.at(1))};

//         EXPECT_EQ(trunk_link.getParent(), nullptr);
//     }
// }

// /**
//  * @brief Set of unit tests for Trunk::getChild function (inherited from Link)
//  */
// TEST(LinkUnitTests, getChild)
// {
//     /**
//      * @test Get the trunk child (case with one child)
//      */
//     {
//         auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//         auto trunk_link{dummy_robot->getLink(components_names.at(1))};

//         EXPECT_EQ(trunk_link.getChildren()[0]->getName(), components_names.at(3));
//     }

//     /**
//      * @test Get the trunk child (case with two children. Child is not univoque and is set as null pointer)
//      */
//     {
//         auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

//         auto trunk_link{dummy_robot->getLink(components_names_2.at(1))};

//         EXPECT_EQ(trunk_link.getChildren().size(), 2);
//     }
// }

// /**
//  * @brief Set of unit tests for Trunk::getChildren function (inherited from Link)
//  */
// TEST(LinkUnitTests, getChildren)
// {
//     /**
//      * @test Iterate over trunk children (case with one child)
//      */
//     {
//         auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//         auto trunk_link{dummy_robot->getLink(components_names.at(1))};

//         EXPECT_EQ((trunk_link.getChildren().size()), 1);

//         for (auto& trunk_child : trunk_link.getChildren())
//         {
//             EXPECT_EQ(trunk_child->getName(), components_names.at(3));
//         }
//     }

//     /**
//      * @test Iterate over the trunk children (case with two children)
//      */
//     {
//         auto dummy_robot = dummy_robot_creator_2.createDummyRobot(components_names_2);

//         auto trunk_link{dummy_robot->getLink(components_names_2.at(1))};

//         EXPECT_EQ(trunk_link.getChildren().size(), 2);

//         unsigned int i{0};
//         for (auto& trunk_child : trunk_link.getChildren())
//         {
//             EXPECT_EQ(trunk_child->getName(), components_names_2.at(4+i));
//             i++;
//         }
//     }
// }
// /// TODO: The following tests on getCoM, getMass, getInertia and getDynParams should be substitued using the set functions
// /// implemented in Robotlib (and so, using a dummy robot) instead of in the Glue.

// /**
//  * @brief Set of unit tests for Trunk::getCoM function
//  */
// TEST(TrunkUnitTests, getCoM)
// {
//      const Eigen::Vector3d com{0.5, 0.3, 0.1};
//      const double mass{5};
//      Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
//      inertia.diagonal()[0] = 0.5;
//      inertia.diagonal()[1] = 0.5;
//      inertia.diagonal()[2] = 0.5;

//      const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
//      /**
//       * @test Trunk com with dummy values
//       */
//      {
// 	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
//           EXPECT_EQ(trunk.getCoM(), com);
//      }
// }


// /**
//  * @brief Set of unit tests for Trunk::getMass function
//  */
// TEST(TrunkUnitTests, getMass)
// {
//      const Eigen::Vector3d com{0.5, 0.3, 0.1};
//      const double mass{5};
//      Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
//      inertia.diagonal()[0] = 0.5;
//      inertia.diagonal()[1] = 0.5;
//      inertia.diagonal()[2] = 0.5;

//      const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
//      /**
//       * @test Trunk mass with dummy values
//       */
//      {
// 	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
//           EXPECT_EQ(trunk.getMass(), mass);
//      }
// }


// /**
//  * @brief Set of unit tests for Trunk::getInertia function
//  */
// TEST(TrunkUnitTests, getInertia)
// {
//      const Eigen::Vector3d com{0.5, 0.3, 0.1};
//      const double mass{5};
//      Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
//      inertia.diagonal()[0] = 0.5;
//      inertia.diagonal()[1] = 0.5;
//      inertia.diagonal()[2] = 0.5;

//      const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
//      /**
//       * @test Trunk inertia with dummy values
//       */
//      {
// 	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
//           EXPECT_EQ(trunk.getInertia(), inertia);
//      }
// }

// /**
//  * @brief Set of unit tests for Trunk::getDynParams function
//  */
// TEST(TrunkUnitTests, getDynParams)
// {
//      const Eigen::Vector3d com{0.5, 0.3, 0.1};
//      const double mass{5};
//      Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
//      inertia.diagonal()[0] = 0.5;
//      inertia.diagonal()[1] = 0.5;
//      inertia.diagonal()[2] = 0.5;

//      const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
//      /**
//       * @test Trunk inertia with dummy values
//       */
//      {
// 	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
//           EXPECT_EQ(trunk.getDynParams().getCoM(), com);
//           EXPECT_EQ(trunk.getDynParams().getMass(), mass);
//           EXPECT_EQ(trunk.getDynParams().getInertia(), inertia);
//      }
// }