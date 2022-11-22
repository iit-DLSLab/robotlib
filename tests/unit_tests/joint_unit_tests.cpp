/**
 * @file joint_unit_tests.cpp
 * @brief Unit tests for Joint class
 *
 * @authors Authors in alphabetic order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "robot_factory.hpp"
#include "joint.hpp"

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
          ASSERT_EQ(joint.getName(), "joint_test");
     }

     /**
      * @test Joint name with two separate words
      */
     {
          robotlib::Joint joint_two_words_name{"joint test"};
          ASSERT_EQ(joint_two_words_name.getName(), "joint test");
     }

     /**
      * @test Joint name with an empty string
      */
     {
          robotlib::Joint joint_empty_name{""};
          ASSERT_EQ(joint_empty_name.getName(), "");
     }

     /**
      * @test Joint name with a single space character
      */
     {
          robotlib::Joint joint_single_space_name{" "};
          ASSERT_EQ(joint_single_space_name.getName(), " ");
     }

     /**
      * @test Dummy Quadruped - Limbs (legs) joints names
      */
     {
          std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
          std::array<std::array<std::string, 3>, 4> joints_names{{{"LF_HAA", "LF_HFE", "LF_KFE"},
                                                                  {"RF_HAA", "RF_HFE", "RF_KFE"},
                                                                  {"LH_HAA", "LH_HFE", "LH_KFE"},
                                                                  {"RH_HAA", "RH_HFE", "RH_KFE"}}};
          unsigned int i {0}, j {0};
          for (auto leg : *(dummy_quadruped->getLegs()))
          {
               for (auto leg_joint : *(leg->getJoints()))
               {
                    ASSERT_EQ(leg_joint->getName(), joints_names.at(i).at(j));
                    j++;
               }
               i++;
               j = 0;
          }
     }
}

/**
 * @brief Set of unit tests for Joint::getParent function
 */
TEST(JointUnitTests, getParent)
{
     /**
      * @test Dummy Quadruped - Joint parent compared according to its names only
      * /// TODO: Overload operator= to compare Link objects directly
      */
     std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

     const auto joint_lf_haa = dummy_quadruped->getJoint("LF_HAA");
     const std::shared_ptr<robotlib::Link> joint_lf_haa_parent = std::make_shared<robotlib::Link>("TRUNK");
     const auto joint_lf_hfe = dummy_quadruped->getJoint("LF_HFE");
     const std::shared_ptr<robotlib::Link> joint_lf_hfe_parent = std::make_shared<robotlib::Link>("LF_ASSEMBLY");
     const auto joint_lf_kfe = dummy_quadruped->getJoint("LF_KFE");
     const std::shared_ptr<robotlib::Link> joint_lf_kfe_parent = std::make_shared<robotlib::Link>("LF_UPPERLEG");

     const auto joint_rf_haa = dummy_quadruped->getJoint("RF_HAA");
     const std::shared_ptr<robotlib::Link> joint_rf_haa_parent = std::make_shared<robotlib::Link>("TRUNK");
     const auto joint_rf_hfe = dummy_quadruped->getJoint("RF_HFE");
     const std::shared_ptr<robotlib::Link> joint_rf_hfe_parent = std::make_shared<robotlib::Link>("RF_ASSEMBLY");
     const auto joint_rf_kfe = dummy_quadruped->getJoint("RF_KFE");
     const std::shared_ptr<robotlib::Link> joint_rf_kfe_parent = std::make_shared<robotlib::Link>("RF_UPPERLEG");

     const auto joint_lh_haa = dummy_quadruped->getJoint("LH_HAA");
     const std::shared_ptr<robotlib::Link> joint_lh_haa_parent = std::make_shared<robotlib::Link>("TRUNK");
     const auto joint_lh_hfe = dummy_quadruped->getJoint("LH_HFE");
     const std::shared_ptr<robotlib::Link> joint_lh_hfe_parent = std::make_shared<robotlib::Link>("LH_ASSEMBLY");
     const auto joint_lh_kfe = dummy_quadruped->getJoint("LH_KFE");
     const std::shared_ptr<robotlib::Link> joint_lh_kfe_parent = std::make_shared<robotlib::Link>("LH_UPPERLEG");

     const auto joint_rh_haa = dummy_quadruped->getJoint("RH_HAA");
     const std::shared_ptr<robotlib::Link> joint_rh_haa_parent = std::make_shared<robotlib::Link>("TRUNK");
     const auto joint_rh_hfe = dummy_quadruped->getJoint("RH_HFE");
     const std::shared_ptr<robotlib::Link> joint_rh_hfe_parent = std::make_shared<robotlib::Link>("RH_ASSEMBLY");
     const auto joint_rh_kfe = dummy_quadruped->getJoint("RH_KFE");
     const std::shared_ptr<robotlib::Link> joint_rh_kfe_parent = std::make_shared<robotlib::Link>("RH_UPPERLEG");

     ASSERT_EQ(joint_lf_haa->getParent()->getName(), joint_lf_haa_parent->getName());
     ASSERT_EQ(joint_lf_hfe->getParent()->getName(), joint_lf_hfe_parent->getName());
     ASSERT_EQ(joint_lf_kfe->getParent()->getName(), joint_lf_kfe_parent->getName());

     ASSERT_EQ(joint_rf_haa->getParent()->getName(), joint_rf_haa_parent->getName());
     ASSERT_EQ(joint_rf_hfe->getParent()->getName(), joint_rf_hfe_parent->getName());
     ASSERT_EQ(joint_rh_kfe->getParent()->getName(), joint_rh_kfe_parent->getName());

     ASSERT_EQ(joint_lh_haa->getParent()->getName(), joint_lh_haa_parent->getName());
     ASSERT_EQ(joint_lh_hfe->getParent()->getName(), joint_lh_hfe_parent->getName());
     ASSERT_EQ(joint_lf_kfe->getParent()->getName(), joint_lf_kfe_parent->getName());

     ASSERT_EQ(joint_rh_haa->getParent()->getName(), joint_rh_haa_parent->getName());
     ASSERT_EQ(joint_rh_hfe->getParent()->getName(), joint_rh_hfe_parent->getName());
     ASSERT_EQ(joint_rh_kfe->getParent()->getName(), joint_rh_kfe_parent->getName());
}

/**
 * @brief Set of unit tests for Joint::getChild function
 */
TEST(JointUnitTests, getChild)
{
     /**
      * @test Dummy Quadruped - Joint child compared according to its names only
      * /// TODO: Overload operator= to compare Link objects directly
      */
     std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

     const auto joint_lf_haa = dummy_quadruped->getJoint("LF_HAA");
     const std::shared_ptr<robotlib::Link> joint_lf_haa_child = std::make_shared<robotlib::Link>("LF_ASSEMBLY");
     const auto joint_lf_hfe = dummy_quadruped->getJoint("LF_HFE");
     const std::shared_ptr<robotlib::Link> joint_lf_hfe_child = std::make_shared<robotlib::Link>("LF_UPPERLEG");
     const auto joint_lf_kfe = dummy_quadruped->getJoint("LF_KFE");
     const std::shared_ptr<robotlib::Link> joint_lf_kfe_child = std::make_shared<robotlib::Link>("LF_LOWERLEG");

     const auto joint_rf_haa = dummy_quadruped->getJoint("RF_HAA");
     const std::shared_ptr<robotlib::Link> joint_rf_haa_child = std::make_shared<robotlib::Link>("RF_ASSEMBLY");
     const auto joint_rf_hfe = dummy_quadruped->getJoint("RF_HFE");
     const std::shared_ptr<robotlib::Link> joint_rf_hfe_child = std::make_shared<robotlib::Link>("RF_UPPERLEG");
     const auto joint_rf_kfe = dummy_quadruped->getJoint("RF_KFE");
     const std::shared_ptr<robotlib::Link> joint_rf_kfe_child = std::make_shared<robotlib::Link>("RF_LOWERLEG");

     const auto joint_lh_haa = dummy_quadruped->getJoint("LH_HAA");
     const std::shared_ptr<robotlib::Link> joint_lh_haa_child = std::make_shared<robotlib::Link>("LH_ASSEMBLY");
     const auto joint_lh_hfe = dummy_quadruped->getJoint("LH_HFE");
     const std::shared_ptr<robotlib::Link> joint_lh_hfe_child = std::make_shared<robotlib::Link>("LH_UPPERLEG");
     const auto joint_lh_kfe = dummy_quadruped->getJoint("LH_KFE");
     const std::shared_ptr<robotlib::Link> joint_lh_kfe_child = std::make_shared<robotlib::Link>("LH_LOWERLEG");

     const auto joint_rh_haa = dummy_quadruped->getJoint("RH_HAA");
     const std::shared_ptr<robotlib::Link> joint_rh_haa_child = std::make_shared<robotlib::Link>("RH_ASSEMBLY");
     const auto joint_rh_hfe = dummy_quadruped->getJoint("RH_HFE");
     const std::shared_ptr<robotlib::Link> joint_rh_hfe_child = std::make_shared<robotlib::Link>("RH_UPPERLEG");
     const auto joint_rh_kfe = dummy_quadruped->getJoint("RH_KFE");
     const std::shared_ptr<robotlib::Link> joint_rh_kfe_child = std::make_shared<robotlib::Link>("RH_LOWERLEG");

     ASSERT_EQ(joint_lf_haa->getChild()->getName(), joint_lf_haa_child->getName());
     ASSERT_EQ(joint_lf_hfe->getChild()->getName(), joint_lf_hfe_child->getName());
     ASSERT_EQ(joint_lf_kfe->getChild()->getName(), joint_lf_kfe_child->getName());

     ASSERT_EQ(joint_rf_haa->getChild()->getName(), joint_rf_haa_child->getName());
     ASSERT_EQ(joint_rf_hfe->getChild()->getName(), joint_rf_hfe_child->getName());
     ASSERT_EQ(joint_rh_kfe->getChild()->getName(), joint_rh_kfe_child->getName());

     ASSERT_EQ(joint_lh_haa->getChild()->getName(), joint_lh_haa_child->getName());
     ASSERT_EQ(joint_lh_hfe->getChild()->getName(), joint_lh_hfe_child->getName());
     ASSERT_EQ(joint_lf_kfe->getChild()->getName(), joint_lf_kfe_child->getName());

     ASSERT_EQ(joint_rh_haa->getChild()->getName(), joint_rh_haa_child->getName());
     ASSERT_EQ(joint_rh_hfe->getChild()->getName(), joint_rh_hfe_child->getName());
     ASSERT_EQ(joint_rh_kfe->getChild()->getName(), joint_rh_kfe_child->getName());
}

/**
 * @brief Set of unit tests for Joint::setParent function
 */
TEST(JointUnitTests, setParent)
{
     /**
      * @test Dummy Quadruped - Set of joint parent
      * /// TODO: implement the unit tests. Set method is private
      */
}

/**
 * @brief Set of unit tests for Joint::setChild function
 */
TEST(JointUnitTests, setChild)
{
     /**
      * @test Dummy Quadruped - Set of joint child
      * /// TODO: implement the unit tests. Set method is private
      */
}

/**
 * @brief Set of unit tests for 
 * Joint::setMinAngle and Joint::getMinAngle functions
 * Joint::setMaxAngle and Joint::getMaxAngle functions
 * Joint::setMaxVelocity and Joint::getMaxVelocity functions
 * Joint::setMaxEffort and Joint::getMaxEffort functions
 */
TEST(JointUnitTests, setLimits_getLimits)
{
     /**
      * @test Joint limits compared with a ground truth
      */
     std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

     const double q_min_gt {0};
     const double q_max_gt {90};
     const double qd_max_gt {3};
     const double tau_max_gt {5};

     for (auto leg : *(dummy_quadruped->getLegs()))
     {
          for (auto leg_joint : *(leg->getJoints()))
          {
               EXPECT_EQ(q_min_gt, leg_joint->getMinAngle());
               EXPECT_EQ(q_max_gt, leg_joint->getMaxAngle());
               EXPECT_EQ(qd_max_gt, leg_joint->getMaxVelocity());
               EXPECT_EQ(tau_max_gt, leg_joint->getMaxEffort());
          }
     }
}