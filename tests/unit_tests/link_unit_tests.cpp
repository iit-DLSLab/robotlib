/**
 * @file link_unit_tests.cpp
 * @brief Unit tests for Link class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "link.hpp"
#include "../src/robots/dummy_quadruped.cpp" /// TODO: Remove cpp inclusion

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
          ASSERT_EQ(link.getName(), "link_test");
     }

     /**
      * @test Link name with two separate words
      */
     {
          robotlib::Link link_two_words_name{"link test"};
          ASSERT_EQ(link_two_words_name.getName(), "link test");
     }

     /**
      * @test Link name with an empty string
      */
     {
          robotlib::Link link_empty_name{""};
          ASSERT_EQ(link_empty_name.getName(), "");
     }

     /**
      * @test Link name with a single space character
      */
     {
          robotlib::Link link_single_space_name{" "};
          ASSERT_EQ(link_single_space_name.getName(), " ");
     }

     /**
      * @test Dummy Quadruped - Limbs (legs) links names
      */
     {
          std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
          std::array<std::array<std::string, 3>, 4> links_names{{{"LF_ASSEMBLY", "LF_UPPERLEG", "LF_LOWERLEG"},
                                                                 {"RF_ASSEMBLY", "RF_UPPERLEG", "RF_LOWERLEG"},
                                                                 {"LH_ASSEMBLY", "LH_UPPERLEG", "LH_LOWERLEG"},
                                                                 {"RH_ASSEMBLY", "RH_UPPERLEG", "RH_LOWERLEG"}}};
          unsigned int i{0}, j{0};
          for (auto leg : *(dummy_quadruped->getLegs()))
          {
               for (auto leg_link : *(leg->getLinks()))
               {
                    ASSERT_EQ(leg_link->getName(), links_names.at(i).at(j));
                    j++;
               }
               i++;
               j = 0;
          }
     }
}

/**
 * @brief Set of unit tests for Link::getParent function
 */
TEST(LinkUnitTests, getParent)
{
     /**
      * @test Dummy Quadruped - Joint parent compared according to its name only
      * /// TODO: Overload operator= to compare Link objects directly
      */
     std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();

     const auto link_lf_assembly = dummy_quadruped->getLink("LF_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_lf_assembly_parent = std::make_shared<robotlib::Joint>("LF_HAA");
     const auto link_lf_upperleg = dummy_quadruped->getLink("LF_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_lf_upperleg_parent = std::make_shared<robotlib::Joint>("LF_HFE");
     const auto link_lf_lowerleg = dummy_quadruped->getLink("LF_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_lf_lowerleg_parent = std::make_shared<robotlib::Joint>("LF_KFE");

     const auto link_rf_assembly = dummy_quadruped->getLink("RF_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_rf_assembly_parent = std::make_shared<robotlib::Joint>("RF_HAA");
     const auto link_rf_upperleg = dummy_quadruped->getLink("RF_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_rf_upperleg_parent = std::make_shared<robotlib::Joint>("RF_HFE");
     const auto link_rf_lowerleg = dummy_quadruped->getLink("RF_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_rf_lowerleg_parent = std::make_shared<robotlib::Joint>("RF_KFE");

     const auto link_lh_assembly = dummy_quadruped->getLink("LH_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_lh_assembly_parent = std::make_shared<robotlib::Joint>("LH_HAA");
     const auto link_lh_upperleg = dummy_quadruped->getLink("LH_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_lh_upperleg_parent = std::make_shared<robotlib::Joint>("LH_HFE");
     const auto link_lh_lowerleg = dummy_quadruped->getLink("LH_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_lh_lowerleg_parent = std::make_shared<robotlib::Joint>("LH_KFE");

     const auto link_rh_assembly = dummy_quadruped->getLink("RH_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_rh_assembly_parent = std::make_shared<robotlib::Joint>("RH_HAA");
     const auto link_rh_upperleg = dummy_quadruped->getLink("RH_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_rh_upperleg_parent = std::make_shared<robotlib::Joint>("RH_HFE");
     const auto link_rh_lowerleg = dummy_quadruped->getLink("RH_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_rh_lowerleg_parent = std::make_shared<robotlib::Joint>("RH_KFE");

     ASSERT_EQ(link_lf_assembly->getParent()->getName(), link_lf_assembly_parent->getName());
     ASSERT_EQ(link_lf_upperleg->getParent()->getName(), link_lf_upperleg_parent->getName());
     ASSERT_EQ(link_lf_lowerleg->getParent()->getName(), link_lf_lowerleg_parent->getName());

     ASSERT_EQ(link_rf_assembly->getParent()->getName(), link_rf_assembly_parent->getName());
     ASSERT_EQ(link_rf_upperleg->getParent()->getName(), link_rf_upperleg_parent->getName());
     ASSERT_EQ(link_rh_lowerleg->getParent()->getName(), link_rh_lowerleg_parent->getName());

     ASSERT_EQ(link_lh_assembly->getParent()->getName(), link_lh_assembly_parent->getName());
     ASSERT_EQ(link_lh_upperleg->getParent()->getName(), link_lh_upperleg_parent->getName());
     ASSERT_EQ(link_lf_lowerleg->getParent()->getName(), link_lf_lowerleg_parent->getName());

     ASSERT_EQ(link_rh_assembly->getParent()->getName(), link_rh_assembly_parent->getName());
     ASSERT_EQ(link_rh_upperleg->getParent()->getName(), link_rh_upperleg_parent->getName());
     ASSERT_EQ(link_rh_lowerleg->getParent()->getName(), link_rh_lowerleg_parent->getName());
}

/**
 * @brief Set of unit tests for Link::getChild function
 */
TEST(LinkUnitTests, getChild)
{
     /**
      * @test Dummy Quadruped - Joint child compared according to its names only
      * /// TODO: Overload operator= to compare Link objects directly
      */
     std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();

     const auto link_lf_assembly = dummy_quadruped->getLink("LF_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_lf_assembly_child = std::make_shared<robotlib::Joint>("LF_HFE");
     const auto link_lf_upperleg = dummy_quadruped->getLink("LF_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_lf_upperleg_child = std::make_shared<robotlib::Joint>("LF_KFE");
     const auto link_lf_lowerleg = dummy_quadruped->getLink("LF_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_lf_lowerleg_child = std::make_shared<robotlib::Joint>("");

     const auto link_rf_assembly = dummy_quadruped->getLink("RF_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_rf_assembly_child = std::make_shared<robotlib::Joint>("RF_HFE");
     const auto link_rf_upperleg = dummy_quadruped->getLink("RF_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_rf_upperleg_child = std::make_shared<robotlib::Joint>("RF_KFE");
     const auto link_rf_lowerleg = dummy_quadruped->getLink("RF_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_rf_lowerleg_child = std::make_shared<robotlib::Joint>("");

     const auto link_lh_assembly = dummy_quadruped->getLink("LH_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_lh_assembly_child = std::make_shared<robotlib::Joint>("LH_HFE");
     const auto link_lh_upperleg = dummy_quadruped->getLink("LH_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_lh_upperleg_child = std::make_shared<robotlib::Joint>("LH_KFE");
     const auto link_lh_lowerleg = dummy_quadruped->getLink("LH_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_lh_lowerleg_child = std::make_shared<robotlib::Joint>("");

     const auto link_rh_assembly = dummy_quadruped->getLink("RH_ASSEMBLY");
     const std::shared_ptr<robotlib::Joint> link_rh_assembly_child = std::make_shared<robotlib::Joint>("RH_HFE");
     const auto link_rh_upperleg = dummy_quadruped->getLink("RH_UPPERLEG");
     const std::shared_ptr<robotlib::Joint> link_rh_upperleg_child = std::make_shared<robotlib::Joint>("RH_KFE");
     const auto link_rh_lowerleg = dummy_quadruped->getLink("RH_LOWERLEG");
     const std::shared_ptr<robotlib::Joint> link_rh_lowerleg_child = std::make_shared<robotlib::Joint>("");

     ASSERT_EQ(link_lf_assembly->getChild()->getName(), link_lf_assembly_child->getName());
     ASSERT_EQ(link_lf_upperleg->getChild()->getName(), link_lf_upperleg_child->getName());
     /// TODO: if getName() is called on nullptr (no parent/child as shared_ptr<Joint/Link>) you get segmentation fault.
     /// We should avoid this and return an empty string as name
     ASSERT_EQ(link_lf_lowerleg->getChild(), nullptr);

     ASSERT_EQ(link_rf_assembly->getChild()->getName(), link_rf_assembly_child->getName());
     ASSERT_EQ(link_rf_upperleg->getChild()->getName(), link_rf_upperleg_child->getName());
     ASSERT_EQ(link_rh_lowerleg->getChild(), nullptr);

     ASSERT_EQ(link_lh_assembly->getChild()->getName(), link_lh_assembly_child->getName());
     ASSERT_EQ(link_lh_upperleg->getChild()->getName(), link_lh_upperleg_child->getName());
     ASSERT_EQ(link_lf_lowerleg->getChild(), nullptr);

     ASSERT_EQ(link_rh_assembly->getChild()->getName(), link_rh_assembly_child->getName());
     ASSERT_EQ(link_rh_upperleg->getChild()->getName(), link_rh_upperleg_child->getName());
     ASSERT_EQ(link_rh_lowerleg->getChild(), nullptr);
}

/**
 * @brief Set of unit tests for Link::setParent function
 */
TEST(LinkUnitTests, setParent)
{
     /**
      * @test Dummy Quadruped - Set of link parent
      * /// TODO: implement the unit tests. Set method is private
      */
}

/**
 * @brief Set of unit tests for Link::setChild function
 */
TEST(LinkUnitTests, setChild)
{
     /**
      * @test Dummy Quadruped - Set of link child
      * /// TODO: implement the unit tests. Set method is private
      */
}

/**
 * @brief Set of unit tests for Link::setChildren function
 */
TEST(LinkUnitTests, setChildren)
{
     /**
      * @test Dummy Quadruped - Set of link children
      * /// TODO: implement the unit tests. Set method is private
      */
}

int main(int argc, char **argv)
{
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}