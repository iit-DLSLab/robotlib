/**
 * @file trunk_unit_tests.cpp
 * @brief Unit tests for Trunk class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "trunk.hpp"

/**
 * @brief Set of unit tests for Trunk::getName function
 */
TEST(TrunkUnitTests, getName)
{
     /**
      * @test Trunk name with a complete string
      */
     dls::robotlib::Trunk trunk{"trunk_test"};
     ASSERT_EQ(trunk.getName(), "trunk_test");

     /**
      * @test Trunk name with two separate words
      */
     dls::robotlib::Trunk trunk_two_words_name{"trunk test"};
     ASSERT_EQ(trunk_two_words_name.getName(), "trunk test");

     /**
      * @test Trunk name with an empty string
      */
     dls::robotlib::Trunk trunk_empty_name{""};
     ASSERT_EQ(trunk_empty_name.getName(), "");

     /**
      * @test Trunk name with a single space character
      */
     dls::robotlib::Trunk trunk_single_space_name{" "};
     ASSERT_EQ(trunk_single_space_name.getName(), " ");
}

int main(int argc, char **argv)
{
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}