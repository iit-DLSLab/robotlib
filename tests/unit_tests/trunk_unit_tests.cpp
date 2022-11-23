/**
 * @file trunk_unit_tests.cpp
 * @brief Unit tests for Trunk class
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
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
     {

	     robotlib::Trunk trunk{"trunk_test"};
          EXPECT_EQ(trunk.getName(), "trunk_test");
     }

     /**
      * @test Trunk name with two separate words
      */
     {
          robotlib::Trunk trunk_two_words_name{"trunk test"};
          EXPECT_EQ(trunk_two_words_name.getName(), "trunk test");
     }

     /**
      * @test Trunk name with an empty string
      */
     {
          robotlib::Trunk trunk_empty_name{""};
          EXPECT_EQ(trunk_empty_name.getName(), "");
     }

     /**
      * @test Trunk name with a single space character
      */
     {
          robotlib::Trunk trunk_single_space_name{" "};
          EXPECT_EQ(trunk_single_space_name.getName(), " ");
     }
}


/**
 * @brief Set of unit tests for Trunk::getCoM function
 */
TEST(TrunkUnitTests, getCoM)
{
     const Eigen::Vector3d com{0.5, 0.3, 0.1};
     const double mass{5};
     Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
     inertia.diagonal()[0] = 0.5;
     inertia.diagonal()[1] = 0.5;
     inertia.diagonal()[2] = 0.5;

     const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
     /**
      * @test Trunk com with dummy values
      */
     {
	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
          EXPECT_EQ(trunk.getCoM(), com);
     }

     /**
      * @test TODO: limit case values
      */
}


/**
 * @brief Set of unit tests for Trunk::getMass function
 */
TEST(TrunkUnitTests, getMass)
{
     const Eigen::Vector3d com{0.5, 0.3, 0.1};
     const double mass{5};
     Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
     inertia.diagonal()[0] = 0.5;
     inertia.diagonal()[1] = 0.5;
     inertia.diagonal()[2] = 0.5;

     const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
     /**
      * @test Trunk mass with dummy values
      */
     {
	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
          EXPECT_EQ(trunk.getMass(), mass);
     }

     /**
      * @test TODO: limit case values
      */
}


/**
 * @brief Set of unit tests for Trunk::getInertia function
 */
TEST(TrunkUnitTests, getInertia)
{
     const Eigen::Vector3d com{0.5, 0.3, 0.1};
     const double mass{5};
     Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
     inertia.diagonal()[0] = 0.5;
     inertia.diagonal()[1] = 0.5;
     inertia.diagonal()[2] = 0.5;

     const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
     /**
      * @test Trunk inertia with dummy values
      */
     {
	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
          EXPECT_EQ(trunk.getInertia(), inertia);
     }

     /**
      * @test TODO: limit case values
      */
}

/**
 * @brief Set of unit tests for Trunk::getDynParams function
 */
TEST(TrunkUnitTests, getDynParams)
{
     const Eigen::Vector3d com{0.5, 0.3, 0.1};
     const double mass{5};
     Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
     inertia.diagonal()[0] = 0.5;
     inertia.diagonal()[1] = 0.5;
     inertia.diagonal()[2] = 0.5;

     const robotlib::DynParams trunk_dyn_params{com, mass, inertia}; // dummy com, mass, inertia
          
     /**
      * @test Trunk inertia with dummy values
      */
     {
	     robotlib::Trunk trunk{"trunk", trunk_dyn_params};
          EXPECT_EQ(trunk.getDynParams().getCoM(), com);
          EXPECT_EQ(trunk.getDynParams().getMass(), mass);
          EXPECT_EQ(trunk.getDynParams().getInertia(), inertia);
     }

     /**
      * @test TODO: limit case values
      */
}