/**
 * @file link_unit_tests.cpp
 * @brief Unit tests for Link class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "link.hpp"

/**
 * @brief Set of unit tests for Link::getName function
 */
TEST(LinkUnitTests, getName)
{
     /**
      * @test Link name with a complete string
      */
     dls::robotlib::Link link{"link_test"};
     ASSERT_EQ(link.getName(), "link_test");

     /**
      * @test Link name with two separate words
      */
     dls::robotlib::Link link_two_words_name{"link test"};
     ASSERT_EQ(link_two_words_name.getName(), "link test");

     /**
      * @test Link name with an empty string
      */
     dls::robotlib::Link link_empty_name{""};
     ASSERT_EQ(link_empty_name.getName(), "");

     /**
      * @test Link name with a single space character
      */
     dls::robotlib::Link link_single_space_name{" "};
     ASSERT_EQ(link_single_space_name.getName(), " ");
}

/**
 * @brief Set of unit tests for Link::getParent function
 */
TEST(LinkUnitTests, getParent)
{
}

/**
 * @brief Set of unit tests for Link::getChild function
 */
TEST(LinkUnitTests, getChild)
{
}

/**
 * @brief Set of unit tests for Link::setChild function
 */
TEST(LinkUnitTests, setChild)
{
}

/**
 * @brief Set of unit tests for Link::setParent function
 */
TEST(LinkUnitTests, setParent)
{
}

/**
 * @brief Set of unit tests for Link::setChildren function
 */
TEST(LinkUnitTests, setChildren)
{
}