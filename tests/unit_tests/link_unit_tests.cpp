/**
 * @file link_unit_tests.cpp
 * 
 * @brief Unit tests for Link class
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
 * @test Dummy robot
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
 * @brief Set of unit tests for Link::getName function
 */
TEST(LinkUnitTests, getName)
{
    /**
     * @test Link name with a complete string
     */
    {
        robotlib::Link link{"link_test"};
        EXPECT_EQ(link.getName(), "link_test");
    }

    /**
     * @test Link name with two separate words
     */
    {
        robotlib::Link link{"link test"};
        EXPECT_EQ(link.getName(), "link test");
    }

    /**
     * @test Link name with an empty string
     */
    {
        robotlib::Link link{""};
        EXPECT_EQ(link.getName(), "");
    }

    /**
     * @test Link name with a single space character
     */
    {
        robotlib::Link link{" "};
        EXPECT_EQ(link.getName(), " ");
    }

    /**
     * @test Dummy robot links names
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        std::vector<std::string> links_name;
        for(auto& limb : limbs)
        {
            for(const auto& name : limb["links"])
            {
                links_name.push_back(name);
            }
        }
        unsigned int i {0};
        for (auto& link : dummy_robot->getLinks())
        {
            EXPECT_EQ(link->getName(), links_name[i]);
            i++;
        }
    }
}