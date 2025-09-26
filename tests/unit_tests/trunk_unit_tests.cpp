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
 * @brief Set of unit tests for Trunk::getName function
 */
TEST(TrunkUnitTests, getName)
{  
    /**
     * @test Dummy robot trunk name
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

        auto trunk_link{dummy_robot->getTrunk()};

        EXPECT_EQ(trunk_link->getName(), "TRUNK");
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
        robotlib::Trunk trunk{trunk_dyn_params};
        EXPECT_EQ(trunk.getCoM(), com);
    }
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
        robotlib::Trunk trunk{trunk_dyn_params};
        EXPECT_EQ(trunk.getMass(), mass);
    }
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
        robotlib::Trunk trunk{trunk_dyn_params};
        EXPECT_EQ(trunk.getInertia(), inertia);
    }
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
        robotlib::Trunk trunk{trunk_dyn_params};
        EXPECT_EQ(trunk.getDynParams().getCoM(), com);
        EXPECT_EQ(trunk.getDynParams().getMass(), mass);
        EXPECT_EQ(trunk.getDynParams().getInertia(), inertia);
    }
}