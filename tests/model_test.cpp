#include "robot.hpp"
#include "robot_factory.hpp"
#include <gtest/gtest.h>

using namespace std;
using namespace dls;
using namespace dls::robot;

TEST(ModelTest, robotLegs)
{
    /// Ground truth
    std::array<std::string, 4> hyq_legs{{
        std::string{"LF"},
        std::string{"RF"},
        std::string{"LH"},
        std::string{"RH"}
    }};
    
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 

    int i{0};
    for (auto l : *hyq) {
        ASSERT_EQ(l->getName(), hyq_legs.at(i++));
    }
}

TEST(ModelTest, legDataMap)
{
    /// Ground truth
    std::array<int, 4> hyq_leg_data_map{{0,1,2,3}};
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 
    auto leg_data_map = hyq->makeLegDataMap<int>();

    int i{0};
    for (auto leg_dm : leg_data_map) {
        leg_dm=i; 
        ASSERT_EQ(leg_dm, hyq_leg_data_map.at(i++));
    }
}

TEST(ModelTest, linkDataMap)
{
    /// Ground truth
    std::array<int, 8> hyq_link_data_map{{0,1,2,3,4,5,6,7}};
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 
    auto link_data_map = hyq->makeLinkDataMap<int>();
    
    int i{0};
    for (auto link_dm : link_data_map) {
        link_dm=i;
        ASSERT_EQ(link_dm, hyq_link_data_map.at(i++));
    }
}

TEST(ModelTest, jointDataMap)
{
    /// Ground truth
    std::array<int, 12> hyq_joint_data_map{{0,1,2,3,4,5,6,7,8,9,10,11}};
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 
    auto joint_data_map = hyq->makeJointDataMap<int>();

    int i{0};
    for (auto joint_dm : joint_data_map) {
        joint_dm=i;
        ASSERT_EQ(joint_dm, hyq_joint_data_map.at(i++));
    }
}

TEST(ModelTest, legDataMapPair)
{
    /// Ground truth
    std::array<std::string, 4> hyq_leg_data_map_pair{{
        std::string{"LF"},
        std::string{"RF"},
        std::string{"LH"},
        std::string{"RH"}
        }};
    std::array<int, 4> hyq_ldmp_ids{0,1,2,3};
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 
    auto leg_data_map_pair = hyq->makeLegDataMapPair<int>();

    ASSERT_EQ(leg_data_map_pair.getSize(), hyq_leg_data_map_pair.size());

    int i{0};
    for (auto leg_dmp : leg_data_map_pair) {
        leg_dmp.second=i;
        ASSERT_EQ(leg_dmp.first->getName(), hyq_leg_data_map_pair.at(i));
        ASSERT_EQ(leg_dmp.second, hyq_ldmp_ids.at(i++));
    }
}

TEST(ModelTest, linkDataMapPair)
{
    /// Ground truth
    std::array<std::pair<std::string, std::string>, 8> hyq_link_data_map_pair{{
        {std::string{"LF"}, std::string{"Upper leg"}},
        {std::string{"LF"}, std::string{"Lower leg"}},
        {std::string{"RF"}, std::string{"Upper leg"}},
        {std::string{"RF"}, std::string{"Lower leg"}},
        {std::string{"LH"}, std::string{"Upper leg"}},
        {std::string{"LH"}, std::string{"Lower leg"}},
        {std::string{"RH"}, std::string{"Upper leg"}},
        {std::string{"RH"}, std::string{"Lower leg"}}
    }};
    std::array<int, 8> hyq_ldmp_ids{0,1,2,3,4,5,6,7};
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 
    auto link_data_map_pair = hyq->makeLinkDataMapPair<int>();
    
    ASSERT_EQ(link_data_map_pair.getSize(), hyq_link_data_map_pair.size());

    int i{0};
    for (auto link_dmp : link_data_map_pair) {
        link_dmp.second=i;
        ASSERT_EQ(link_dmp.first->getParent()->getName(), hyq_link_data_map_pair.at(i).first);
        ASSERT_EQ(link_dmp.first->getName(), hyq_link_data_map_pair.at(i).second);
        ASSERT_EQ(link_dmp.second, hyq_ldmp_ids.at(i++));
    }
}

TEST(ModelTest, jointDataMapPair)
{
    /// Ground truth
    std::array<std::pair<std::string, std::string>, 12> hyq_joint_data_map_pair{{
        {std::string{"LF"}, std::string{"HAA"}},
        {std::string{"LF"}, std::string{"HFE"}},
        {std::string{"LF"}, std::string{"KFE"}},
        {std::string{"RF"}, std::string{"HAA"}},
        {std::string{"RF"}, std::string{"HFE"}},
        {std::string{"RF"}, std::string{"KFE"}},
        {std::string{"LH"}, std::string{"HAA"}},
        {std::string{"LH"}, std::string{"HFE"}},
        {std::string{"LH"}, std::string{"KFE"}},
        {std::string{"RH"}, std::string{"HAA"}},
        {std::string{"RH"}, std::string{"HFE"}},
        {std::string{"RH"}, std::string{"KFE"}}
    }};
    std::array<int, 12> hyq_jdmp_ids{0,1,2,3,4,5,6,7,8,9,10,11};
    /// HyQ
    auto hyq = RobotFactory::openRobot("hyq"); 
    auto joint_data_map_pair = hyq->makeJointDataMapPair<int>();
    
    ASSERT_EQ(joint_data_map_pair.getSize(), hyq_joint_data_map_pair.size());

    int i{0};
    for (auto joint_dmp : joint_data_map_pair) {
        joint_dmp.second=i;
        ASSERT_EQ(joint_dmp.first->getParent()->getName(), hyq_joint_data_map_pair.at(i).first);
        ASSERT_EQ(joint_dmp.first->getName(), hyq_joint_data_map_pair.at(i).second);
        ASSERT_EQ(joint_dmp.second, hyq_jdmp_ids.at(i++));
    }
}