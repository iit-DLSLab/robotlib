#include "robot.hpp"

#include <gtest/gtest.h>

using namespace std;
using namespace dls;
using namespace dls::robot;

#define NLEGS 4
#define NLINKS 2
#define NJOINTS 3
#define NLINKS_TOT NLEGS*NLINKS
#define NJOINTS_TOT NLEGS*NJOINTS


// Define hyq robot for testing
class HyqLeg : public Leg<NJOINTS,NLINKS> {
public:
    HyqLeg(std::string name) : Leg<NJOINTS,NLINKS>(
        name,
        std::array<std::shared_ptr<Joint>,NJOINTS>({
            std::make_shared<Joint>(this, "HAA"),
            std::make_shared<Joint>(this, "HFE"),
            std::make_shared<Joint>(this, "KFE")
            }),
        std::array<std::shared_ptr<Link>,NLINKS>({
            std::make_shared<Link>(this, "Upper leg"),
            std::make_shared<Link>(this,  "Lower leg")
            })
        ) { }
};

class Hyq : public Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT> {
public:
    Hyq() :Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT>(std::array<std::shared_ptr<LimbBase>,NLEGS>({
        std::make_shared<HyqLeg>("LF"),
        std::make_shared<HyqLeg>("RF"),
        std::make_shared<HyqLeg>("LH"),
        std::make_shared<HyqLeg>("RH"),
        
    })) { }
};

TEST(ModelTest, robotLegs)
{
    /// Ground truth
    std::array<std::shared_ptr<LimbBase>,4> hyq_legs{
        std::make_shared<HyqLeg>("LF"),
        std::make_shared<HyqLeg>("RF"),
        std::make_shared<HyqLeg>("LH"),
        std::make_shared<HyqLeg>("RH")
    };
    /// HyQ
    Hyq hyq;
    int i{0};

    for (auto l : hyq) {
        ASSERT_EQ(l->getName(), hyq_legs.at(i)->getName());
        i++;
    }
}

TEST(ModelTest, legDataMap)
{
    /// Ground truth
    std::array<int, 4> hyq_leg_data_map{0,1,2,3};
    /// HyQ
    Hyq hyq;
    LegDataMap<int> leg_data_map(hyq.getNLEGS());

    int i{0}, j{0};
    for (auto leg_dm : leg_data_map) {
        leg_dm=i++; 
        ASSERT_EQ(leg_dm, hyq_leg_data_map.at(j++));
    }
}

TEST(ModelTest, linkDataMap)
{
    /// Ground truth
    std::array<int, 8> hyq_link_data_map{0,1,2,3,4,5,6,7};
    /// HyQ
    Hyq::LinkDataMap<int> link_data_map;
    
    int i{0}, j{0};
    for (auto link_dm : link_data_map) {
        link_dm=i++;
        ASSERT_EQ(link_dm, hyq_link_data_map.at(j++));
    }
}

TEST(ModelTest, jointDataMap)
{
    /// Ground truth
    std::array<int, 12> hyq_joint_data_map{0,1,2,3,4,5,6,7,8,9,10,11};
    /// HyQ
    Hyq::JointDataMap<int> joint_data_map;

    int i{0}, j{0};
    for (auto joint_dm : joint_data_map) {
        joint_dm=i++;
        ASSERT_EQ(joint_dm, hyq_joint_data_map.at(j++));
    }
}

TEST(ModelTest, legDataMapPair)
{
    /// Ground truth
    std::map<std::shared_ptr<LimbBase>,int> hyq_leg_data_map_pair{
        {std::make_shared<HyqLeg>("LF"), 0},
        {std::make_shared<HyqLeg>("RF"), 1},
        {std::make_shared<HyqLeg>("LH"), 2},
        {std::make_shared<HyqLeg>("RH"), 3}
    };
    /// HyQ
    Hyq hyq;
    Hyq::LegDataMapPair<int> leg_data_map_pair(hyq);

    size_t ldmp_size{leg_data_map_pair.size()};
    size_t hyq_ldmp_size{hyq_leg_data_map_pair.size()};

    ASSERT_EQ(ldmp_size, hyq_ldmp_size);
    
    std::map<std::string,int> hyq_and_ground_truth;
    
    int i{0};
    for (auto leg_dmp : leg_data_map_pair) {
        *leg_dmp.second=i++;
        hyq_and_ground_truth.insert(std::pair<std::string, int>(leg_dmp.first->getName(), *leg_dmp.second));
    }

    for(auto hyq_leg_dmp: hyq_leg_data_map_pair){
        hyq_and_ground_truth.insert(std::pair<std::string, int>(hyq_leg_dmp.first->getName(), hyq_leg_dmp.second));
    }

    size_t hgt_size{hyq_and_ground_truth.size()};

    /// The size of the final "hyq_and_ground_truth" map should be the same as before, since maps do not contains duplicates.
    ASSERT_EQ(hgt_size, ldmp_size);
}

TEST(ModelTest, linkDataMapPair)
{
    /// Ground truth
    std::map<std::string, std::string> hyq_link_data_map_pair{
        {std::string{"LF"}, std::string{"Upper leg"}},
        {std::string{"LF"}, std::string{"Lower leg"}},
        {std::string{"RF"}, std::string{"Upper leg"}},
        {std::string{"RF"}, std::string{"Lower leg"}},
        {std::string{"LH"}, std::string{"Upper leg"}},
        {std::string{"LH"}, std::string{"Lower leg"}},
        {std::string{"RH"}, std::string{"Upper leg"}},
        {std::string{"RH"}, std::string{"Lower leg"}}
    };
    std::array<int, 8> hyq_link_data_map_pair_ids{0,1,2,3,4,5,6,7};
    /// HyQ
    Hyq hyq;
    Hyq::LinkDataMapPair<int> link_data_map_pair(hyq);
    
    int i{0};
    for (auto link_dmp : link_data_map_pair) {
        *link_dmp.second=i++;
        std::cout << link_dmp.first->getParent()->getName() << "," << link_dmp.first->getName() << "=" << *link_dmp.second << std::endl;
    }
}

TEST(robotLib, legs){
    int i=0;
    
    // Create hyq robot robot
    Hyq hyq;

    Hyq::JointDataMapPair<int> joint_data_map_pair(hyq);
    std::cout << "For each pair in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << *x.second << std::endl;
    }
}