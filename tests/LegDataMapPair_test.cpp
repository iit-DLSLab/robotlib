#include "robot.hpp"
#include "joint_data_map_pair.hpp"
#include "link_data_map_pair.hpp"

#include <gtest/gtest.h>
#include <utils.hpp>

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
    Hyq() :Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT>(
        
        std::array<std::shared_ptr<LimbBase>,NLEGS>({
        std::make_shared<HyqLeg>("LF"),
        std::make_shared<HyqLeg>("RF"),
        std::make_shared<HyqLeg>("LH"),
        std::make_shared<HyqLeg>("RH"),    
         })
    ) 
    { }
};


TEST(robotLib, legDataMapPair){
    auto robot = std::make_shared<Hyq>();
    
    auto leg_data_map_pair = robot->makeLegDataMapPair<int>();

    int i = 0;
    std::cout << "For each pair in leg data map" << std::endl;
    i=0;
    for (auto x : leg_data_map_pair) {
        x.second=i++;
        std::cout << x.first->getName() << "=" << x.second << std::endl;
    }

}

TEST(robotLib, jointDataMapPair){
    auto robot = std::make_shared<Hyq>();
 
    auto joint_data_map_pair = robot->makeJointDataMapPair<int>();

    std::cout << "For each value in joint data map" << std::endl;
    int i=0;
    for (auto x : joint_data_map_pair) {
        x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << x.second << std::endl;
    }
}

TEST(robotLib, linkDataMapPair){
    auto robot = std::make_shared<Hyq>();
 
    auto link_data_map_pair = robot->makeLinkDataMapPair<int>();

    std::cout << "For each value in link data map" << std::endl;
    int i=0;
    for (auto x : link_data_map_pair) {
        x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << x.second << std::endl;
    }
}