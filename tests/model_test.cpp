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


TEST(robotLib, legs){
    int i=0;
    
    // Create hyq robot robot
    Hyq hyq;

    std::cout << "For each leg in robot" << std::endl;
    
    for (auto l : hyq) {
        std::cout << l->getName() << std::endl;
    }
    
    LegDataMap<int> leg_data_map(hyq.getNLEGS());

    std::cout << "For each value in leg data map" << std::endl;
    for (auto x : leg_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

    LegDataMap<int> link_data_map(hyq.getNLINKS());

    i=0;
    std::cout << "For each value in a link data map" << std::endl;
    for (auto x : link_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

    LegDataMap<int> joint_data_map(hyq.getNJOINTS());
    
    std::cout << "For each value in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

    Hyq::LegDataMapPair<int> leg_data_map_pair(hyq);
    std::cout << "For each pair in leg data map" << std::endl;
    i=0;
    for (auto x : leg_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getName() << "=" << *x.second << std::endl;
    }
    
    Hyq::LinkDataMapPair<int> link_data_map_pair(hyq);
    std::cout << "For each pair in link data map" << std::endl;
    i=0;
    for (auto x : link_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << *x.second << std::endl;
    }

    Hyq::JointDataMapPair<int> joint_data_map_pair(hyq);
    std::cout << "For each pair in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << *x.second << std::endl;
    }
}