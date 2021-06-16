#include "robot.hpp"
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


TEST(robotLib, legDataMap){
    RobotBase *robot = new Hyq;

    std::cout << "For each leg in robot" << std::endl;
    
    for (auto l : *robot) {
        std::cout << l->getName() << std::endl;
    }

    auto leg_data_map = robot->makeLegDataMap<int>();
    
    // std::cout << "For each value in leg data map" << std::endl;
    // int i=0;
    // for (auto x : leg_data_map) {
    //     x=i++;
    //     std::cout << x << std::endl;
    // }
}

TEST(robotLib, jointDataMap){
    RobotBase *robot = new Hyq;

    auto joint_data_map = robot->makeJointDataMap<int>();
    
    std::cout << "For each value in joint data map" << std::endl;
    int i=0;
    for (auto x : joint_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }
}

TEST(robotLib, linkDataMap){
    RobotBase *robot = new Hyq;

    auto link_data_map = robot->makeLinkDataMap<int>();
    
    std::cout << "For each value in link data map" << std::endl;
    int i=0;
    for (auto x : link_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }
}