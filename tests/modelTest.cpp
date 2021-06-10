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


// Define dummy robot for testing
class HyqLeg : public Leg<NJOINTS,NLINKS> {
public:
    HyqLeg(std::string name) : Leg<NJOINTS,NLINKS>(
        name,
        std::array<std::shared_ptr<Joint>,NJOINTS>({
            std::make_shared<Joint>("HAA"),
            std::make_shared<Joint>("HFE"),
            std::make_shared<Joint>("KFE")
            }),
        std::array<std::shared_ptr<Link>,NLINKS>({
            std::make_shared<Link>( "Upper leg"),
            std::make_shared<Link>( "Lower leg")
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
        
    })) { 
        // std::cout << "-----------------------------------" << std::endl;
        // for (int i=0;i<4;i++) {
        //     std::cout << legs_[i]->getName() << std::endl;
        //     for (int j=0;j<3;j++)
        //         std::cout << legs_[i]->joints_[j]->getName() << ",";
        //     for (int j=0;j<2;j++)
        //         std::cout << legs_[i]->links_[j]->getName() << ",";
        //     std::cout << std::endl;
        // }
        // std::cout << "-----------------------------------" << std::endl;
    
    }
};


 

TEST(robotLib, legs){
    int i=0;
    
    // Create dummy robot
    Hyq hyq;

    std::cout << "For each leg in robot" << std::endl;
    
    for (auto l : hyq) {
        std::cout << l->getName() << std::endl;
    }
    
    Hyq::LegDataMap<int> leg_data_map;
    std::cout << "For each value in leg data map" << std::endl;
    for (auto x : leg_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }
    
    Hyq::LinkDataMap<int> link_data_map;
    i=0;
    std::cout << "For each value in a link data map" << std::endl;
    for (auto x : link_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }
    
    Hyq::JointDataMap<int> joint_data_map;
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
        std::cout << x.first->getName() << "=" << *x.second << std::endl;
    }

    Hyq::JointDataMapPair<int> joint_data_map_pair(hyq);
    std::cout << "For each pair in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map_pair) {
        *x.second=i++;
        std::cout << x.first->getName() << "=" << *x.second << std::endl;
    }


    // // Create a set of legs
    // std::array<std::shared_ptr<LimbBase>, NLEGS> legs;

    // legs[0] = std::make_shared<Leg<3,3>>("FL");
    // legs[1] = std::make_shared<Leg<2,2>>("FR");
    // legs[2] = std::make_shared<Leg<1,2>>("RL");
    // legs[3] = std::make_shared<Leg<3,2>>("RR");

    // // Create a link and joint
    // Joint joint = Joint ("haa");
    // Link link = Link ("l1");
    
    // // Test get and set functions on legs
    // int i=3;
    // r.setLeg(legs[i],i); //or all at once with r->setLegs(legs)

    // std::cout << "LEG " << legs[i]->getName() << '\n';
    
    // int nLinks = legs[i]->getNumLinks();
    // int nJoints = legs[i]->getNumJoints();

    // std::cout << "\tLINKS " << '\n';
    // for (int idL = 0; idL<nLinks; ++idL){
    //     legs[i]->setLink(link, idL);
    //     std::cout << "\t" << legs[i]->getLink(idL).getName() <<  "\n";
    // }

    // std::cout << "\tJOINTS " << '\n';
    // for (int idJ = 0; idJ<nJoints; ++idJ){
    //     legs[i]->setJoint(joint, idJ);
    //     std::cout << "\t" << legs[i]->getJoint(idJ).getName() <<  "\n";
    // }
    

    //legs[0].getLinks();    
    // for (int i=0;i<4;++i){
    //    std::shared_ptr<LimbBase> leg = r.getLeg(i);

    //     std::cout << leg->getName() << "n. links " << leg->getLinks().size << "n. joints " << leg->getJoints().size <<'\n';
    // }
}