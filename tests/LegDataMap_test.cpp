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
    { 
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


template<class T>
class LegDataMap{
public:
    LegDataMap(RobotBase* robot): nLegs_(robot->getNLEGS()){    
        T *p = new T[nLegs_];
        std::shared_ptr<T> pshrd(p);
        data_ = pshrd;
    }
    ~LegDataMap(){};
    
    Iterator<T> begin() { return Iterator<T>(&data_.get()[0]); }
    Iterator<T> end() { return Iterator<T>(&data_.get()[nLegs_]); }
private:
    std::shared_ptr<T> data_;
    const int nLegs_;
};

TEST(robotLib, legDataMap){
    RobotBase *robot = new Hyq;

    std::cout << "For each leg in robot" << std::endl;
    
    for (auto l : *robot) {
        std::cout << (*l).getName() << std::endl;
    }

    LegDataMap<int> leg_data_map(robot);
    
    std::cout << "For each value in leg data map" << std::endl;
    int i=0;
    for (auto x : leg_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }
 
}