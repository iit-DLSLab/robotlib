#include <stdlib.h>

#include "robot.hpp"
#include "robot_factory.hpp"
#include <gtest/gtest.h>


using namespace std;
using namespace dls;
using namespace dls::robot;

int main(int argc, char *argv[])
{   
    
    if (argc<=1){
        std::cout<< "Please specify the type of robot: hyq or hyqreal"<<std::endl;
        exit (EXIT_FAILURE);
    }
    const string robotType = argv[1];
    std::cout << "INFO ON ROBOT " << robotType << '\n';


    int i=0;
    
    // Create robot
    auto robot = RobotFactory::openRobot("hyq"); 
    
    robot->getLegsName();
    robot->getLinksName();
    robot->getJointsName();

    auto leg_data_map = robot->makeLegDataMap<int>();

    std::cout << "For each value in leg data map" << std::endl;
    for (auto x : leg_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

    auto link_data_map = robot->makeLinkDataMap<int>();

    i=0;
    std::cout << "For each value in a link data map" << std::endl;
    for (auto x : link_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

    auto joint_data_map = robot->makeJointDataMap<int>();
    
    std::cout << "For each value in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map) {
        x=i++;
        std::cout << x << std::endl;
    }

    auto leg_data_map_pair = robot->makeLegDataMapPair<int>();
    
    std::cout << "For each pair in leg data map" << std::endl;
    i=0;
    for (auto x : leg_data_map_pair) {
        x.second=i++;
        std::cout << x.first->getName() << "=" << x.second << std::endl;
    }
    
    auto link_data_map_pair = robot->makeLinkDataMapPair<int>();

    std::cout << "For each pair in link data map" << std::endl;
    i=0;
    for (auto x : link_data_map_pair) {
        x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << x.second << std::endl;
    }
    
    auto joint_data_map_pair = robot->makeJointDataMapPair<int>();
    
    std::cout << "For each pair in joint data map" << std::endl;
    i=0;
    for (auto x : joint_data_map_pair) {
        x.second=i++;
        std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << x.second << std::endl;
    }
}