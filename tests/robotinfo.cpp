#include <stdlib.h>

#include "robot.hpp"
#include "robot_factory.hpp"
#include <gtest/gtest.h>

using namespace std;
using namespace dls;
using namespace dls::robot;

static void display_help(std::string name)
{
    std::cerr << "Usage:\n"
              << "\t" << name << " --help\t\t\tShow this help message\n"
              << "\t" << name << " Robot_type <option>\t\tExecute the option\n"
              << "Robot_type:\n"
              << "\t hyq\n"
              << "\t hyqreal\n"
              << "Options:\n"
              << "\t--info\t\tShow info of the robot\n"
              << std::endl;
}
static void info(const std::shared_ptr<RobotBase> &robot)
{
    std::cout << "INFO ON ROBOT " << robot->getName() << '\n';
    robot->getLegsName();
    robot->getLinksName();
    robot->getJointsName();
}

int main(int argc, char *argv[])
{
    if (argc <= 1 || (argc == 2 && strcmp(argv[1], "--help") != 0))
    {
        display_help(argv[0]);
        std::cout << "failure\n";
        exit(EXIT_FAILURE);
    }

    // ROBOT ARGUMENTS
    const string robotType = argv[1];
    std::shared_ptr<RobotBase> robot;
    if (strcmp(argv[1], "hyq") == 0)
        robot = RobotFactory::openRobot("hyq");
    else if (strcmp(argv[1], "hyqreal") == 0)
    {
        std::cout << "Hyqreal robot has not been implemented yet\n";
        exit(EXIT_FAILURE);
    }
    else if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)
    {
        display_help(argv[0]);
        exit(EXIT_SUCCESS);
    }

    // OPTION ARGUMENTS
    if (strcmp(argv[2], "--info") == 0)
    {
        info(robot);
    }
    else
    { // if a wrong option is passed as input, the help message is showed
        display_help(argv[0]);
        exit(EXIT_FAILURE);
    }

    // int i=0;
    // auto leg_data_map = robot->makeLegDataMap<int>();

    // std::cout << "For each value in leg data map" << std::endl;
    // for (auto x : leg_data_map) {
    //     x=i++;
    //     std::cout << x << std::endl;
    // }

    // auto link_data_map = robot->makeLinkDataMap<int>();

    // i=0;
    // std::cout << "For each value in a link data map" << std::endl;
    // for (auto x : link_data_map) {
    //     x=i++;
    //     std::cout << x << std::endl;
    // }

    // auto joint_data_map = robot->makeJointDataMap<int>();

    // std::cout << "For each value in joint data map" << std::endl;
    // i=0;
    // for (auto x : joint_data_map) {
    //     x=i++;
    //     std::cout << x << std::endl;
    // }

    // auto leg_data_map_pair = robot->makeLegDataMapPair<int>();

    // std::cout << "For each pair in leg data map" << std::endl;
    // i=0;
    // for (auto x : leg_data_map_pair) {
    //     x.second=i++;
    //     std::cout << x.first->getName() << "=" << x.second << std::endl;
    // }

    // auto link_data_map_pair = robot->makeLinkDataMapPair<int>();

    // std::cout << "For each pair in link data map" << std::endl;
    // i=0;
    // for (auto x : link_data_map_pair) {
    //     x.second=i++;
    //     std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << x.second << std::endl;
    // }

    // auto joint_data_map_pair = robot->makeJointDataMapPair<int>();

    // std::cout << "For each pair in joint data map" << std::endl;
    // i=0;
    // for (auto x : joint_data_map_pair) {
    //     x.second=i++;
    //     std::cout << x.first->getParent()->getName() << "," << x.first->getName() << "=" << x.second << std::endl;
    // }
}