#include "robot.hpp"
#include "robot_factory.hpp"
#include <gtest/gtest.h>
#include <stdlib.h>

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

static void info(const std::shared_ptr<dls::robotlib::RobotBase> &robot)
{
    std::cout << "\nINFO ON ROBOT " << robot->getName() << std::endl;

    std::cout << "\n*** LEGS OF " << robot->getName() << " ***" << std::endl;
    for (auto leg : *robot->getLegs())
    {
        std::cout << leg->getName() << std::endl;
    }

    std::cout << "\n*** LINKS FOR EACH LEG OF " << robot->getName() << " ***" << std::endl;
    for (auto leg : *robot->getLegs())
    {
        std::cout << leg->getName() << ":  ";
        int nLinks = leg->getNLinks();
        for (int link = 0; link < nLinks; ++link)
        {
            if (link == nLinks - 1)
                std::cout << std::static_pointer_cast<dls::robotlib::Link>(leg->getLink(link))->getName() << std::endl;
            else
                std::cout << std::static_pointer_cast<dls::robotlib::Link>(leg->getLink(link))->getName() << ", ";
        }
    }

    std::cout << "\n*** JOINTS FOR EACH LEG OF " << robot->getName() << " ***" << std::endl;
    for (auto leg : *robot->getLegs())
    {
        std::cout << leg->getName() << ":  ";
        int nJoints = leg->getNJoints();
        for (int joint = 0; joint < nJoints; ++joint)
        {
            std::cout << std::static_pointer_cast<dls::robotlib::Joint>(leg->getJoint(joint))->getName() << ", ";
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

int main(int argc, char *argv[])
{
    if (argc <= 1 || (argc == 2 && strcmp(argv[1], "--help") != 0))
    {
        display_help(argv[0]);
        return 1;
    }

    /// ROBOT ARGUMENTS
    const std::string robotType = argv[1];
    std::shared_ptr<dls::robotlib::RobotBase> robot;

    try
    {
        robot = dls::robotlib::RobotFactory::openRobot(argv[1]);

        /// OPTION ARGUMENTS
        if (strcmp(argv[2], "--info") == 0)
        {
            info(robot);
        }
        else
        {
            display_help(argv[0]);
            return 1;
        }
    }
    catch (const std::string &e)
    {
        std::cout << e << std::endl;
    }
}
