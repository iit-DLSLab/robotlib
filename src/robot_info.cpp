/**
 * @file robot_info.cpp
 *
 * @brief Functions to display robots information
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "robot.hpp"
#include "robot_factory.hpp"
#include <stdlib.h>

namespace robotlib
{
    static void display_message(std::string message)
    {
        if (message.compare("manual") == 0)
        {
            std::cerr << "\n*** Robot Info manual ***\n"
                      << "\nCommands:\n"
                      << "1. ./robot_info --help\t\t\t[Show user manual]\n"
                      << "2. ./robot_info <robot_library> --info\t[Show robot information]\n"
                      << "\nRobot libraries:\n"
                      << "- dummy-quadruped\n"
                      << std::endl;
        }
        else if (message.compare("suggestion") == 0)
        {
            std::cout << "\nTo read Robot Info manual, run:\n"
                      << std::endl;
            std::cout << "robot_info --help\n"
                      << std::endl;
        }
        else
            std::cout << "Is this a message or a suggestion?" << std::endl;
    }

    static void info(const std::shared_ptr<robotlib::RobotBase> &robot)
    {
        std::cout << "\nINFO ON ROBOT " << robot->getName() << std::endl;

        std::cout << "\n*** LIMBS OF " << robot->getName() << " ***" << std::endl;
        for (auto& limb : robot->getLimbs())
        {
            std::cout << limb.getName() << std::endl;
        }

        std::cout << "\n*** LINKS FOR EACH LIMB OF " << robot->getName() << " ***" << std::endl;
        for (auto& limb : robot->getLimbs())
        {
            std::cout << limb.getName() << ":  ";
            unsigned int nLinks {limb.getNLinks()};

            unsigned int count_links {0};
            for (auto& link : limb.getLinks())
            {
                if (count_links == nLinks - 1)
                    std::cout << link.getName() << std::endl;
                else
                    std::cout << link.getName() << ", ";
                count_links++;
            }
        }

        std::cout << "\n*** JOINTS FOR EACH LIMB OF " << robot->getName() << " ***" << std::endl;
        for (auto& limb : robot->getLimbs())
        {
            std::cout << limb.getName() << ":  ";
            int count_joints {0};
            for (auto& joint : limb.getJoints())
            {
                std::cout << joint.getName() << ", ";
            }
            std::cout << '\n';
            count_joints++;
        }
        std::cout << '\n';
    }
} // namespace robotlib

/**
 * @brief Main function called to display robots information
 *
 * @param [in] argc this is the number of
 * @param [in] argv
 * @return int
 */
int main(int argc, char *argv[])
{
    switch (argc)
    {
    case 2:
    {
        if (strcmp(argv[1], "--help") == 0)
            robotlib::display_message("manual");
        else
            robotlib::display_message("suggestion");
        break;
    }
    case 3:
    {
        /// ROBOT ARGUMENTS
        std::shared_ptr<robotlib::RobotBase> robot;

        if (strcmp(argv[2], "--info") == 0)
        {
            try
            {
                /// argv[1] is the robot library used
                robot = robotlib::RobotFactory::openRobot(argv[1]);
                robotlib::info(robot);
            }
            catch (const std::string &e)
            {
                std::cout << e << std::endl;
            }
        }
        else
            robotlib::display_message("suggestion");
        break;
    }
    default:
    {
        robotlib::display_message("suggestion");
    }
    }
}