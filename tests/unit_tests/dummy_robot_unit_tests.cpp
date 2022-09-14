#include <gtest/gtest.h>
#include "dummy_robot.hpp"

TEST(DummyRobotUnitTests, test)
{
     robotlib::DummyRobotCreator<12, 12, 4, 0, 3, 3> dummy_robot_creator;
     auto dummy_robot = dummy_robot_creator.createDummyRobot();
     std::cout << dummy_robot->getName() << std::endl;
}