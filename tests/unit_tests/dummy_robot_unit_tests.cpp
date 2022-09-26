#include <gtest/gtest.h>
#include "dummy_robot.hpp"

TEST(DummyRobotUnitTests, test)
{
     robotlib::DummyRobotCreator<4, 0, 12, 12, 3, 3> dummy_robot_creator;

     /* Component names = [Robot name | Leg names | Arms names | Joint names | Link names (+ Trunk)] */
     std::array<std::string, 30> components_names{"Dummy Robot",
                                                  "LF", "RF", "LH", "RH",
                                                  "LF_joint_1", "LF_joint_2", "LF_joint_3",
                                                  "RF_joint_1", "RF_joint_2", "RF_joint_3",
                                                  "LH_joint_1", "LH_joint_2", "LH_joint_3",
                                                  "RH_joint_1", "RH_joint_2", "RH_joint_3",
                                                  "LF_link_1", "LF_link_2", "LF_link_3",
                                                  "RF_link_1", "RF_link_2", "RF_link_3",
                                                  "LH_link_1", "LH_link_2", "LH_link_3",
                                                  "RH_link_1", "RH_link_2", "RH_link_3",
                                                  "TRUNK"};

     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
}