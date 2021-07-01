#include "robot_base.hpp"
// TODO: Change for this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

TEST(RobotBaseUnitTests, getTransform)
{
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = std::make_shared<dls::robotlib::DummyQuadruped>();
    auto q = dummy_quadruped->makeJointState();

    //std::string trunk{"TRUNK"}, lf_haa{"LF_HAA"};
    //Eigen::Matrix4d transform = dummy_quadruped->getTransform(q, dummy_quadruped->getLink(trunk), dummy_quadruped->getJoint(lf_haa));

    auto leg = dummy_quadruped->getLeg(0);
    Eigen::Matrix4d transform = dummy_quadruped->getTransform(q, leg.getLink(1), leg.getJoint(1));
}