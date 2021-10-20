// TODO: Change this include
#include <gtest/gtest.h>

#include "leg.hpp"
#include "joint.hpp"
#include "link.hpp"

const unsigned int NJOINTS = 20;
const unsigned int NLINKS = 15;

class RobotLeg : public dls::robotlib::Leg<NJOINTS, NLINKS>
{
public:
    RobotLeg(std::string name, const std::array<std::shared_ptr<const dls::robotlib::Joint>, NJOINTS> joints,
             const std::array<std::shared_ptr<const dls::robotlib::Link>, NLINKS> links)
        : Leg<NJOINTS, NLINKS>(name, joints, links){};

    virtual const std::string jointToChildName(const std::shared_ptr<dls::robotlib::Joint> joint) const
    {
        return "jointToChildName";
    };
    virtual const std::string jointToParentName(const std::shared_ptr<dls::robotlib::Joint> joint) const
    {
        return "jointToParentName";
    };

    virtual const std::string linkToChildName(const std::shared_ptr<dls::robotlib::Link> link) const
    {
        return "linkToChildName";
    };
    virtual const std::string linkToParentName(const std::shared_ptr<dls::robotlib::Link> link) const
    {
        return "linkToParentName";
    };
};

TEST(LegUnitTests, getName)
{
    std::cout << "Testing Leg getName()...\n";

    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    std::array<std::shared_ptr<const dls::robotlib::Joint>, NJOINTS> joints;
    std::array<std::shared_ptr<const dls::robotlib::Link>, NLINKS> links;

    for (int i = 0; i < NJOINTS; ++i)
    {
        std::string name{"joint_" + std::to_string(i)};
        joint_names.push_back(name);
        joints[i] = std::make_shared<dls::robotlib::Joint>(name);
    }
    for (int i = 0; i < NLINKS; ++i)
    {
        std::string name{"link_" + std::to_string(i)};
        link_names.push_back(name);
        links[i] = std::make_shared<dls::robotlib::Link>(name);
    }

    RobotLeg leg("leg_1", joints, links);

    int i = 0;
    for (auto link : *leg.getLinks())
    {
        EXPECT_EQ(link->getName(), link_names[i]);
        i++;
    }
    i = 0;
    for (auto link : *leg.getJoints())
    {
        EXPECT_EQ(link->getName(), joint_names[i]);
        i++;
    }
}

TEST(LegUnitTests, getNLinks)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getNJoints)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getLink)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getJoint)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getJoints)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getLinks)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, jointToChildName)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, jointToParentName)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, linkToChildName)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, linkToParentName)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getLink_string)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getJoint_string)
{
    std::cout << "TODO\n";
}
TEST(LegUnitTests, getEndEffector)
{
    std::cout << "TODO\n";
}
