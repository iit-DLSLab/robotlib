#include <gtest/gtest.h>
#include "leg.hpp"
#include "joint.hpp"
#include "link.hpp"

const unsigned int NJOINTS {20};
const unsigned int NLINKS {15};

/*!
* @brief RobotLeg class.
* @details
* This class allows to create a robt leg having a specific morphology with the only purpose of testing Robotlib structures.
*/
class RobotLeg : public robotlib::Leg<NJOINTS, NLINKS>
{
public:
    /*!
    * @brief Constructor.
    * @param[in] name name of the leg.
    * @param[in] joints array of shared pointers pointing to leg's joints.
    * @param[in] links array of shared pointers pointing to leg's links.
    */
    RobotLeg(std::string name, const std::array<std::shared_ptr<robotlib::Joint>, NJOINTS> joints,
             const std::array<std::shared_ptr<robotlib::Link>, NLINKS> links)
        : Leg<NJOINTS, NLINKS>(name, joints, links){};

    /*!
    * @brief Get the name of the joint's child of the leg.
    * @param[in] joint Joint object for which you get the name of the child.
    * @return name of the joint's child.
    */
    virtual std::string jointToChildName(const std::shared_ptr<robotlib::Joint> joint) const
    {
        joint->getName();

        return "jointToChildName";
    };

    /*!
    * @brief Get the name of the joint's parent of the leg.
    * @param[in] joint Joint object for which you get the name of the parent.
    * @return name of the joint's parent. 
    */
    virtual std::string jointToParentName(const std::shared_ptr<robotlib::Joint> joint) const
    {
        joint->getName();

        return "jointToParentName";
    };

    /*!
    * @brief Get the name of the link's child of the leg.
    * @param[in] link Link object for which you get the name of the child.
    * @return name of the link's child.
    */
    virtual std::string linkToChildName(const std::shared_ptr<robotlib::Link> link) const
    {
        link->getName();

        return "linkToChildName";
    };

    /*!
    * @brief Get the name of link's parent of the leg.
    * @param[in] link Link object for which you get the name of the parent.
    * @return name of the link's parent.
    */
    virtual std::string linkToParentName(const std::shared_ptr<robotlib::Link> link) const
    {
        link->getName();

        return "linkToParentName";
    };
};

TEST(LimbUnitTests, getName)
{
    std::cout << "Testing Leg getName()...\n";

    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    std::array<std::shared_ptr<robotlib::Joint>, NJOINTS> joints;
    std::array<std::shared_ptr<robotlib::Link>, NLINKS> links;

    for (unsigned int i {0}; i < NJOINTS; ++i)
    {
        std::string name{"joint_" + std::to_string(i)};
        joint_names.push_back(name);
        joints[i] = std::make_shared<robotlib::Joint>(name);
    }
    for (unsigned int i {0}; i < NLINKS; ++i)
    {
        std::string name{"link_" + std::to_string(i)};
        link_names.push_back(name);
        links[i] = std::make_shared<robotlib::Link>(name);
    }

    RobotLeg leg("leg_1", joints, links);

    int i {0};
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

TEST(LimbUnitTests, getNLinks)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getNJoints)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getLink)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getJoint)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getJoints)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getLinks)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, jointToChildName)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, jointToParentName)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, linkToChildName)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, linkToParentName)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getLink_string)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getJoint_string)
{
    std::cout << "TODO\n";
}
TEST(LimbUnitTests, getEndEffector)
{
    std::cout << "TODO\n";
}