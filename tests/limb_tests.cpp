// TODO: Change this include
#include <gtest/gtest.h>

#include "leg.hpp"
#include "joint.hpp"
#include "link.hpp"

const unsigned int NJOINTS = 2;
const unsigned int NLINKS = 2;

class RobotLeg : public dls::robotlib::Leg<NJOINTS, NLINKS>
{
public:
    RobotLeg(std::string name, const std::array<const std::shared_ptr<const dls::robotlib::Joint>, NJOINTS> joints,
             const std::array<const std::shared_ptr<const dls::robotlib::Link>, NLINKS> links)
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

    std::cout << "Testing Limb constructor...\n";

    const std::shared_ptr<const dls::robotlib::Joint> joint_1 = std::make_shared<const dls::robotlib::Joint>("Joint_1");
    const std::shared_ptr<const dls::robotlib::Joint> joint_2 = std::make_shared<const dls::robotlib::Joint>("Joint_2");
    const std::shared_ptr<const dls::robotlib::Link> link_1 = std::make_shared<const dls::robotlib::Link>("Link_1");
    const std::shared_ptr<const dls::robotlib::Link> link_2 = std::make_shared<const dls::robotlib::Link>("Link_2");

    const std::array<const std::shared_ptr<const dls::robotlib::Joint>, NJOINTS> joints = {joint_1, joint_2};
    const std::array<const std::shared_ptr<const dls::robotlib::Link>, NLINKS> links = {link_1, link_2};

    RobotLeg leg("leg_1", joints, links);
}

const std::string getName() const;

virtual const int getNLinks() const = 0;
virtual const int getNJoints() const = 0;

virtual const std::shared_ptr<const Link> getLink(const int linkId) const = 0;
virtual const std::shared_ptr<const Joint> getJoint(const int jointId) const = 0;

virtual const std::shared_ptr<const ContainerBase<const std::shared_ptr<const Joint>>> getJoints() = 0;
virtual const std::shared_ptr<const ContainerBase<const std::shared_ptr<const Link>>> getLinks() = 0;

virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) const = 0;  //overridden by Glue
virtual const std::string jointToParentName(const std::shared_ptr<Joint> joint) const = 0; //overridden by Glue

virtual const std::string linkToChildName(const std::shared_ptr<Link> joint) const = 0;  //overridden by Glue
virtual const std::string linkToParentName(const std::shared_ptr<Link> joint) const = 0; //overridden by Glue

virtual const std::shared_ptr<const Link> getLink(const std::string &name) const = 0;
virtual const std::shared_ptr<const Joint> getJoint(const std::string &name) const = 0;

virtual const std::shared_ptr<const Link> getEndEffector() const = 0; //overridden by Limb class