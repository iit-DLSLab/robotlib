#include <gtest/gtest.h>
#include "robot_factory.hpp"

// In order to do the tests you need to install the dummy robots.
// To do so, just do make install inside the build folder of robotlib, from docker root terminal. 

TEST(RobotBaseUnitTests, getNLegs)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto n_legs_dq{dummy_quadruped->getNLEGS()};

    /// Ground truth
    int n_legs_gt{4};

    /// Assert conditions
    ASSERT_EQ(n_legs_dq, n_legs_gt);
    ASSERT_EQ(typeid(n_legs_dq), typeid(n_legs_gt));
}

TEST(RobotBaseUnitTests, getNJoints)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto n_joints_dq{dummy_quadruped->getNJOINTS()};

    /// Ground truth
    int n_joints_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_joints_dq, n_joints_gt);
    ASSERT_EQ(typeid(n_joints_dq), typeid(n_joints_gt));
}

TEST(RobotBaseUnitTests, getNLinks)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto n_links_dq{dummy_quadruped->getNLINKS()};

    /// Ground truth
    int n_links_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_links_dq, n_links_gt);
    ASSERT_EQ(typeid(n_links_dq), typeid(n_links_gt));
}

TEST(RobotBaseUnitTests, getLeg)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    /// Ground truth
    std::array<std::string, 4> legs_gt{{"LF", "RF", "LH", "RH"}};

    int count_legs {0};
    for (auto legs_dq : *dummy_quadruped->getLegs())
    {
        /// Assert conditions
        ASSERT_EQ(legs_dq->getName(), legs_gt.at(count_legs));
        ASSERT_EQ(typeid(legs_dq->getName()), typeid(legs_gt.at(count_legs)));
        count_legs++;
    }
}

TEST(RobotBaseUnitTests, getFramePosition)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Vector3d frame_position = dummy_quadruped->getFramePosition(joint_state,
                                                                       dummy_quadruped->getLink("TRUNK"),
                                                                       dummy_quadruped->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(frame_position, Eigen::Vector3d().setZero());
}

TEST(RobotBaseUnitTests, getFrameOrientation)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix3d frame_orientation = dummy_quadruped->getFrameOrientation(joint_state,
                                                                             dummy_quadruped->getLink("TRUNK"),
                                                                             dummy_quadruped->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(frame_orientation, Eigen::Matrix3d().setZero());
}

TEST(RobotBaseUnitTests, getFramePose)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix4d frame_pose_dq = dummy_quadruped->getFramePose(joint_state,
                                                                  dummy_quadruped->getLink("TRUNK"),
                                                                  dummy_quadruped->getLink("LF_UPPERLEG"));
    /// Ground truth
    Eigen::Matrix4d frame_pose_gt;
    frame_pose_gt.setZero();
    frame_pose_gt(3, 3) = 1;

    /// Assert conditions
    ASSERT_EQ(frame_pose_dq, frame_pose_gt);
    ASSERT_EQ(typeid(frame_pose_dq), typeid(frame_pose_gt));
}

TEST(RobotBaseUnitTests, getFootPosition)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Vector3d foot_position = dummy_quadruped->getFootPosition(joint_state,
                                                                     dummy_quadruped->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(foot_position, Eigen::Vector3d().setZero());
}

TEST(RobotBaseUnitTests, getFootOrientation)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix3d foot_orientation = dummy_quadruped->getFootOrientation(joint_state,
                                                                           dummy_quadruped->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(foot_orientation, Eigen::Matrix3d().setZero());
}

TEST(RobotBaseUnitTests, getFootPose)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    Eigen::Matrix4d foot_pose_dq = dummy_quadruped->getFootPose(joint_state,
                                                                dummy_quadruped->getLink("LF_LOWERLEG"));

    /// Ground truth
    Eigen::Matrix4d foot_pose_gt;
    foot_pose_gt.setZero();
    foot_pose_gt(3, 3) = 1;

    /// Assert conditions
    ASSERT_EQ(foot_pose_dq, foot_pose_gt);
    ASSERT_EQ(typeid(foot_pose_dq), typeid(foot_pose_gt));
}

TEST(RobotBaseUnitTests, getLink)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto link = dummy_quadruped->getLink("LF_ASSEMBLY");

    /// Ground truth
    robotlib::Link link_gt("LF_ASSEMBLY"); //TODO : generalize the test for all the links

    /// Assert conditions
    ASSERT_EQ(link->getName(), link_gt.getName());
    ASSERT_EQ(typeid(*link), typeid(link_gt));
}

TEST(RobotBaseUnitTests, getJoint)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    
    auto joint_dq = dummy_quadruped->getJoint("LF_HFE");

    /// Ground truth
    robotlib::Joint joint_gt("LF_HFE");

    /// Assert conditions
    ASSERT_EQ(joint_dq->getName(), joint_gt.getName());
    ASSERT_EQ(typeid(*joint_dq), typeid(joint_gt));
}

// TODO
//TEST(RobotBaseUnitTests, forwardKinematicsOne)
//{
//    /// Dummy quadruped
//   std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
//
//    /// Ground truth
//
//    /// Assert conditions
//    ASSERT_EQ(1, 1);
//    ASSERT_EQ(typeid(1), typeid(1));
//}

// TODO
//TEST(RobotBaseUnitTests, forwardKinematicsTwo)
//{
//    /// Dummy quadruped
//   std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
//
//    /// Ground truth
//
//    /// Assert conditions
//    ASSERT_EQ(1, 1);
//    ASSERT_EQ(typeid(1), typeid(1));
//}

// TODO
//TEST(RobotBaseUnitTests, inverseKinematicsOne)
//{
//    /// Dummy quadruped
//   std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
//
//    /// Ground truth
//
//    /// Assert conditions
//    ASSERT_EQ(1, 1);
//    ASSERT_EQ(typeid(1), typeid(1));
//}

// TODO
//TEST(RobotBaseUnitTests, inverseKinematicsTwo)
//{
//    /// Dummy quadruped
//   std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
//
//    /// Ground truth
//
//    /// Assert conditions
//    ASSERT_EQ(1, 1);
//    ASSERT_EQ(typeid(1), typeid(1));
//}

TEST(RobotBaseUnitTests, getName)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto name_dq = dummy_quadruped->getName();

    /// Ground truth
    std::string name_gt{"Quadruped"};

    /// Assert conditions
    ASSERT_EQ(name_dq, name_gt);
    ASSERT_EQ(typeid(name_dq), typeid(name_gt));
}

TEST(RobotBaseUnitTests, joint_parent_child)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    /// Groud truth
    std::map<std::string, std::pair<std::string, std::string>> jointMap_gt{
        //joint name, parent name, child name
        {"LF_HAA", std::make_pair("TRUNK", "LF_ASSEMBLY")},
        {"LF_HFE", std::make_pair("LF_ASSEMBLY", "LF_UPPERLEG")},
        {"LF_KFE", std::make_pair("LF_UPPERLEG", "LF_LOWERLEG")},
        {"RF_HAA", std::make_pair("TRUNK", "RF_ASSEMBLY")},
        {"RF_HFE", std::make_pair("RF_ASSEMBLY", "RF_UPPERLEG")},
        {"RF_KFE", std::make_pair("RF_UPPERLEG", "RF_LOWERLEG")},
        {"LH_HAA", std::make_pair("TRUNK", "LH_ASSEMBLY")},
        {"LH_HFE", std::make_pair("LH_ASSEMBLY", "LH_UPPERLEG")},
        {"LH_KFE", std::make_pair("LH_UPPERLEG", "LH_LOWERLEG")},
        {"RH_HAA", std::make_pair("TRUNK", "RH_ASSEMBLY")},
        {"RH_HFE", std::make_pair("RH_ASSEMBLY", "RH_UPPERLEG")},
        {"RH_KFE", std::make_pair("RH_UPPERLEG", "RH_LOWERLEG")},
    };

    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *(leg->getJoints()))
        {
            //std::cout << joint->getName() << " " << joint->getParent()->getName() << " " << joint->getChild()->getName() << std::endl;
            ASSERT_EQ(joint->getParent()->getName(), jointMap_gt[joint->getName()].first);
            ASSERT_EQ(joint->getChild()->getName(), jointMap_gt[joint->getName()].second);
        }
    }
}

TEST(RobotBaseUnitTests, link_parent_child)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    /// Groud truth
    std::map<std::string, std::pair<std::string, std::string>> linkMap_gt{
        //link name, parent name, child name
        {"LF_ASSEMBLY", std::make_pair("LF_HAA", "LF_HFE")},
        {"LF_UPPERLEG", std::make_pair("LF_HFE", "LF_KFE")},
        {"LF_LOWERLEG", std::make_pair("LF_KFE", "")},
        {"RF_ASSEMBLY", std::make_pair("RF_HAA", "RF_HFE")},
        {"RF_UPPERLEG", std::make_pair("RF_HFE", "RF_KFE")},
        {"RF_LOWERLEG", std::make_pair("RF_KFE", "")},
        {"LH_ASSEMBLY", std::make_pair("LH_HAA", "LH_HFE")},
        {"LH_UPPERLEG", std::make_pair("LH_HFE", "LH_KFE")},
        {"LH_LOWERLEG", std::make_pair("LH_KFE", "")},
        {"RH_ASSEMBLY", std::make_pair("RH_HAA", "RH_HFE")},
        {"RH_UPPERLEG", std::make_pair("RH_HFE", "RH_KFE")},
        {"RH_LOWERLEG", std::make_pair("RH_KFE", "")}};

    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto link : *(leg->getLinks()))
        {

            // std::cout << link->getName() << " " << link->getParent()->getName() << " ";
            // if (link->getChild() != nullptr)
            // {
            //     std::cout << link->getChild()->getName() << std::endl;
            // }
            // else
            //     std::cout << std::endl;

            ASSERT_EQ(link->getParent()->getName(), linkMap_gt[link->getName()].first);
            if (link->getChild() != nullptr)
            {
                ASSERT_EQ(link->getChild()->getName(), linkMap_gt[link->getName()].second);
            }
        }
    }
}

TEST(RobotBaseUnitTests, limb_getEndEffector)
{
    std::cout << "TODO: getEndEffector TEST" << std::endl;

    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    /// Groud truth
    //...

    auto footPos = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>();

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        leg->getEndEffector();
        //std::cout << leg->getEndEffector()->getName() << std::endl;
    }
}

TEST(RobotBaseUnitTests, LegDataMapCopyOperators)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto footPos = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>();
    auto q = dummy_quadruped->makeJointState();

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        footPos[leg] = dummy_quadruped->getFootPosition(q, leg);
        Eigen::Vector3d pos(10, 20, 30);
        footPos[leg] = pos;
    }

    std::cout << "Foot Position" << std::endl;

    for (auto fp : footPos)
    {
        std::cout << fp.key_->getName() << " " << fp.data_.transpose() << std::endl;
    }

    /// LegDataMap Copy

    auto footPosCopy = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>();

    for (auto legCopy : *(dummy_quadruped->getLegs()))
    {
        Eigen::Vector3d posCopy(5, 5, 5);
        footPosCopy[legCopy] = posCopy;
    }

    footPosCopy = footPos;

    std::cout << "Foot Position Copy" << std::endl;

    for (auto fpCopy : footPosCopy)
    {
        std::cout << fpCopy.key_->getName() << " " << fpCopy.data_.transpose() << std::endl;
    }

    /// LegDataMap Copy AssignAll

    auto footPosAssign = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>();
    auto qAssign = dummy_quadruped->makeJointState();

    footPosAssign = Eigen::Vector3d().setOnes();

    std::cout << "Foot Position Copy AssignAll" << std::endl;

    for (auto fpAssign : footPosAssign)
    {
        std::cout << fpAssign.key_->getName() << " " << fpAssign.data_.transpose() << std::endl;
    }

    // TODO: Const
}

TEST(RobotBaseUnitTests, jointStateOperators)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    auto q = dummy_quadruped->makeJointState();
    double value_gt {10};
    
    /// Operator[]
    for (auto leg : *(dummy_quadruped->getLegs()))
    { /// TODO: implement and use q.getSize()
        for (auto joint : *(leg->getJoints()))
        {
            q[joint] = value_gt;
        }
    }
    std::cout << "Testing [] operator...\n";

    std::cout << "JointState q elements:" << std::endl;
    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *leg->getJoints())
        {
            EXPECT_EQ(value_gt, q[joint]);
        }
    }

    /// Operator= Copy
    auto q2 = dummy_quadruped->makeJointState();
    q2 = q;
    
    std::cout << "Testing = operator using another joint state...\n";
    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *leg->getJoints())
        {
            EXPECT_EQ(q[joint], q2[joint]);   
        }
    }

    // Dummy check: Checking that q and q2 does not point to the same memory
    std::cout << "Testing = operator does not allow variables to point to same memory\n";
    double value {5};
    auto joint{dummy_quadruped->getJoint("LF_HAA")};
    q[joint] = value;
    EXPECT_NE(value, q2[joint]);

    /// Operator= AssignAll
    auto q3 = dummy_quadruped->makeJointState();
    q3 = value_gt;

    std::cout << "Testing = operator using a value...\n";
    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *leg->getJoints())
        {
            EXPECT_EQ(value_gt, q3[joint]);
        }
    }
}

TEST(RobotBaseUnitTests, footJacobian)
{
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    auto feetJac = dummy_quadruped->makeFeetJacobian();

    /// TODO: operator= to be overloaded
    Eigen::Matrix<double, 3, 6> footJacValues;
    footJacValues << 1, 2, 3,
        4, 5, 6,
        7, 8, 9,
        10, 11, 12,
        13, 14, 15,
        16, 16, 18;

    /// Access jacobian value associated to a leg
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << "FOOT JAC LEG: " << std::endl;
        feetJac[leg] << 10, 10, 10,
            20, 20, 20,
            30, 30, 30,
            40, 40, 40,
            50, 50, 50,
            60, 60, 60;
        std::cout << feetJac[leg] << std::endl;
    }

    feetJac["LF"] << 5, 5, 5,
        5, 5, 5,
        5, 5, 5,
        6, 6, 6,
        6, 6, 6,
        6, 6, 6;

    /// Access jacobian name and value
    for (auto &fj : feetJac)
    {
        std::cout << "FOOT JAC LEG: " << std::endl;
        std::cout << fj.key_->getName() << std::endl;
        std::cout << fj.data_ << std::endl;
    }
}

TEST(RobotBaseUnitTests, jointStateSetZero)
{
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    auto joint_state = dummy_quadruped->makeJointState();

    /// Operator[]
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        for (auto joint : *(leg->getJoints()))
        {
            joint_state[joint] = 10;
        }
    }

    std::cout << "JointState elements before:" << std::endl;

    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *leg->getJoints())
        {
            std::cout << joint_state[joint] << std::endl;
        }
    }

    joint_state.setZero();

    std::cout << "JointState elements after:" << std::endl;

    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *leg->getJoints())
        {
            std::cout << joint_state[joint] << std::endl;
        }
    }
}

// TEST(RobotBaseUnitTests, LegDataMap)
// {
//     std::cout << "TODO: LegDataMap TEST" << std::endl;

//     /// Dummy quadruped
//     std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

//     /// Groud truth
//     //...

//     auto footPos = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>();
//     auto footJac = dummy_quadruped->makeFeetJacobian();
//     auto q = dummy_quadruped->makeJointState();

//     //    for(auto leg:robot_model_->get() )
//     //    {
//     //        //compute foot position in base space via fwd kinematics
//     //        footPos[leg]= leg.getFootPosition(q);
//     //        JFoot_[LegID(leg)] = leg.getFootJacobian(q);
//     //    }

//     for (auto leg : *(dummy_quadruped->getLegs()))
//     {
//         //compute foot position in base space via fwd kinematics
//         dummy_quadruped->getFootPosition(q, leg, footPos[leg]);
//         dummy_quadruped->getFootJacobian(q, leg, footJac[leg]);
//     }

//     std::cout << "Foot Position" << std::endl;

//     for (auto pos : footPos)
//     {
//         std::cout << pos.data_.transpose() << std::endl;
//     }

//     std::cout << "Foot Jacobian" << std::endl;

//     //TODO: overload = operator in Jacobian class
//     for (auto &jac : footJac)
//     {
//         std::cout << jac.key_->getName() << std::endl;
//         std::cout << jac.data_ << std::endl;
//     }
// }

TEST(RobotBaseUnitTests, LinkDataMap)
{
    std::cout << "TODO: LinkDataMap TEST" << std::endl;

    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    /// Groud truth
    //...

    auto linkPos = dummy_quadruped->makeLinkDataMap<Eigen::Vector3d>();

    Eigen::Vector3d p;
    p.setZero();
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        for (auto link : *(leg->getLinks()))
        {
            linkPos[link] = p;
            p[0]++;
            p[1]++;
            p[2]++;
        }
    }
    for (auto pos : linkPos)
    {
        std::cout << pos.data_.transpose() << std::endl;
    }
}

TEST(RobotBaseUnitTests, JointDataMap)
{
    std::cout << "TODO: JointDataMap TEST" << std::endl;

    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    /// Groud truth
    //...

    auto jointPos = dummy_quadruped->makeJointDataMap<Eigen::Vector3d>();

    Eigen::Vector3d p;
    p.setZero();
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        for (auto joint : *(leg->getJoints()))
        {
            jointPos[joint] = p;
            p[0]++;
            p[1]++;
            p[2]++;
        }
    }
    for (auto pos : jointPos)
    {
        std::cout << pos.data_.transpose() << std::endl;
    }
}

// TEST(RobotBaseUnitTests, getNextLeg)     ///TODO
// {
//     std::cout << "TODO: getNextLeg TEST" << std::endl;

//     /// Dummy quadruped
//     std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

//     /// Groud truth
//     //...

//     Eigen::Vector3d p;
//     p.setZero();
//     for (auto leg : *(dummy_quadruped->getLegs()))
//     {
//         std::cout << "Leg: " << leg->getName() << ", Next leg: " << robot->getNextLeg(leg)->getName() << std::endl;
//     }
// }

TEST(RobotBaseUnitTests, jacobian_operator_equal)
{
    // std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto feetJac = dummy_quadruped->makeFeetJacobian();

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << "FOOT JAC LEG: " << std::endl;
        feetJac[leg] << 10, 10, 10,
            20, 20, 20,
            30, 30, 30,
            40, 40, 40,
            50, 50, 50,
            60, 60, 60;
        std::cout << feetJac[leg] << std::endl;
    }

    feetJac["LF"] << 5, 5, 5,
        5, 5, 5,
        5, 5, 5,
        6, 6, 6,
        6, 6, 6,
        6, 6, 6;

    feetJac["RH"] = feetJac["LF"];

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << leg->getName() << std::endl;
        std::cout << feetJac[leg] << std::endl;
    }

    auto feetJac2 = dummy_quadruped->makeFeetJacobian();

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << "FOOT JAC LEG: " << std::endl;
        feetJac2[leg] << 1, 1, 1,
            2, 2, 2,
            3, 3, 3,
            4, 4, 4,
            5, 5, 5,
            6, 6, 6;
        std::cout << feetJac2[leg] << std::endl;
    }

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << leg->getName() << std::endl;
        std::cout << feetJac2[leg] << std::endl;
    }

    feetJac = feetJac2;

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << leg->getName() << std::endl;
        std::cout << feetJac[leg] << std::endl;
    }
}

TEST(RobotBaseUnitTests, getRobotMass)
{
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    std::cout << dummy_quadruped->getRobotMass() << std::endl;
}

// TEST(RobotBaseUnitTests, inverseDynamics)
// {
//     /// Dummy quadruped
//     std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

//     Eigen::Matrix<double, 6, 1> v;
//     Eigen::Matrix<double, 6, 1> a;
//     Eigen::Matrix<double, 6, 1> g;
//     Eigen::Matrix<double, 6, 1> wrench_base; ///output

//     auto q = dummy_quadruped->makeJointState();
//     auto dq = dummy_quadruped->makeJointState();
//     auto ddq = dummy_quadruped->makeJointState();
//     auto tau = dummy_quadruped->makeJointState();

//     dummy_quadruped->inverseDynamics(v, a, g, q, dq, ddq, wrench_base, tau);
// }

TEST(RobotBaseUnitTests, dataMap_constructor_with_initialization)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    double data_double{1};
    double data{1};
    typedef double type;
    // typedef Eigen::Vector3d type;
    // Eigen::Vector3d data;
    // data.setOnes();

    auto leg_dm = dummy_quadruped->makeLegDataMap<type>(data);
    auto link_dm = dummy_quadruped->makeLinkDataMap<type>(data);
    auto joint_dm = dummy_quadruped->makeJointDataMap<type>(data);
    auto jacobian_dm = dummy_quadruped->makeFeetJacobian(data_double);

    std::cout << "leg_dm\n";
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << leg_dm[leg] << " ";
    }
    std::cout << "\n";

    std::cout << "link_dm\n";
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        for (auto link : *(leg->getLinks()))
            std::cout << link_dm[link] << " ";
        std::cout << "\n***\n";
    }
    std::cout << "\n";

    std::cout << "joint_dm\n";
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        for (auto joint : *(leg->getJoints()))
            std::cout << joint_dm[joint] << " ";
        std::cout << "\n***\n";
    }
    std::cout << "\n";

    std::cout << "jacobian_dm\n";
    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        std::cout << jacobian_dm[leg] << "\n";
        std::cout << "***\n";
    }
    std::cout << "\n";
}

TEST(RobotBaseUnitTests, JointDataMap_leg)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    for (auto leg : *dummy_quadruped->getLegs())
    {
        std::vector<std::string> joint_names;
        for (auto joint : *leg->getJoints())
        {
            joint_names.push_back(joint->getName());
        }

        auto joint_dm_per_leg = dummy_quadruped->makeJointDataMapPerLeg<double>(leg);

        int it {0};
        for (auto pair : joint_dm_per_leg)
        {
            EXPECT_EQ(joint_names[it], pair.key_->getName());
            it++;
        }
        using Type = double;
        Type value {1};
        // using Type = Eigen::Vector3d;
        // Type value{0, 0, 0};

        auto joint_dm_per_leg_default_data = dummy_quadruped->makeJointDataMapPerLeg<Type>(leg, value);

        for (auto pair : joint_dm_per_leg_default_data)
        {
            EXPECT_EQ(pair.data_, value);
        }
    }
}

TEST(RobotBaseUnitTests, getLegJointState)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto joint_state = dummy_quadruped->makeJointState();

    for (auto leg : *dummy_quadruped->getLegs())
    {
        std::vector<std::string> joint_names_gt;
        for (auto joint : *leg->getJoints())
        {
            joint_names_gt.push_back(joint->getName());
        }

        auto joint_state_per_leg = joint_state.getLegJointState(leg); //even if the reference is returned, the joint_data_map pair has to be created in anycase, so it would be a non realtime part!0

        int it {0};
        for (auto pair : *joint_state_per_leg)
        {
            EXPECT_EQ(joint_names_gt[it], pair.key_->getName());
            it++;
        }
    }
}

TEST(RobotBaseUnitTests, JointState_getByJointName)
{
    std::cout << "TODO\n";
    // /// Dummy quadruped
    // std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    // std::vector<std::pair<std::string,double>> values_gt;
    // auto joint_state = dummy_quadruped->makeJointState();
    // double v=0;

    // for (auto leg : *dummy_quadruped->getLegs())
    // {
    //     for (auto joint : *leg->getJoints())
    //     {
    //         values_gt.push_back(std::make_pair<std::string,double>(joint->getName()),v);
    //         joint_state[joint->getName()] = v;
    //         v++;
    //     }
    // }

    // int count = 0;
    // for (auto leg : *dummy_quadruped->getLegs())
    // {
    //     for (auto joint : *leg->getJoints())
    //     {
    //         EXPECT(values_gt[count].second,joint_state[joint->getName()])
    //         count++;
    //     }
    // }
}


TEST(RobotBaseUnitTests, getJointLimits)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto q_min = dummy_quadruped->makeJointState();
    auto q_max = dummy_quadruped->makeJointState();
    auto qd_max = dummy_quadruped->makeJointState();
    auto tau_max = dummy_quadruped->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_min_gt {0};
    const double q_max_gt {90};
    const double qd_max_gt {3};
    const double tau_max_gt {5};

    dummy_quadruped->getMinJointAngle(q_min);
    dummy_quadruped->getMaxJointAngle(q_max);
    dummy_quadruped->getMaxJointVelocity(qd_max);
    dummy_quadruped->getMaxJointEffort(tau_max);

    // TODO: override operator == for dataMap
    for (auto leg : *dummy_quadruped->getLegs())
    {
        for (auto joint : *leg->getJoints())
        {
            EXPECT_EQ(q_min_gt, q_min[joint]);
            EXPECT_EQ(q_max_gt, q_max[joint]);
            EXPECT_EQ(qd_max_gt, qd_max[joint]);
            EXPECT_EQ(tau_max_gt, tau_max[joint]);
        }
    }
}

TEST(RobotBaseUnitTests, minJointValue)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto q = dummy_quadruped->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_min_gt {-10};
    q[dummy_quadruped->getJoint("LF_HAA")] = q_min_gt;
    q[dummy_quadruped->getJoint("LF_HFE")] = 10;
    q[dummy_quadruped->getJoint("LH_HAA")] = 5;
    
    EXPECT_EQ(q.min(), q_min_gt);
}


TEST(RobotBaseUnitTests, maxJointValue)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};

    auto q = dummy_quadruped->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_max_gt = 10;
    q[dummy_quadruped->getJoint("LF_HAA")] = q_max_gt;
    q[dummy_quadruped->getJoint("LF_HFE")] = -10;
    q[dummy_quadruped->getJoint("LH_HAA")] = 5;
    
    EXPECT_EQ(q.max(), q_max_gt);
}