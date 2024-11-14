/**
 * @file robot_base_unit_tests.cpp
 * 
 * @brief Unit tests for RobotBase class
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "dummy_robot/dummy_robot_creator.hpp"


/**
 * @test Dummy robot created with the following structure:
 * 1 leg
 * 2 joints/links per leg
 */
robotlib::DummyRobotCreator<4, 3> dummy_robot_creator;

/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 29> components_names{"Dummy Robot",
                                            "LF", "RF", "LH", "RH",
                                            //joints
                                            "LF_HAA", "LF_HFE", "LF_KFE",
                                            "RF_HAA", "RF_HFE", "RF_KFE",
                                            "LH_HAA", "LH_HFE", "LH_KFE",
                                            "RH_HAA", "RH_HFE", "RH_KFE",
                                            //links
                                            "LF_ASSEMBLY", "LF_UPPERLEG", "LF_LOWERLEG",
                                            "RF_ASSEMBLY", "RF_UPPERLEG", "RF_LOWERLEG",
                                            "LH_ASSEMBLY", "LH_UPPERLEG", "LH_LOWERLEG",
                                            "RH_ASSEMBLY", "RH_UPPERLEG", "RH_LOWERLEG"};


TEST(RobotBaseUnitTests, getNLegs)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto n_legs_dq{dummy_robot->getNLEGS()};

    /// Ground truth
    unsigned int n_legs_gt{4};

    /// Assert conditions
    ASSERT_EQ(n_legs_dq, n_legs_gt);
    ASSERT_EQ(typeid(n_legs_dq), typeid(n_legs_gt));
}

TEST(RobotBaseUnitTests, getNJoints)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto n_joints_dq{dummy_robot->getNJOINTS()};

    /// Ground truth
    unsigned int n_joints_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_joints_dq, n_joints_gt);
    ASSERT_EQ(typeid(n_joints_dq), typeid(n_joints_gt));
}

TEST(RobotBaseUnitTests, getNLinks)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
    
    auto n_links_dq{dummy_robot->getNLINKS()};

    /// Ground truth
    unsigned int n_links_gt{12};

    /// Assert conditions
    ASSERT_EQ(n_links_dq, n_links_gt);
    ASSERT_EQ(typeid(n_links_dq), typeid(n_links_gt));
}

TEST(RobotBaseUnitTests, getLeg)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    /// Ground truth
    std::array<std::string, 4> legs_gt{{"LF", "RF", "LH", "RH"}};

    int count_legs {0};
    for (auto& legs_dq : dummy_robot->getLegs())
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
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Vector3d frame_position = dummy_robot->getFramePosition(joint_state,
                                                                   dummy_robot->getTrunk(),
                                                                   dummy_robot->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(frame_position, Eigen::Vector3d().Zero());
}

TEST(RobotBaseUnitTests, getFrameOrientation)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Matrix3d frame_orientation = dummy_robot->getFrameOrientation(joint_state,
                                                                         dummy_robot->getTrunk(),
                                                                         dummy_robot->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(frame_orientation, Eigen::Matrix3d().Zero());
}

TEST(RobotBaseUnitTests, getFramePose)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Matrix4d frame_pose_dq = dummy_robot->getFramePose(joint_state,
                                                              dummy_robot->getTrunk(),
                                                              dummy_robot->getLink("LF_UPPERLEG"));
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
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Vector3d foot_position = dummy_robot->getFootPosition(joint_state,
                                                                 dummy_robot->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(foot_position, Eigen::Vector3d().Zero());
}

TEST(RobotBaseUnitTests, getFootOrientation)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Matrix3d foot_orientation = dummy_robot->getFootOrientation(joint_state,
                                                                       dummy_robot->getLink("LF_LOWERLEG"));

    /// Assert conditions
    ASSERT_EQ(foot_orientation, Eigen::Matrix3d().Zero());
}

TEST(RobotBaseUnitTests, getFootPose)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto joint_state = dummy_robot->makeJointState();

    Eigen::Matrix4d foot_pose_dq = dummy_robot->getFootPose(joint_state,
                                                            dummy_robot->getLink("LF_LOWERLEG"));

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
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto link = dummy_robot->getLink("LF_ASSEMBLY");

    /// Ground truth
    robotlib::Link link_gt("LF_ASSEMBLY"); //TODO : generalize the test for all the links

    /// Assert conditions
    ASSERT_EQ(link.getName(), link_gt.getName());
    ASSERT_EQ(typeid(link), typeid(link_gt));
}

TEST(RobotBaseUnitTests, getJoint)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
    
    auto joint_dq = dummy_robot->getJoint("LF_HFE");

    /// Ground truth
    robotlib::Joint joint_gt("LF_HFE");

    /// Assert conditions
    ASSERT_EQ(joint_dq.getName(), joint_gt.getName());
    ASSERT_EQ(typeid(joint_dq), typeid(joint_gt));
}

// TODO
//TEST(RobotBaseUnitTests, forwardKinematicsOne)
//{
//    /// Dummy quadruped
//   auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
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
//   auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
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
//   auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
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
//   auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
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
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto name_dq = dummy_robot->getName();

    /// Ground truth
    std::string name_gt{"Dummy Robot"};

    /// Assert conditions
    ASSERT_EQ(name_dq, name_gt);
    ASSERT_EQ(typeid(name_dq), typeid(name_gt));
}

TEST(RobotBaseUnitTests, joint_parent_child)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

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

    for (auto& joint : dummy_robot->getJoints())
    {
        ASSERT_EQ(joint.getParent()->getName(), jointMap_gt[joint.getName()].first);
        ASSERT_EQ(joint.getChild()->getName(), jointMap_gt[joint.getName()].second);
    }
}

TEST(RobotBaseUnitTests, link_parent_child)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

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

    for (auto& link : dummy_robot->getLinks())
    {
        ASSERT_EQ(link.getParent()->getName(), linkMap_gt[link.getName()].first);
        if (link.getChildren().size() != 0)
        {
            ASSERT_EQ(link.getChildren()[0]->getName(), linkMap_gt[link.getName()].second);
        }
    }
}

TEST(RobotBaseUnitTests, limb_getEndEffector)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    for (auto& leg : dummy_robot->getLegs())
    {
        ASSERT_TRUE(leg->getEndEffector().getName().contains("LOWERLEG"));   
    }
}

// TODO
// TEST(RobotBaseUnitTests, LimbDataMapCopyOperators)
// {
//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//     auto footPos = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

//     auto q = dummy_robot->makeJointState();

//     for (auto& leg : dummy_robot->getLegs())
//     {
//         footPos[leg] = dummy_robot->getFootPosition(q, leg->getEndEffector());
//         Eigen::Vector3d pos(10, 20, 30);
//         footPos[leg] = pos;
//     }

//     std::cout << "Foot Position" << std::endl;

//     for (auto& fp : footPos)
//     {
//         std::cout << fp.getKey().getName() << " " << fp.getData().transpose() << std::endl;
//     }

//     /// LimbDataMap Copy

//     auto footPosCopy = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

//     for (auto& leg : dummy_robot->getLegs())
//     {
//         Eigen::Vector3d posCopy(5, 5, 5);
//         footPosCopy[leg] = posCopy;
//     }

//     footPosCopy = footPos;

//     std::cout << "Foot Position Copy" << std::endl;

//     for (auto fpCopy : footPosCopy)
//     {
//         std::cout << fpCopy.getKey().getName() << " " << fpCopy.getData().transpose() << std::endl;
//     }

//     /// LimbDataMap Copy AssignAll

//     auto footPosAssign = dummy_robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

//     auto qAssign = dummy_robot->makeJointState();

//     footPosAssign = Eigen::Vector3d().setOnes();

//     std::cout << "Foot Position Copy AssignAll" << std::endl;

//     for (auto fpAssign : footPosAssign)
//     {
//         std::cout << fpAssign.getKey().getName() << " " << fpAssign.getData().transpose() << std::endl;
//     }

//     // TODO: Const
// }

TEST(RobotBaseUnitTests, jointStateOperators)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
    auto q = dummy_robot->makeJointState();
    double value_gt {10};
    
    /// Operator[]
    for (auto& joint : dummy_robot->getJoints())
    {
        q[joint] = value_gt;
    }
    
    for (auto& joint : dummy_robot->getJoints())
    {
        EXPECT_EQ(value_gt, q[joint]);
    }

    /// Operator= Copy
    auto q2 = dummy_robot->makeJointState();
    q2 = q;
    
    for (auto& joint : dummy_robot->getJoints())
    {
        EXPECT_EQ(q[joint], q2[joint]);   
    }

    // Dummy check: Checking that q and q2 does not point to the same memory
    double value {5};
    auto& joint{dummy_robot->getJoint("LF_HAA")};
    q[joint] = value;
    EXPECT_NE(value, q2[joint]);

    /// Operator= AssignAll
    auto q3 = dummy_robot->makeJointState();
    q3 = value_gt;

    for (auto& joint : dummy_robot->getJoints())
    {
        EXPECT_EQ(value_gt, q3[joint]);
    }
}


// TODO
// TEST(RobotBaseUnitTests, jointStateSetZero)
// {
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
//     auto joint_state = dummy_robot->makeJointState();

//     /// Operator[]
//     for (auto& joint : dummy_robot->getJoints())
//     {
//         joint_state[joint] = 10;
//     }

//     std::cout << "JointState elements before:" << std::endl;

//     for (auto& joint : dummy_robot->getJoints())
//     {
//         std::cout << joint_state[joint] << std::endl;
//     }

//     joint_state.setZero();

//     std::cout << "JointState elements after:" << std::endl;

//     for (auto& joint : dummy_robot->getJoints())
//     {
//         std::cout << joint_state[joint] << std::endl;
//     }
// }

// TODO
// TEST(RobotBaseUnitTests, LimbDataMap)
// {
//     std::cout << "TODO: LimbDataMap TEST" << std::endl;

//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//     /// Groud truth
//     //...

//     auto footPos = dummy_robot->makeLimbDataMap<Eigen::Vector3d>();
//     auto footJac = dummy_robot->makeFeetJacobian();
//     auto q = dummy_robot->makeJointState();

//     //    for(auto leg:robot_model_->get() )
//     //    {
//     //        //compute foot position in base space via fwd kinematics
//     //        footPos[leg]= leg.getFootPosition(q);
//     //        JFoot_[LegID(leg)] = leg.getFootJacobian(q);
//     //    }

//     for (auto leg : *(dummy_robot->getLegs()))
//     {
//         //compute foot position in base space via fwd kinematics
//         dummy_robot->getFootPosition(q, leg, footPos[leg]);
//         dummy_robot->getFootJacobian(q, leg, footJac[leg]);
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

// TODO
// TEST(RobotBaseUnitTests, LinkDataMap)
// {
//     std::cout << "TODO: LinkDataMap TEST" << std::endl;

//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//     /// Groud truth
//     //...

//     auto linkPos = dummy_robot->makeLinkDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());

//     Eigen::Vector3d p;
//     p.setZero();
//     for (auto& link : dummy_robot->getLinks())
//     {
//         linkPos[link] = p;
//         p[0]++;
//         p[1]++;
//         p[2]++;
//     }

//     for (auto& pos : linkPos)
//     {
//         std::cout << pos.getData().transpose() << std::endl;
//     }
// }

// TODO
// TEST(RobotBaseUnitTests, getRobotMass)
// {
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
//     std::cout << dummy_robot->getRobotMass() << std::endl;
// }

// TODO
// TEST(RobotBaseUnitTests, inverseDynamics)
// {
//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//     Eigen::Matrix<double, 6, 1> v;
//     Eigen::Matrix<double, 6, 1> a;
//     Eigen::Matrix<double, 6, 1> g;
//     Eigen::Matrix<double, 6, 1> wrench_base; ///output

//     auto q = dummy_robot->makeJointState();
//     auto dq = dummy_robot->makeJointState();
//     auto ddq = dummy_robot->makeJointState();
//     auto tau = dummy_robot->makeJointState();

//     dummy_robot->inverseDynamics(v, a, g, q, dq, ddq, wrench_base, tau);
// }

// TODO
// TEST(RobotBaseUnitTests, dataMap_constructor_with_initialization)
// {
//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
//     double data_double{1};
//     double data{1};
//     typedef double type;
//     // typedef Eigen::Vector3d type;
//     // Eigen::Vector3d data;
//     // data.setOnes();

//     auto leg_dm = dummy_robot->makeLimbDataMap<type>(data);
//     auto link_dm = dummy_robot->makeLinkDataMap<type>(data);
//     auto joint_dm = dummy_robot->makeJointDataMap<type>(data);
//     auto jacobian_dm = dummy_robot->makeFeetJacobian(data_double);

//     std::cout << "leg_dm\n";
//     for (auto& leg : dummy_robot->getLegs())
//     {
//         std::cout << leg_dm[leg] << " ";
//     }
//     std::cout << "\n";

//     std::cout << "link_dm\n";
//     for (auto& leg : dummy_robot->getLegs())
//     {
//         for (auto link : leg->getLinks())
//             std::cout << link_dm[link] << " ";
//         std::cout << "\n***\n";
//     }
//     std::cout << "\n";

//     std::cout << "joint_dm\n";
//     for (auto& leg : dummy_robot->getLegs())
//     {
//         for (auto joint : leg->getJoints())
//             std::cout << joint_dm[joint] << " ";
//         std::cout << "\n***\n";
//     }
//     std::cout << "\n";

//     std::cout << "jacobian_dm\n";
//     for (auto& leg : dummy_robot->getLegs())
//     {
//         std::cout << jacobian_dm[leg] << "\n";
//         std::cout << "***\n";
//     }
//     std::cout << "\n";
// }

// TODO
// TEST(RobotBaseUnitTests, getLegJointState)
// {
//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//     auto joint_state = dummy_robot->makeJointState();

//     for (auto& leg : dummy_robot->getLegs())
//     {
//         std::vector<std::string> joint_names_gt;
//         for (auto& joint : leg->getJoints())
//         {
//             joint_names_gt.push_back(joint.getName());
//         }

//         auto joint_state_per_leg = joint_state[leg]; //even if the reference is returned, the joint_data_map pair has to be created in anycase, so it would be a non realtime part!0

//         int it{0};
//         for (auto& pair : joint_state_per_leg)
//         {
//             EXPECT_EQ(joint_names_gt[it], pair.getKey().getName());
//             it++;
//         }
//     }
// }

// TODO
// TEST(RobotBaseUnitTests, JointState_getByJointName)
// {
//     std::cout << "TODO\n";
//     /// Dummy quadruped
//     auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

//     std::vector<std::pair<std::string,double>> values_gt;
//     auto joint_state = dummy_robot->makeJointState();
//     double v=0;

//     for (auto leg : *dummy_robot->getLegs())
//     {
//         for (auto joint : leg->getJoints())
//         {
//             values_gt.push_back(std::make_pair<std::string,double>(joint->getName()),v);
//             joint_state[joint->getName()] = v;
//             v++;
//         }
//     }

//     int count = 0;
//     for (auto leg : *dummy_robot->getLegs())
//     {
//         for (auto joint : leg->getJoints())
//         {
//             EXPECT(values_gt[count].second,joint_state[joint->getName()])
//             count++;
//         }
//     }
// }


TEST(RobotBaseUnitTests, getJointLimits)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto q_min = dummy_robot->makeJointState();
    auto q_max = dummy_robot->makeJointState();
    auto qd_max = dummy_robot->makeJointState();
    auto tau_max = dummy_robot->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_min_gt {0};
    const double q_max_gt {90};
    const double qd_max_gt {3};
    const double tau_max_gt {5};

    dummy_robot->getMinJointAngle(q_min);
    dummy_robot->getMaxJointAngle(q_max);
    dummy_robot->getMaxJointVelocity(qd_max);
    dummy_robot->getMaxJointEffort(tau_max);

    // TODO: override operator == for dataMap
    for (auto& joint : dummy_robot->getJoints())
    {
        EXPECT_EQ(q_min_gt, q_min[joint]);
        EXPECT_EQ(q_max_gt, q_max[joint]);
        EXPECT_EQ(qd_max_gt, qd_max[joint]);
        EXPECT_EQ(tau_max_gt, tau_max[joint]);
    }
}


TEST(RobotBaseUnitTests, minJointValue)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto q = dummy_robot->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_min_gt {-10};
    q[dummy_robot->getJoint("LF_HAA")] = q_min_gt;
    q[dummy_robot->getJoint("LF_HFE")] = 10;
    q[dummy_robot->getJoint("LH_HAA")] = 5;
    
    EXPECT_EQ(q.min(), q_min_gt);
}


TEST(RobotBaseUnitTests, maxJointValue)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    auto q = dummy_robot->makeJointState();

    // Ground truth for the dummy quadruped
    const double q_max_gt = 10;
    q[dummy_robot->getJoint("LF_HAA")] = q_max_gt;
    q[dummy_robot->getJoint("LF_HFE")] = -10;
    q[dummy_robot->getJoint("LH_HAA")] = 5;
    
    EXPECT_EQ(q.max(), q_max_gt);
}

TEST(RobotBaseUnitTests, getFootJacobian)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
    auto feet_jacobian = dummy_robot->makeFeetJacobian();
    auto q = dummy_robot->makeJointState(0.0);
    for(auto& leg : dummy_robot->getLegs())
    {
        feet_jacobian[leg].setOnes();
        dummy_robot->getFootJacobian(q, *leg, feet_jacobian[leg]);
    }

    for (auto& leg : dummy_robot->getLegs())
    {
        dummy_robot->getFootJacobian(q, *leg, feet_jacobian[leg]);
        for(int i=0; i<6;i++)
        {
            for (int j=0; j<leg->getNJoints(); j++)
            {
                EXPECT_EQ(feet_jacobian[leg](i,j), 0.0);
            }
        }
    }
}
