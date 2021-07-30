#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>

// TEST(RobotBaseUnitTests, getNLegs)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto n_legs_dq{dummy_quadruped->getNLEGS()};

//     /// Ground truth
//     int n_legs_gt{4};

//     /// Assert conditions
//     ASSERT_EQ(n_legs_dq, n_legs_gt);
//     ASSERT_EQ(typeid(n_legs_dq).name(), typeid(n_legs_gt).name());
// }

// TEST(RobotBaseUnitTests, getNJoints)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto n_joints_dq{dummy_quadruped->getNJOINTS()};

//     /// Ground truth
//     int n_joints_gt{12};

//     /// Assert conditions
//     ASSERT_EQ(n_joints_dq, n_joints_gt);
//     ASSERT_EQ(typeid(n_joints_dq).name(), typeid(n_joints_gt).name());
// }

// TEST(RobotBaseUnitTests, getNLinks)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto n_links_dq{dummy_quadruped->getNLINKS()};

//     /// Ground truth
//     int n_links_gt{12};

//     /// Assert conditions
//     ASSERT_EQ(n_links_dq, n_links_gt);
//     ASSERT_EQ(typeid(n_links_dq).name(), typeid(n_links_gt).name());
// }

// TEST(RobotBaseUnitTests, getLeg)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     /// Ground truth
//     std::array<std::string, 4> legs_gt{{"LF", "RF", "LH", "RH"}};

//     for (int i{0}; i < dummy_quadruped->getNLEGS(); i++)
//     {
//         auto legs_dq{dummy_quadruped->getLeg(i)};

//         /// Assert conditions
//         ASSERT_EQ(legs_dq->getName(), legs_gt.at(i));
//         ASSERT_EQ(typeid(legs_dq->getName()).name(), typeid(legs_gt.at(i)).name());
//     }
// }

// TEST(RobotBaseUnitTests, getFramePosition)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     Eigen::Vector3d frame_position = dummy_quadruped->getFramePosition(joint_state,
//                                                                        dummy_quadruped->getLink("trunk"),
//                                                                        dummy_quadruped->getLink("LF_lowerleg"));

//     /// Assert conditions
//     ASSERT_EQ(frame_position, Eigen::Vector3d().setZero());
// }

// TEST(RobotBaseUnitTests, getFrameOrientation)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     Eigen::Matrix3d frame_orientation = dummy_quadruped->getFrameOrientation(joint_state,
//                                                                              dummy_quadruped->getLink("trunk"),
//                                                                              dummy_quadruped->getLink("LF_lowerleg"));

//     /// Assert conditions
//     ASSERT_EQ(frame_orientation, Eigen::Matrix3d().setZero());
// }

// TEST(RobotBaseUnitTests, getFramePose)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     Eigen::Matrix4d frame_pose_dq = dummy_quadruped->getFramePose(joint_state,
//                                                                   dummy_quadruped->getLink("trunk"),
//                                                                   dummy_quadruped->getLink("LF_upperleg"));
//     /// Ground truth
//     Eigen::Matrix4d frame_pose_gt;
//     frame_pose_gt.setZero();
//     frame_pose_gt(3, 3) = 1;

//     /// Assert conditions
//     ASSERT_EQ(frame_pose_dq, frame_pose_gt);
//     ASSERT_EQ(typeid(frame_pose_dq).name(), typeid(frame_pose_gt).name());
// }

// TEST(RobotBaseUnitTests, getFootPosition)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     Eigen::Vector3d foot_position = dummy_quadruped->getFootPosition(joint_state,
//                                                                      dummy_quadruped->getLink("LF_lowerleg"));

//     /// Assert conditions
//     ASSERT_EQ(foot_position, Eigen::Vector3d().setZero());
// }

// TEST(RobotBaseUnitTests, getFootOrientation)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     Eigen::Matrix3d foot_orientation = dummy_quadruped->getFootOrientation(joint_state,
//                                                                            dummy_quadruped->getLink("LF_lowerleg"));

//     /// Assert conditions
//     ASSERT_EQ(foot_orientation, Eigen::Matrix3d().setZero());
// }

// TEST(RobotBaseUnitTests, getFootPose)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     Eigen::Matrix4d foot_pose_dq = dummy_quadruped->getFootPose(joint_state,
//                                                                 dummy_quadruped->getLink("LF_lowerleg"));

//     /// Ground truth
//     Eigen::Matrix4d foot_pose_gt;
//     foot_pose_gt.setZero();
//     foot_pose_gt(3, 3) = 1;

//     /// Assert conditions
//     ASSERT_EQ(foot_pose_dq, foot_pose_gt);
//     ASSERT_EQ(typeid(foot_pose_dq).name(), typeid(foot_pose_gt).name());
// }

// TEST(RobotBaseUnitTests, getLink)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto link_dq = dummy_quadruped->getLink("LF_assembly");

//     /// Ground truth
//     dls::robotlib::Link link_gt("LF_assembly"); //TODO : generalize the test for all the links

//     /// Assert conditions
//     ASSERT_EQ(link_dq->getName(), link_gt.getName());
//     ASSERT_EQ(typeid(*link_dq).name(), typeid(link_gt).name());
// }

// TEST(RobotBaseUnitTests, getJoint)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_dq = dummy_quadruped->getJoint("LF_hfe");

//     /// Ground truth
//     dls::robotlib::Joint joint_gt("LF_hfe");

//     /// Assert conditions
//     ASSERT_EQ(joint_dq->getName(), joint_gt.getName());
//     ASSERT_EQ(typeid(*joint_dq).name(), typeid(joint_gt).name());
// }

// TEST(RobotBaseUnitTests, getFeet)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto joint_state = dummy_quadruped->makeJointState();

//     auto feet_dq = dummy_quadruped->getFeet();

//     /// Ground truth
//     Eigen::Matrix4d foot_pose_gt{};
//     foot_pose_gt.setZero();
//     foot_pose_gt(3, 3) = 1;

//     for (const auto foot : feet_dq)
//     {
//         Eigen::Matrix4d foot_pose_dq{};
//         foot_pose_dq.setZero();
//         foot_pose_dq = dummy_quadruped->getFootPose(joint_state, foot);

//         /// Assert conditions
//         ASSERT_EQ(foot_pose_dq, foot_pose_gt);
//         ASSERT_EQ(typeid(foot_pose_dq).name(), typeid(foot_pose_gt).name());
//     }
// }

// // TODO
// //TEST(RobotBaseUnitTests, inverseKinematicsOne)
// //{
// //    /// Dummy quadruped
// //   std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();
// //
// //    /// Ground truth
// //
// //    /// Assert conditions
// //    ASSERT_EQ(1, 1);
// //    ASSERT_EQ(typeid(1).name(), typeid(1).name());
// //}

// // TODO
// //TEST(RobotBaseUnitTests, inverseKinematicsTwo)
// //{
// //    /// Dummy quadruped
// //   std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();
// //
// //    /// Ground truth
// //
// //    /// Assert conditions
// //    ASSERT_EQ(1, 1);
// //    ASSERT_EQ(typeid(1).name(), typeid(1).name());
// //}

// TEST(RobotBaseUnitTests, getName)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto name_dq = dummy_quadruped->getName();

//     /// Ground truth
//     std::string name_gt{"Quadruped"};

//     /// Assert conditions
//     ASSERT_EQ(name_dq, name_gt);
//     ASSERT_EQ(typeid(name_dq).name(), typeid(name_gt).name());
// }

// TEST(RobotBaseUnitTests, makeFootJacobian)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto foot = dummy_quadruped->getLink("LF_upperleg");

//     auto jacobian = dummy_quadruped->makeFootJacobian(foot);

//     // const int nJoints = foot.getParentLimb()->getNJoints();

//     // int count = 0;
//     // for (int i = 0; i < jacobian.rows(); ++i)
//     // {
//     //     for (int j = 0; j < jacobian.cols(); ++j)
//     //     {
//     //         jacobian(i, j) = count;
//     //         count++;
//     //     }
//     // }

//     // /// Ground truth
//     // double *data_gt = new double[6 * nJoints];
//     // for (int i = 0; i < 6 * nJoints; ++i)
//     // {
//     //     data_gt[i] = 0;
//     // }

//     // Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian_gt(data_gt, 6, nJoints);
//     // count = 0;
//     // for (int i = 0; i < jacobian_gt.rows(); ++i)
//     // {
//     //     for (int j = 0; j < jacobian_gt.cols(); ++j)
//     //     {
//     //         jacobian_gt(i, j) = count;
//     //         count++;
//     //     }
//     // }

//     // auto linearJacobian = jacobian.getLinearJacobian();
//     // auto angularJacobian = jacobian.getAngularJacobian();

//     // /// Assert conditions
//     // ASSERT_EQ(jacobian, jacobian_gt);
//     // ASSERT_EQ(jacobian.getLinearJacobian(), jacobian_gt.block(0, 0, 3, nJoints));
//     // ASSERT_EQ(jacobian.getAngularJacobian(), jacobian_gt.block(3, 0, 3, nJoints));
//     //ASSERT_EQ(typeid(jacobian).name(), typeid(jacobian_gt).name());
// }

// TEST(RobotBaseUnitTests, makeJacobian)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     auto foot = dummy_quadruped->getLink("LF_upperleg");

//     dummy_quadruped->makeJacobian(foot, foot);
// }

// TEST(RobotBaseUnitTests, joint_parent_child)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     /// Groud truth
//     std::map<std::string, std::pair<std::string, std::string>> jointMap_gt{
//         //joint name, parent name, child name
//         {"LF_haa", std::make_pair("trunk", "LF_assembly")},
//         {"LF_hfe", std::make_pair("LF_assembly", "LF_upperleg")},
//         {"LF_kfe", std::make_pair("LF_upperleg", "LF_lowerleg")},
//         {"RF_haa", std::make_pair("trunk", "RF_assembly")},
//         {"RF_hfe", std::make_pair("RF_assembly", "RF_upperleg")},
//         {"RF_kfe", std::make_pair("RF_upperleg", "RF_lowerleg")},
//         {"LH_haa", std::make_pair("trunk", "LH_assembly")},
//         {"LH_hfe", std::make_pair("LH_assembly", "LH_upperleg")},
//         {"LH_kfe", std::make_pair("LH_upperleg", "LH_lowerleg")},
//         {"RH_haa", std::make_pair("trunk", "RH_assembly")},
//         {"RH_hfe", std::make_pair("RH_assembly", "RH_upperleg")},
//         {"RH_kfe", std::make_pair("RH_upperleg", "RH_lowerleg")},
//     };

//     for (auto leg : *dummy_quadruped->getLegs())
//     {
//         for (auto joint : *(leg->getJoints()))
//         {
//             //std::cout << joint->getName() << " " << joint->getParent()->getName() << " " << joint->getChild()->getName() << std::endl;
//             ASSERT_EQ(joint->getParent()->getName(), jointMap_gt[joint->getName()].first);
//             ASSERT_EQ(joint->getChild()->getName(), jointMap_gt[joint->getName()].second);
//         }
//     }
// }

// TEST(RobotBaseUnitTests, link_parent_child)
// {
//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     /// Groud truth
//     std::map<std::string, std::pair<std::string, std::string>> linkMap_gt{
//         //link name, parent name, child name
//         {"LF_assembly", std::make_pair("LF_haa", "LF_hfe")},
//         {"LF_upperleg", std::make_pair("LF_hfe", "LF_kfe")},
//         {"LF_lowerleg", std::make_pair("LF_kfe", "")},
//         {"RF_assembly", std::make_pair("RF_haa", "RF_hfe")},
//         {"RF_upperleg", std::make_pair("RF_hfe", "RF_kfe")},
//         {"RF_lowerleg", std::make_pair("RF_kfe", "")},
//         {"LH_assembly", std::make_pair("LH_haa", "LH_hfe")},
//         {"LH_upperleg", std::make_pair("LH_hfe", "LH_kfe")},
//         {"LH_lowerleg", std::make_pair("LH_kfe", "")},
//         {"RH_assembly", std::make_pair("RH_haa", "RH_hfe")},
//         {"RH_upperleg", std::make_pair("RH_hfe", "RH_kfe")},
//         {"RH_lowerleg", std::make_pair("RH_kfe", "")}};

//     for (auto leg : *dummy_quadruped->getLegs())
//     {
//         for (auto link : *(leg->getLinks()))
//         {

//             // std::cout << link->getName() << " " << link->getParent()->getName() << " ";
//             // if (link->getChild() != nullptr)
//             // {
//             //     std::cout << link->getChild()->getName() << std::endl;
//             // }
//             // else
//             //     std::cout << std::endl;

//             ASSERT_EQ(link->getParent()->getName(), linkMap_gt[link->getName()].first);
//             if (link->getChild() != nullptr)
//             {
//                 ASSERT_EQ(link->getChild()->getName(), linkMap_gt[link->getName()].second);
//             }
//         }
//     }
// }
// TEST(RobotBaseUnitTests, limb_getEndEffector)
// {
//     std::cout << "TODO: getEndEffector TEST" << std::endl;

//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     /// Groud truth
//     //...

//     auto footPos = dummy_quadruped->makeLegDataMapPair<Eigen::Vector3d>();

//     for (auto leg : *(dummy_quadruped->getLegs()))
//     {
//         leg->getEndEffector();
//         //std::cout << leg->getEndEffector()->getName() << std::endl;
//     }
// }

TEST(RobotBaseUnitTests, legDataMap)
{
    std::cout << "TODO: LegDataMap TEST" << std::endl;

    /// Dummy quadruped
    std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

    /// Groud truth
    //...

    auto footPos = dummy_quadruped->makeLegDataMapPair<Eigen::Vector3d>();
    auto footJac = dummy_quadruped->makeFeetJacobian();
    auto q = dummy_quadruped->makeJointState();

    //    for(auto leg:robot_model_->get() )
    //    {
    //        //compute foot position in base space via fwd kinematics
    //        footPos[leg]= leg.getFootPosition(q);
    //        JFoot_[LegID(leg)] = leg.getFootJacobian(q);
    //    }

    for (auto leg : *(dummy_quadruped->getLegs()))
    {
        //compute foot position in base space via fwd kinematics
        dummy_quadruped->getFootPosition(q, leg, footPos[leg]);
        dummy_quadruped->getFootJacobian(q, leg, footJac[leg]);
    }

    for (auto pos : footPos)
    {
        std::cout << pos.second.transpose() << std::endl;
    }

    //TODO: overload = operator in Jacobian class
    for (auto &jac : footJac)
    {
        std::cout << jac.first->getName() << std::endl;
        std::cout << jac.second << std::endl;
    }
}

// TEST(RobotBaseUnitTests, linkDataMap)
// {
//     std::cout << "TODO: LinkDataMap TEST" << std::endl;

//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     /// Groud truth
//     //...

//     auto linkPos = dummy_quadruped->makeLinkDataMapPair<Eigen::Vector3d>();

//     Eigen::Vector3d p;
//     p.setZero();
//     for (auto leg : *(dummy_quadruped->getLegs()))
//     {
//         for (auto link : *(leg->getLinks()))
//         {
//             linkPos[link] = p;
//             p[0]++;
//             p[1]++;
//             p[2]++;
//         }
//     }
//     for (auto pos : linkPos)
//     {
//         std::cout << pos.second.transpose() << std::endl;
//     }
// }

// TEST(RobotBaseUnitTests, jointDataMap)
// {
//     std::cout << "TODO: JointDataMap TEST" << std::endl;

//     /// Dummy quadruped
//     std::shared_ptr<dls::robotlib::RobotBase> dummy_quadruped = createRobot_t();

//     /// Groud truth
//     //...

//     auto jointPos = dummy_quadruped->makeJointDataMapPair<Eigen::Vector3d>();

//     Eigen::Vector3d p;
//     p.setZero();
//     for (auto leg : *(dummy_quadruped->getLegs()))
//     {
//         for (auto joint : *(leg->getJoints()))
//         {
//             jointPos[joint] = p;
//             p[0]++;
//             p[1]++;
//             p[2]++;
//         }
//     }
//     for (auto pos : jointPos)
//     {
//         std::cout << pos.second.transpose() << std::endl;
//     }
// }