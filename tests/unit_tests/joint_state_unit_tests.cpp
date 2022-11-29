/**
 * @file link_unit_tests.cpp
 * @brief Unit tests for Link class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "robot_factory.hpp"
#include "link.hpp"

/*!
 * @brief Unit tests for JointState class.
 * @details Set of unit tests for JointState::vec_ function.
 */
TEST(JointStateUnitTest, vec_)
{
    std::shared_ptr<robotlib::RobotBase> dummy_quadruped {robotlib::RobotFactory::openRobot("dummy-quadruped")};
    auto joint_state = dummy_quadruped->makeJointState(0.0);

     /*!
      * @test Dummy Quadruped - Type returned by vec_() is Eigen::VectorXd type
      */
     {
          Eigen::VectorXd joint_state_data {dummy_quadruped->getNJOINTS()};
          EXPECT_EQ(typeid(joint_state.vec_()), typeid(joint_state_data));

          for(auto leg :*dummy_quadruped->getLegs())
          {
               Eigen::VectorXd joint_state_leg_data {leg->getNJoints()};
               EXPECT_EQ(typeid(joint_state.vec_(leg)), typeid(joint_state_leg_data));
          }
     }

     /*!
      * @test Dummy Quadruped - Values inside variable returned by vec_() is correct
      */
     {
          double joint_value {1.0};
          joint_state = joint_value;
          auto joint_state_data {joint_state.vec_()};
          for(int i=0; i<joint_state.getSize(); i++)
          {
               EXPECT_EQ(joint_state_data(i), joint_value);
          }

          for(auto leg :*dummy_quadruped->getLegs())
          {
               joint_value++;
               joint_state[leg] = joint_value;
               auto joint_state_leg_data {joint_state.vec_(leg)};
               for(int i=0; i<leg->getNJoints(); i++)
               {
                    EXPECT_EQ(joint_state_leg_data(i), joint_value);
               }
          }
     }

     /*!
      * @test Dummy Quadruped - Moltiplication with a jacobian object
      */
     {
          double joint_value {2.0};
          double jacobian_value {3.0};
          joint_state = joint_value;
          for(auto leg :*dummy_quadruped->getLegs())
          {
               auto leg_joint_state {joint_state.vec_(leg)};
               auto foot_jacobian = dummy_quadruped->makeFootJacobian(leg, jacobian_value);
               auto res = foot_jacobian * leg_joint_state;
               auto res_linear = foot_jacobian.getLinearJacobian() * leg_joint_state;
               auto res_angular = foot_jacobian.getLinearJacobian() * leg_joint_state;
               for(int i=0; i<5; i++)
               {
                    double res_value {leg->getNJoints()*(joint_value*jacobian_value)};
                    for (int j=0; j<leg->getNJoints(); j++)
                    {
                         EXPECT_EQ(res(i,j), res_value);
                         if(i<3)
                         {
                              EXPECT_EQ(res_linear(i,j), res_value);
                              EXPECT_EQ(res_angular(i,j), res_value);
                         }
                    }
               }
          }
     }
}