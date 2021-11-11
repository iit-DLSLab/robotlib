/**
 * @file data_map_unit_tests.cpp
 * @brief Unit tests for DataMap, LegDataMap, LinkDataMap, JointDataMap, JointState classes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "../src/robots/dummy_quadruped.cpp" /// TODO: Remove cpp inclusion

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::makeJointState function
 */
TEST(JointStateUnitTests, makeJointState)
{
     std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
     
     /**
      * @test Dummy Quadruped - JointState initialized with all 0.0 values when called makeJointState
      */
     {
         auto joint_state = dummy_quadruped->makeJointState();
         
         for(auto leg: *dummy_quadruped->getLegs())
		 {
			for(auto joint : *leg->getJoints())
			{
                ASSERT_EQ(joint_state[joint], 0.0);
            }
         }
     }
     /**
      * @test Dummy Quadruped - JointState initialized with a chosen value for each element using makeJointState
      */
     {
         auto joint_state = dummy_quadruped->makeJointState(2.0);
         
         for(auto leg: *dummy_quadruped->getLegs())
		 {
			for(auto joint : *leg->getJoints())
			{
                ASSERT_NE(joint_state[joint], 0.0);
                ASSERT_EQ(joint_state[joint], 2.0);
            }
         }
     }
}

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::operator[] functions
 */
TEST(JointStateUnitTests, operatorSquareBracket)
{
     /**
      * @test Dummy Quadruped - JointState values set with operator[]
      */
     {
         std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
         auto joint_state = dummy_quadruped->makeJointState();
         
         for(auto leg: *dummy_quadruped->getLegs())
		 {
			for(auto joint : *leg->getJoints())
			{
                ASSERT_EQ(joint_state[joint], 0.0);
                
                joint_state[joint] = 2.0;

                ASSERT_EQ(joint_state[joint], 2.0);
            }
         }
     }
}

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::setZero function
 */
TEST(JointStateUnitTests, setZero)
{
     /**
      * @test Dummy Quadruped - JointState values set to 0.0 with setZero function
      */
     {
         std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
         auto joint_state = dummy_quadruped->makeJointState(2.0);
         
         joint_state.setZero();

         for(auto leg: *dummy_quadruped->getLegs())
		 {
			for(auto joint : *leg->getJoints())
			{
                ASSERT_NE(joint_state[joint], 2.0);
                ASSERT_EQ(joint_state[joint], 0.0);
            }
         }
     }
}

int main(int argc, char **argv)
{
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}