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
 * @brief Unit tests for LegDataMap class
 * @details Set of unit tests for LegDataMap::print function
 */
TEST(LegDataMapUnitTests, print)
{
     /**
      * @test Dummy Quadruped - LegDataMap values printed with print function
      */
     {
        std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
        auto leg_data_map = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        
        leg_data_map.print();
     }
}

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

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::size function
 */
TEST(JointStateUnitTests, size)
{
     /**
      * @test Dummy Quadruped - JointState with 12 (4 legs x 3 joints) elements
      */
     {
        std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
        auto joint_state = dummy_quadruped->makeJointState();

        ASSERT_EQ(joint_state.size(), 12);
     }
     /**
      * @test Dummy Quadruped - JointState empty
      */
     {}
     /**
      * @test Dummy Quadruped - JointState with one element
      */
    {}
     /**
      * @test Dummy Quadruped - JointState with 100 elements
      */
    {}
}

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::print function
 */
TEST(JointStateUnitTests, print)
{
     /**
      * @test Dummy Quadruped - JointState values printed with print function
      */
     {
        std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
        auto joint_state = dummy_quadruped->makeJointState();

        joint_state.print();
     }
}

/**
 * @brief Unit tests for Jacobian class
 * @details Set of unit tests for Jacobian::getLinearJacobian function
 */
TEST(JacobianUnitTests, print)
{
     /**
      * @test Dummy Quadruped - Linear Jacobian matrix extracted with getLinearJacobian function
      */
     {
        std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
        auto feet_jacobian = dummy_quadruped->makeFeetJacobian();

        std::cout << "PRINT FROM LEG DATA MAP" << std::endl;
        feet_jacobian.print();

        std::cout << "\nPRINT FROM JACOBIAN" << std::endl;
        for(auto leg: *dummy_quadruped->getLegs())
        {
            feet_jacobian[leg].print();
        }

        for (auto leg : *(dummy_quadruped->getLegs()))
        {
            feet_jacobian[leg] <<
                1, 2, 3,
                4, 5, 6,
                7, 8, 9,
                10, 20, 30,
                40, 50, 60,
                70, 80, 90;
        }

        std::cout << "PRINT FROM LEG DATA MAP" << std::endl;
        feet_jacobian.print();

        std::cout << "\nPRINT FROM JACOBIAN" << std::endl;
        for(auto leg: *dummy_quadruped->getLegs())
        {
            feet_jacobian[leg].print();
        }
     }
}

int main(int argc, char **argv)
{
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}