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
        /// Uncomment if you want to test the print method
        //std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
        //auto leg_data_map = dummy_quadruped->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        //
        //leg_data_map.print();
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
        /// Uncomment if you want to test the print method
        //std::shared_ptr<robotlib::RobotBase> dummy_quadruped = createRobot_t();
        //auto joint_state = dummy_quadruped->makeJointState();
        //
        //joint_state.print();
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

        for (auto leg : *(dummy_quadruped->getLegs()))
        {
            feet_jacobian[leg] <<
                11, 22, 33,
                44, 55, 66,
                77, 88, 99,
                111, 222, 333,
                444, 555, 666,
                777, 888, 999;
        }

        //std::cout << "JACOBIAN - LEG DATA MAP PRINT" << std::endl;
        //feet_jacobian.print();

        std::cout << "\nJACOBIAN - JACOBIAN STD::COUT" << std::endl;
        std::cout << feet_jacobian["LF"].block<3,3>(0,0) << std::endl;
        std::cout << feet_jacobian["LF"].block<3,3>(3,0) << std::endl;

        //for(auto leg: *dummy_quadruped->getLegs())
        //{
        //    std::cout << "Jacobian [Linear]" << std::endl;
        //    std::cout << "-----------------" << std::endl;
        //    std::cout << feet_jacobian[leg].block<3,3>(0,0) << std::endl;
        //    std::cout << "Jacobian [Angular]" << std::endl;
        //    std::cout << "-----------------" << std::endl;
        //    std::cout << feet_jacobian[leg].block<3,3>(3,0) << std::endl;
        //}

        std::cout << "\nJACOBIAN - JACOBIAN PRINT" << std::endl;
        feet_jacobian["LF"].print();
        //for(auto leg: *dummy_quadruped->getLegs())
        //{
        //    feet_jacobian[leg].print();
        //}
        std::cout << "" << std::endl;
     }
}

int main(int argc, char **argv)
{
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
}