/**
 * @file data_map_unit_tests.cpp
 * 
 * @brief Unit tests for DataMap, LimbDataMap, LinkDataMap, JointDataMap, JointState classes
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

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::makeJointState function
 */
TEST(JointStateUnitTests, makeJointState)
{
    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

    /**
     * @test Dummy Robot - JointState initialized with all 0.0 values when called makeJointState
     */
    {
        auto joint_state = dummy_robot->makeJointState();
    
        for(auto& joint : dummy_robot->getJoints())
        {
            ASSERT_EQ(joint_state[joint], 0.0);
        }
    }

    /**
     * @test Dummy Robot - JointState initialized with a chosen value for each element using makeJointState
     */
    {
        auto joint_state = dummy_robot->makeJointState(2.0);

        for(auto& joint : dummy_robot->getJoints())
        {
            ASSERT_NE(joint_state[joint], 0.0);
            ASSERT_EQ(joint_state[joint], 2.0);
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
     * @test Dummy Robot - JointState values set with operator[]
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
        auto joint_state = dummy_robot->makeJointState();
         
        for(auto& joint: dummy_robot->getJoints())
        {
            ASSERT_EQ(joint_state[joint], 0.0);
            
            joint_state[joint] = 2.0;

            ASSERT_EQ(joint_state[joint], 2.0);
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
     * @test Dummy Robot - JointState values set to 0.0 with setZero function
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
        auto joint_state = dummy_robot->makeJointState(2.0);
        
        joint_state.setZero();

        for(auto& joint : dummy_robot->getJoints())
        {
            ASSERT_NE(joint_state[joint], 2.0);
            ASSERT_EQ(joint_state[joint], 0.0);
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
     * @test Dummy Robot - JointState with 12 (4 legs x 3 joints) elements
     */
    {
        auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
        auto joint_state = dummy_robot->makeJointState();

        ASSERT_EQ(joint_state.size(), 12);
    }
    /**
     * @test Dummy Robot - JointState empty
     */
    {}
    /**
     * @test Dummy Robot - JointState with one element
     */
    {}
    /**
     * @test Dummy Robot - JointState with 100 elements
     */
    {}
}

// /**
//  * @brief Unit tests for JointState class
//  * @details Set of unit tests for JointState::print function
//  */
// TEST(JointStateUnitTests, print)
// {
//      /**
//       * @test Dummy Robot - JointState values printed with print function
//       */
//      {
//         /// Uncomment if you want to test the print method
//         //std::shared_ptr<robotlib::RobotBase> dummy_robot = createRobot_t();
//         //auto joint_state = dummy_robot->makeJointState();
//         //
//         //joint_state.print();
//      }
// }

// /**
//  * @brief Unit tests for Jacobian class
//  * @details Set of unit tests for Jacobian::getLinearJacobian function
//  */
// TEST(JacobianUnitTests, print)
// {
//      /**
//       * @test Dummy Robot - Linear Jacobian matrix extracted with getLinearJacobian function
//       */
//      {
//         std::shared_ptr<robotlib::RobotBase> dummy_robot {robotlib::RobotFactory::openRobot("dummy-quadruped")};
//         auto feet_jacobian = dummy_robot->makeFeetJacobian();

//         // for(auto& leg: dummy_robot->getLegs())
//         // {
//         //     feet_jacobian[leg].print();
//         // }

//         // for (auto leg : *(dummy_robot->getLegs()))
//         // {
//         //     feet_jacobian[leg] <<
//         //         11, 22, 33,
//         //         44, 55, 66,
//         //         77, 88, 99,
//         //         111, 222, 333,
//         //         444, 555, 666,
//         //         777, 888, 999;
//         // }

//         // for(auto leg: *dummy_robot->getLegs())
//         // {
//         //     feet_jacobian[leg].print();
//         // }
//      }
// }