/**
 * @file jacobian_unit_tests.cpp
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

    auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);


/**
 * @brief Unit tests for Jacobian class
 * @details Set of unit tests for Jacobian::getLinearJacobian function
 */


/*
 * @test Dummy Robot - Linear Jacobian matrix extracted with getLinearJacobian function
 */
TEST(JacobianUnitTests, makeFeetJacobian)
{
	auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);
	auto feet_jacobian = dummy_robot->makeFeetJacobian(10);

	for(auto& value : feet_jacobian)
	{
		std::cout << "TEST " << value.getData().rows() << ", " << value.getData().cols() << std::endl;
		EXPECT_EQ(value.getData().getLinearJacobian(), Eigen::MatrixXd::Ones(3,3)*10);
	}
}


// /*!
// * @test Dummy Robot - Multiplication with a jacobian object
// */
// TEST(JacobianUnitTests, Multiplication)
// {
// 	auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

// 	double joint_value {2.0};
// 	double jacobian_value {3.0};

// 	auto joint_state = dummy_robot->makeJointState(joint_value);
// 	auto feet_jacobian = dummy_robot->makeFeetJacobian(jacobian_value);

// 	for(auto& leg : dummy_robot->getLegs())
// 	{
// 		auto leg_joint_state {joint_state[leg].tovec_()};
// 		auto foot_jacobian = dummy_robot->makeFootJacobian(*leg, jacobian_value);

// 		std::cout << "TEST " << foot_jacobian.rows() << ", " << foot_jacobian.cols() << std::endl;

// 		// Eigen::VectorXd res = foot_jacobian * Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(leg_joint_state.data(), leg_joint_state.size(), 1);
// 	// 	Eigen::Vector3d res_linear = foot_jacobian.getLinearJacobian() * leg_joint_state;
// 	// 	Eigen::Vector3d res_angular = foot_jacobian.getLinearJacobian() * leg_joint_state;

// 	// 	double res_value {leg->getNJoints()*(joint_value*jacobian_value)};
// 	// 	for(int i=0; i<5; i++)
// 	// 	{
// 	// 		EXPECT_EQ(res(i), res_value);
// 	// 		if(i<3)
// 	// 		{
// 	// 				EXPECT_EQ(res_linear(i), res_value);
// 	// 				EXPECT_EQ(res_angular(i), res_value);
// 	// 		}
// 	// 	}
// 	}
// }