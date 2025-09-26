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
robotlib::DummyRobotCreator dummy_robot_creator;
std::vector<std::map<std::string,std::vector<std::string>>> limbs {
    {
        {"name", {"LF"}},
        {"joints", {"LF_HAA", "LF_HFE", "LF_KFE"}},
        {"links",  {"LF_ASSEMBLY", "LF_UPPERLEG", "LF_LOWERLEG"}},
        {"type", {"leg"}}
    },
    {
        {"name", {"RF"}},
        {"joints", {"RF_HAA", "RF_HFE", "RF_KFE"}},
        {"links",  {"RF_ASSEMBLY", "RF_UPPERLEG", "RF_LOWERLEG"}},
        {"type", {"leg"}}
    },
    {
        {"name", {"LH"}},
        {"joints", {"LH_HAA", "LH_HFE", "LH_KFE"}},
        {"links",  {"LH_ASSEMBLY", "LH_UPPERLEG", "LH_LOWERLEG"}},
        {"type", {"leg"}}
    },
    {
        {"name", {"RH"}},
        {"joints", {"RH_HAA", "RH_HFE", "RH_KFE"}},
        {"links",  {"RH_ASSEMBLY", "RH_UPPERLEG", "RH_LOWERLEG"}},
        {"type", {"leg"}}
    }
};
const std::string robot_name{"Dummy Quadruped"};

/**
 * @brief Unit tests for Jacobian class
 * @details Set of unit tests for Jacobian::getLinearJacobian function
 */

/*
 * @test Dummy Robot - Linear Jacobian matrix extracted with getLinearJacobian function
 */
TEST(JacobianUnitTests, makeFootJacobian)
{
	auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

	const robotlib::LimbPtr leg = dummy_robot->getLegs()[0];
	auto foot_jacobian = dummy_robot->makeFootJacobian(leg, 4);

	EXPECT_EQ(foot_jacobian.getLinearJacobian(), Eigen::MatrixXd::Ones(3,3)*4);
}

/*
 * @test Dummy Robot - Linear Jacobian matrix extracted with getLinearJacobian function
 */
TEST(JacobianUnitTests, makeFeetJacobian)
{
	auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
	auto feet_jacobian_map = dummy_robot->makeFeetJacobian(10);

	for(auto& map : feet_jacobian_map)
	{
		EXPECT_EQ(map.second.getLinearJacobian(), Eigen::MatrixXd::Ones(3,3)*10);
	}
}

/*!
* @test Dummy Robot - Multiplication with a jacobian object
*/
TEST(JacobianUnitTests, Multiplication)
{
	auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

	double joint_value {2.0};
	double jacobian_value {3.0};

	auto joint_state = dummy_robot->makeJointState(joint_value);
	auto feet_jacobian = dummy_robot->makeFeetJacobian(jacobian_value);

	for(auto& leg : dummy_robot->getLegs())
	{
		auto leg_joint_state  = dummy_robot->getLimbJointState(leg, joint_state);
		auto foot_jacobian = dummy_robot->makeFootJacobian(leg, jacobian_value);

		// auto temp = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(leg_joint_state.data(), leg_joint_state.size(), 1);

		Eigen::VectorXd res = foot_jacobian * leg_joint_state;
		Eigen::Vector3d res_linear = foot_jacobian.getLinearJacobian() * leg_joint_state;
		Eigen::Vector3d res_angular = foot_jacobian.getAngularJacobian() * leg_joint_state;

		double res_value {leg->getNJoints()*joint_value*jacobian_value};
		for(int i = 0; i < 5; i++)
		{
			EXPECT_EQ(res(i), res_value);
			if(i<3)
			{
					EXPECT_EQ(res_linear(i), res_value);
					EXPECT_EQ(res_angular(i), res_value);
			}
		}
	}
}


TEST(JacobianUnitTests, OperatorEqual)
{

	auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto feetJac = dummy_robot->makeFeetJacobian();
	
	Eigen::Matrix<double, 6, 3> footJacValues;
    footJacValues << 10, 10, 10,
					 20, 20, 20,
					 30, 30, 30,
					 40, 40, 40,
					 50, 50, 50,
					 60, 60, 60;

    for (auto& leg : dummy_robot->getLegs())
    {
        feetJac[leg] << 10, 10, 10,
						20, 20, 20,
						30, 30, 30,
						40, 40, 40,
						50, 50, 50,
						60, 60, 60;

		EXPECT_EQ(feetJac[leg], footJacValues);
    }

    feetJac[dummy_robot->getLimb("LF")] << 5, 5, 5,
					 5, 5, 5,
					 5, 5, 5,
					 6, 6, 6,
					 6, 6, 6,
					 6, 6, 6;

    feetJac[dummy_robot->getLimb("RH")] = feetJac[dummy_robot->getLimb("LF")];

    EXPECT_EQ(feetJac[dummy_robot->getLimb("RH")], feetJac[dummy_robot->getLimb("LF")]);
}