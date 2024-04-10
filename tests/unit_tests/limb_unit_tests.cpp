/**
 * @file limb_unit_tests.cpp
 * 
 * @brief Unit tests for Limb class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "dummy_robot/dummy_robot_creator.hpp"

/**
 * @test Dummy robot created with the following structure:
 * 1 leg
 * 2 joints/links per leg
 * 1 arm
 * 2 joints/links per arm
 */
robotlib::DummyRobotCreator<1, 2, 1, 2> dummy_robot_creator;

//! Names of the components of the robot.
/* Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names] */
std::array<std::string, 11> components_names{"Dummy Robot",
                                            "Leg",
                                            "Leg_joint_1",
											"Leg_joint_2",
                                            "Leg_link_1",
											"Leg_link_2",
                                            "Arm",
                                            "Arm_joint_1",
											"Arm_joint_2",
                                            "Arm_link_1",
											"Arm_link_2"};

/**
 * @brief Set of unit tests for Limb::getName function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getName)
{
	/**
	 * @test Get the limb (leg) name (case with a complete string)
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getName(), components_names[1]);
		}
	}

	/**
	 * @test Get the limb (arm) name (case with a complete string)
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getName(), components_names[6]);
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getNJoints function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getNJoints)
{
	/**
 * @test Get the number of joints for a limb (leg)
 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getNJoints(), 2);
		}
	}

	/**
 * @test Get the number of joints for a limb (arm)
 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getNJoints(), 2);
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getNLinks function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getNLinks)
{
	/**
 * @test Get the number of links for a leg (case with two links)
 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getNLinks(), 2);
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getJoint function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getJoint)
{
	/**
	 * @test Get the two limb (leg) joints and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getJoint(components_names[2]).getName(), components_names[2]);
			EXPECT_EQ(leg->getJoint(components_names[3]).getName(), components_names[3]);
		}
	}

	/**
	 * @test Get the two limb (arm) joints and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getJoint(components_names[7]).getName(), components_names[7]);
			EXPECT_EQ(arm->getJoint(components_names[8]).getName(), components_names[8]);
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getLink function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getLink)
{
	/**
	 * @test Get the two limb (leg) links and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getLink(components_names[4]).getName(), components_names[4]);
			EXPECT_EQ(leg->getLink(components_names[5]).getName(), components_names[5]);
		}
	}

	/**
	 * @test Get the two limb (leg) links and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getLink(components_names[9]).getName(), components_names[9]);
			EXPECT_EQ(arm->getLink(components_names[10]).getName(), components_names[10]);
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getEndEffector function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getEndEffector)
{
	/**
	 * @test Get the limb (leg) end-effector (last link) and check its name
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getEndEffector().getName(), components_names[5]);
		}
	}

	/**
	 * @test Get the limb (arm) end-effector (last link) and check its name
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getEndEffector().getName(), components_names[10]);
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getJoints function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getJoints)
{
	/**
	 * @test Iterate over the two limbs (legs) joints and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		unsigned int i{0};
		for (auto& leg : dummy_robot->getLegs())
		{
			for (auto& joint : leg->getJoints())
			{
				EXPECT_EQ(joint.getName(), components_names[2+i]);
				i++;
			}
		}
	}

	/**
	 * @test Iterate over the two limbs (arms) joints and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		unsigned int i{0};
		for (auto& arm : dummy_robot->getArms())
		{
			for (auto& joint : arm->getJoints())
			{
				EXPECT_EQ(joint.getName(), components_names[7+i]);
				i++;
			}
		}
	}
}

/**
 * @brief Set of unit tests for Limb::getLinks function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getLinks)
{
	/**
	 * @test Iterate over the two limbs (legs) links and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		unsigned int i{0};
		for (auto& leg : dummy_robot->getLegs())
		{
			for (auto& link : leg->getLinks())
			{
				EXPECT_EQ(link.getName(), components_names[4+i]);
				i++;
			}
		}
	}

	/**
	 * @test Iterate over the two limbs (arms) links and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(components_names);

		unsigned int i{0};
		for (auto& arm : dummy_robot->getArms())
		{
			for (auto& link : arm->getLinks())
			{
				EXPECT_EQ(link.getName(), components_names[9+i]);
				i++;
			}
		}
	}
}