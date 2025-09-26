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
 * @brief Set of unit tests for Limb::getName function (inherited from LimbBase)
 */
TEST(LimbUnitTests, getName)
{
	/**
	 * @test Get the limb (leg) name (case with a complete string)
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        std::vector<std::string> leg_names;
        for(auto& limb : limbs)
        {
			if(limb["type"][0]=="leg")
				leg_names.push_back(limb["name"][0]);
        }
		int i=0;
		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getName(), leg_names[i]);
			i++;
		}
	}

	/**
	 * @test Get the limb (arm) name (case with a complete string)
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
        std::vector<std::string> arm_names;
        for(auto& limb : limbs)
        {
			if(limb["type"][0]=="arm")
				arm_names.push_back(limb["name"][0]);
        }
		int i=0;
		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getName(), arm_names[i]);
			i++;
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getNJoints(), limbs[leg->id]["joints"].size());
		}
	}

	/**
 * @test Get the number of joints for a limb (arm)
 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& arm : dummy_robot->getArms())
		{
			EXPECT_EQ(arm->getNJoints(), limbs[arm->id]["joints"].size());
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
		for (auto& leg : dummy_robot->getLegs())
		{
			EXPECT_EQ(leg->getNLinks(), limbs[leg->id]["links"].size());
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& leg : dummy_robot->getLegs())
		{				
			std::vector<std::string> joints_name = limbs[leg->id]["joints"];
			// check  the links of the arm with a for loop
			for(const auto& name : joints_name)
			{
				EXPECT_EQ(leg->getJoint(name)->getName(), name);
			}
		}
	}

	/**
	 * @test Get the two limb (arm) joints and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& arm : dummy_robot->getArms())
		{
			std::vector<std::string> joints_name = limbs[arm->id]["joints"];
			// check  the links of the arm with a for loop
			for(const auto& name : joints_name)
			{
				EXPECT_EQ(arm->getJoint(name)->getName(), name);
			}
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& leg : dummy_robot->getLegs())
		{
			std::vector<std::string> links_name = limbs[leg->id]["links"];
			// check  the links of the arm with a for loop
			for(const auto& name : links_name)
			{
				EXPECT_EQ(leg->getLink(name)->getName(), name);
			}

		}
	}

	/**
	 * @test Get the two limb (leg) links and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& arm : dummy_robot->getArms())
		{
			std::vector<std::string> links_name = limbs[arm->id]["links"];
			// check  the links of the arm with a for loop
			for(const auto& name : links_name)
			{
				EXPECT_EQ(arm->getLink(name)->getName(), name);
			}
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& leg : dummy_robot->getLegs())
		{
			std::string link_name = limbs[leg->id]["links"].back();
			EXPECT_EQ(leg->getEndEffector()->getName(), link_name);
		}
	}

	/**
	 * @test Get the limb (arm) end-effector (last link) and check its name
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& arm : dummy_robot->getArms())
		{
			std::string link_name = limbs[arm->id]["links"].back();
			EXPECT_EQ(arm->getEndEffector()->getName(), link_name);
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& leg : dummy_robot->getLegs())
		{
			int i =0;
			for (auto& joint : leg->getJoints())
			{
				EXPECT_EQ(joint->getName(), limbs[leg->id]["joints"][i]);
				i++;
			}
		}
	}

	/**
	 * @test Iterate over the two limbs (arms) joints and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& arm : dummy_robot->getArms())
		{
			unsigned int i{0};
			for (auto& joint : arm->getJoints())
			{
				EXPECT_EQ(joint->getName(), limbs[arm->id]["joints"][i]);
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
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& leg : dummy_robot->getLegs())
		{
			int i=0;
			for (auto& link : leg->getLinks())
			{
				EXPECT_EQ(link->getName(), limbs[leg->id]["links"][i]);
				i++;
			}
		}
	}

	/**
	 * @test Iterate over the two limbs (arms) links and check their names
	 */
	{
		auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

		for (auto& arm : dummy_robot->getArms())
		{
			int i=0;
			for (auto& link : arm->getLinks())
			{
				EXPECT_EQ(link->getName(), limbs[arm->id]["links"][i]);
				i++;
			}
		}
	}
}