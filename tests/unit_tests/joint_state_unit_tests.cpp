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

TEST(JointStateUnitTests, Zero)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

    for (auto& limb_pair : joint_state)
    {
        for (auto& joint_pair : limb_pair.getData())
          EXPECT_EQ(joint_pair.getData(),  0);
    }
}


TEST(JointStateUnitTests, NonZero)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    double p(3);
    auto joint_state = dummy_robot->makeJointState(p);

    for (auto& limb_pair : joint_state)
    {
        for (auto& joint_pair : limb_pair.getData())
        	EXPECT_EQ(joint_pair.getData(),  p);
    }
}

TEST(JointStateUnitTests, Attribution)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

    double p(0);
    for (auto& joint : dummy_robot->getJoints())
    {
        p++;
        joint_state[joint] = p;

        EXPECT_EQ(joint_state[joint],  p);
    }
}

TEST(JointStateUnitTests, Iteration)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

    // get joints name from definition
    std::vector<std::string> joints_name;
    for (auto limb : limbs)
    {
        for (auto joint_name : limb["joints"])
        {
            joints_name.push_back(joint_name);
        }
    }

    auto i{0};
    for (auto& limb_pair : joint_state)
    {
        for (auto& joint_pair : limb_pair.getData()){
       		EXPECT_EQ(joint_pair.getKey().getName(), joints_name[i]);
            i++;
        }
    }
}


TEST(JointStateUnitTests, Size)
{
    /// Dummy quadruped
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

    auto joint_count{0};
    for (auto& limb_pair : joint_state)
    {
        for (auto& joint_pair : limb_pair.getData())
        	joint_count++;
    }

    EXPECT_EQ(joint_count,  12);
    EXPECT_EQ(joint_state.size(),  12);
}

/*!
 * @brief Unit tests for JointState class.
 * @details Set of unit tests for JointState::toeig_ function.
 */
TEST(JointStateUnitTest, toeig_)
{
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

	/*!
     * @test Dummy Robot - Type returned by toeig_() is correct
     */
	{
		Eigen::VectorXd joint_state_data {dummy_robot->getNJOINTS()};
		
		EXPECT_EQ(typeid(joint_state.toeig_()), typeid(joint_state_data));
	}

	/*!
     * @test Dummy Robot - Size of toeig_() is correct
     */
	{
		EXPECT_EQ(joint_state.toeig_().size(), 12);
	}


    /*!
	 * @test Dummy Robot - Values inside variable returned by toeig_() are correct
	 */
	{
		double joint_value {1.0};
		joint_state = joint_value;

		auto joint_state_data(joint_state.toeig_());

		for(int i=0; i < joint_state.size(); i++)
		{
		    EXPECT_EQ(joint_state_data[i], joint_value);
		}

		auto joint_count{0};
		for(auto& leg : dummy_robot->getLegs())
		{
			joint_value++;
			joint_state[leg] = joint_value;

			joint_state_data.segment(joint_count*3, 3).setConstant(joint_value);
			joint_count++;

		    EXPECT_EQ(joint_state_data, joint_state.toeig_());
		}
	}
}


/*!
 * @brief Unit tests for JointState class.
 * @details Set of unit tests for JointState::tovec_ function.
 */
TEST(JointStateUnitTest, tovec_)
{
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

    auto joint_state = dummy_robot->makeJointState();

	/*!
     * @test Dummy Robot - Type returned by toeig_() is correct
     */
	{
		std::vector<double> joint_state_data(dummy_robot->getNJOINTS(), 0.0);
		
		EXPECT_EQ(typeid(joint_state.tovec_()), typeid(joint_state_data));
	}

	/*!
     * @test Dummy Robot - Size of toeig_() is correct
     */
	{
		EXPECT_EQ(joint_state.tovec_().size(), 12);
	}


    /*!
	 * @test Dummy Robot - Values inside variable returned by toeig_() are correct
	 */
	{
		double joint_value {1.0};
		joint_state = joint_value;

		auto joint_state_data(joint_state.tovec_());

		for(int i=0; i < joint_state.size(); i++)
		{
		    EXPECT_EQ(joint_state_data[i], joint_value);
		}

		auto joint_count{0};
		for(auto& leg : dummy_robot->getLegs())
		{
			joint_value++;
			joint_state[leg] = joint_value;

			for(int i = 0; i <= 2; i++)
				joint_state_data[joint_count*3 + i] = joint_value;

		    EXPECT_EQ(joint_state_data, joint_state.tovec_());
			joint_count++;
		}
	}
}

/**
 * @brief Unit tests for JointState class
 * @details Set of unit tests for JointState::makeJointState function
 */
TEST(JointStateUnitTests, makeJointState)
{
    auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);

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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
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
        auto dummy_robot = dummy_robot_creator.createDummyRobot(robot_name, limbs);
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