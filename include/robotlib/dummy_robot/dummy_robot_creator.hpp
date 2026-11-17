/*!
 * @file dummy_robot_creator.hpp
 *
 * @brief Dummy robot class definition and functions prototypes. Used to create generic types of robots for tests
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_DUMMY_ROBOT_CREATOR_HPP_
#define _ROBOTLIB_DUMMY_ROBOT_CREATOR_HPP_

#include "robot_base.hpp"

namespace robotlib
{
    /*!
     * @brief DummyRobotCreator class.
     * @details
     * This class allows to create dummy robots having different morphologies with the only purpose of testing Robotlib structures.
     */
    class DummyRobotCreator
    {
    public:
        /*!
         * @brief Constructor.
         */
        DummyRobotCreator() = default;

        /*!
         * @brief Destructor.
         */
        virtual ~DummyRobotCreator() = default;

        /*!
         * @brief Function to create a dummy robot.
         * @details
         * The names of the items defining the robot structure follow this order:
         * 
         * Component names = [Robot name | Trunk name |  Leg names | Leg joint names | Leg link names | Arms names | Arm joint names | Arm link names]
         * @param[in] components_names names of the items defining the robot.
         * @return shared pointer pointing to the RobotBase object.
         */
        std::shared_ptr<RobotBase> createDummyRobot(const std::string& name, const std::vector<std::map<std::string,std::vector<std::string>>>& limbs_string_type);
    };
} // namespace robotlib

#endif // _ROBOTLIB_DUMMY_ROBOT_CREATOR_HPP_