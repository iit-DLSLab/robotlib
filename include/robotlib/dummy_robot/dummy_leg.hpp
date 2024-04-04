/*!
 * @file dummy_robot.hpp
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

#ifndef _ROBOTLIB_DUMMY_LEG_HPP_
#define _ROBOTLIB_DUMMY_LEG_HPP_

#include "leg.hpp"

namespace robotlib
{
    /*!
    * @brief DummyLeg class.
    * @details
    * This class allows to create a dummy leg having a specific morphology with the only purpose of testing Robotlib structures.
    * @tparam NJOINTSLEG number of joints of the leg.
    * @tparam NLINKSLEG number of links of the leg.
    */
    template <unsigned int NLINKSJOINTS>
    class DummyLeg : public Leg<NLINKSJOINTS, NLINKSJOINTS>
    {
    public:
        /*!
        * @brief Constructor.
        * @param[in] leg_name name of the leg.
        * @param[in] trunk_name name of the robot trunk.
        * @param[in] leg_joints array of shared pointers pointing to leg's joints.
        * @param[in] leg_links array of shared pointers pointing to leg's links.
        */
        DummyLeg(const std::string& name,
                 const Container<Link, NLINKSJOINTS>& links,
                 const Container<Joint, NLINKSJOINTS>& joints);

        /*!
        * @brief Destructor.
        */
        virtual ~DummyLeg() = default;
    };

} // namespace robotlib

#include "dummy_leg.tpp"

#endif // _ROBOTLIB_DUMMY_LEG_HPP_