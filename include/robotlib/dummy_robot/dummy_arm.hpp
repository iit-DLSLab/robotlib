/*!
 * @file dummy_arm.hpp
 *
 * @brief Dummy arm class definition and functions prototypes. 
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

#ifndef _ROBOTLIB_DUMMY_ARM_HPP_
#define _ROBOTLIB_DUMMY_ARM_HPP_

#include "arm.hpp"

namespace robotlib
{
    /*!
    * @brief DummyArm class.
    * @details
    * This class allows to create a dummy arm having a specific morphology with the only purpose of testing Robotlib structures.
    */
    template <unsigned int NLINKS, unsigned int NJOINTS>
    class DummyArm : public Arm<NLINKS, NJOINTS>
    {
    public:
        /*!
        * @brief Constructor.
        * @param[in] name name of the arm.
        * @param[in] joints array of shared pointers pointing to arm's joints.
        * @param[in] links array of shared pointers pointing to arm's links.
        */
        DummyArm(const std::string& name,
                 const std::array<Link, NLINKS>& links,
                 const std::array<Joint, NJOINTS>& joints);

        /*!
        * @brief Destructor.
        */
        virtual ~DummyArm() = default;
    };
} // namespace robotlib

#include "dummy_arm.tpp"

#endif // _ROBOTLIB_DUMMY_ARM_HPP_