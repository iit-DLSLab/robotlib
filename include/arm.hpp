/*!
 * @file arm.hpp
 *
 * @brief Arm class definition and functions prototypes.
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_ARM_HPP_
#define _ROBOTLIB_ARM_HPP_

#include "limb.hpp"

namespace robotlib
{
    /*!
	 * @brief Arm class.
	 * @details
	 * This templated class represents a generic robot arm and extends the templated class Limb.
     * @tparam NJOINTS number of joints of the arm.
     * @tparam NLINKS number of links of the arm.
	 */
    template <unsigned int NJOINTS, unsigned int NLINKS>
    class Arm : public Limb<NJOINTS, NLINKS>
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] name name of the robot.
         * @param[in] joints array of shared pointers pointing to arm's joints.
         * @param[in] links array of shared pointers pointing to arm's links.
         */
        Arm(const std::string &name,
            const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
            const std::array<std::shared_ptr<Link>, NLINKS> &links);

        /*!
         * @brief Destructor.
         */
        virtual ~Arm();
    };
} // namespace robotlib

#include "arm.tpp"

#endif // _ROBOTLIB_ARM_HPP_