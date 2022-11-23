/*!
 * @file leg.hpp
 *
 * @brief Leg class definition and functions prototypes.
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_LEG_HPP_
#define _ROBOTLIB_LEG_HPP_

#include "limb.hpp"

namespace robotlib
{
    /*!
	 * @brief Leg class.
	 * @details
	 * This templated class represents a generic robot leg and extends the class Limb.
     * @tparam NJOINTS number of joints of the leg.
     * @tparam NLINKS number of links of the leg.
	 */
    template <unsigned int NJOINTS, unsigned int NLINKS>
    class Leg : public Limb<NJOINTS, NLINKS>
    {
    public:
    	/*!
         * @brief Constructor.
         * @param[in] name name of the leg.
         * @param[in] joints array of shared pointers pointing to leg's joints.
         * @param[in] links array of shared pointers pointing to leg's links.
         */
        Leg(const std::string &name,
            const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
            const std::array<std::shared_ptr<Link>, NLINKS> &links);

        /*!
         * @brief Destructor.
         */
        virtual ~Leg();
    };
} // namespace robotlib

#include "leg.tpp"

#endif // _ROBOTLIB_LEG_HPP_