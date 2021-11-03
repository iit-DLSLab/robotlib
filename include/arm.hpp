/**
 * @file arm.hpp
 *
 * @brief Arm class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_ARM_HPP_
#define _ROBOTLIB_ARM_HPP_

#include "limb.hpp"

namespace dls
{
    namespace robotlib
    {
        /**
		* @brief Arm class
		* @details
		* This class represents a generic robot arm and extends the class Limb
		*/
        template <unsigned int NJOINTS, unsigned int NLINKS>
        class Arm : public Limb<NJOINTS, NLINKS>
        {
        public:
            Arm(const std::string &name,
                const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
                const std::array<std::shared_ptr<Link>, NLINKS> &links);

            virtual ~Arm();
        };
    } // namespace robotlib
} // namespace dls

#include "arm.tpp"

#endif // _ROBOTLIB_ARM_HPP_
