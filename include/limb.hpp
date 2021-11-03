/**
 * @file limb.hpp
 *
 * @brief Limb class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_LIMB_HPP_
#define _ROBOTLIB_LIMB_HPP_

#include "limb_base.hpp"
#include "joint.hpp"
#include "link.hpp"

namespace dls
{
    namespace robotlib
    {
        /**
		* @brief Limb class
		* @details
		* This class represents a generic robot limb (leg/arm) and extends the abstract class LimbBase
		*/
        template <unsigned int NJOINTS, unsigned int NLINKS>
        class Limb : public LimbBase
        {
        public:
            Limb(const std::string &name,
                 const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
                 const std::array<std::shared_ptr<Link>, NLINKS> &links);

            virtual ~Limb();

            /**
             * @brief Get the number of joints of the Limb
             * @return const int
             */
            virtual const int getNJoints() const override;
            /**
             * @brief Get the number of links of the Limb
             * @return const int
             */
            virtual const int getNLinks() const override;
            /**
             * @brief Get the Joint object associated to the name
             * @param name name of the Joint object that is returned
             * @return const std::shared_ptr<Joint> 
             */
            virtual const std::shared_ptr<Joint> getJoint(const std::string &name) const override;
            /**
             * @brief Get the Link object associated to the name
             * @param name name of the Link object that is returned
             * @return const std::shared_ptr<Link> 
             */
            virtual const std::shared_ptr<Link> getLink(const std::string &name) const override;
            /**
             * @brief Get the Link object correspondent to the Limb end-effector
             * @return const std::shared_ptr<Link> 
             */
            virtual const std::shared_ptr<Link> getEndEffector() const override;
            /**
             * @brief Get the joints of the Limb
             * @return const std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>>
             */
            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getJoints() const { return joints_; };
            /**
             * @brief Get the links of the Limb
             * @return const std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> 
             */
            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> getLinks() const { return links_; };

        protected:
            const std::shared_ptr<const Container<std::shared_ptr<Joint>, NJOINTS>> joints_;
            const std::shared_ptr<const Container<std::shared_ptr<Link>, NLINKS>> links_;
        };
    } // namespace robotlib
} // namespace dls

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_
