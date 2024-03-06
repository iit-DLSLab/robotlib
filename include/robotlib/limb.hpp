/*!
 * @file limb.hpp
 *
 * @brief Limb class definition and functions prototypes.
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

#ifndef _ROBOTLIB_LIMB_HPP_
#define _ROBOTLIB_LIMB_HPP_

#include "limb_base.hpp"
#include "utils/container.hpp"

namespace robotlib
{
   /*!
	 * @brief Limb class.
	 * @details
	 * This class represents a robot limb (leg/arm/other) with a specific number of joints and links.
    */
   template <unsigned int NLINKS, unsigned int NJOINTS>
   class Limb : public LimbBase
   {
      //! Robot is a friend class to let it use the private methods of the Joint class.
		template <unsigned int NLMBS, unsigned int NLNKS, unsigned int NJONTS>
		friend class Robot;

   public:
      /*!
      * @brief Constructor.
      * @param[in] name name of the limb.
      * @param[in] joints array of shared pointers pointing to limb's joints.
      * @param[in] links array of shared pointers pointing to limb's links.
      */
      Limb(const std::string& name,
           const std::array<std::shared_ptr<Link>, NLINKS>& links,
           const std::array<std::shared_ptr<Joint>, NJOINTS>& joints);

      /*!
      * @brief Destructor.
      */
      virtual ~Limb() = default;

      /*!
       * @brief Get the number of the joints of the Limb.
       * @return number of the limb's joints.
       */
      unsigned int getNJoints() const override;

      /*!
       * @brief Get the number of the links of the limb.
       * @return number of the limb's links.
       */
      unsigned int getNLinks() const override;

      /*!
       * @brief Get the Joint object associated to the joint name.
       * @param[in] name name of the Joint object that is returned
       * @return shared pointer pointing to the joint.
      */
      const Joint& getJoint(const std::string &name) const override;

      /*!
       * @brief Get the Link object associated to the name.
       * @param[in] name name of the Link object that is returned.
       * @return shared pointer pointing to the link.
       */
      const Link& getLink(const std::string &name) const override;

      /*!
       * @brief Get the limb end-effector.
       * @return shared pointer pointing to the limb end-effector. 
       */
      const Link& getEndEffector() const override;   

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      const ContainerBase<Joint> getJoints() const override;

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      const ContainerBase<Link> getLinks() const override;

   // protected:

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      ContainerBase<Joint> getJoints() override;

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      ContainerBase<Link> getLinks() override;


   private:

      //! Shared pointer pointing to the list of the limb's joints.
      Container<Joint, NJOINTS> joints_;

      //! Shared pointer pointing to the list of the limb's links.
      Container<Link, NLINKS> links_;
   };
} // namespace robotlib

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_