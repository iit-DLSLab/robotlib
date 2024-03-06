/*!
 * @file limb_base.hpp
 *
 * @brief LimbBase class definition and functions prototypes.
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

#ifndef _ROBOTLIB_LIMB_BASE_HPP_
#define _ROBOTLIB_LIMB_BASE_HPP_

#include <memory>
#include "frame.hpp"
#include "joint.hpp"
#include "link.hpp"
#include "utils/container_base.hpp"

namespace robotlib
{
   /*!
	 * @brief LimbBase class.
	 * @details
	 * This class represents a generic robot Limb
    * It is an interface used to abstract a limb object, independently from the limb morphology.
	 */
   class LimbBase
   {
      //! Robot is a friend class to let it use the private methods of the class.
		template <unsigned int NLMBS, unsigned int NLNKS, unsigned int NJONTS>
		friend class Robot;

   public:

     /*!
      * @brief Constructor.
      * @param[in] name name of the limb.
      */
      LimbBase(const std::string& name);

      /*!
      * @brief Destructor.
      */
      virtual ~LimbBase() = default;

      /*!
		 * @brief Get the name of the limb.
		 * @return name of the limb.
		 */
      const std::string& getName() const;

      /*!
       * @brief Get the number of the joints of the Limb.
       * @return number of the limb's joints.
       */
      virtual unsigned int getNJoints() const = 0;

      /*!
       * @brief Get the number of the links of the limb.
       * @return number of the limb's links.
       */
      virtual unsigned int getNLinks() const = 0;

      /*!
       * @brief Get the Joint object associated to the joint name.
       * @param[in] name name of the Joint object that is returned
       * @return shared pointer pointing to the joint.
	   */
	   virtual const Joint& getJoint(const std::string &name) const = 0;

      /*!
       * @brief Get the Link object associated to the name.
       * @param[in] name name of the Link object that is returned.
       * @return shared pointer pointing to the link.
       */
      virtual const Link& getLink(const std::string &name) const = 0;

      /*!
       * @brief Get the limb end-effector.
       * @return shared pointer pointing to the limb end-effector. 
       */
      virtual const Link& getEndEffector() const = 0;

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      virtual const ContainerBase<Joint> getJoints() const = 0;

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      virtual const ContainerBase<Link> getLinks() const = 0;

      /*!
       * @brief Type of the the limb.
       * @return return a string with the type of the arm e.g, leg, arm, etc.
       */
      virtual std::string type() const = 0;

      /*!
       * @brief Operator for comparing two limbs.
       * @return return true if they are considered equal.
       */
      bool operator==(const LimbBase& rhs) const;

   // protected:

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      virtual ContainerBase<Joint> getJoints() = 0;

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      virtual ContainerBase<Link> getLinks() = 0;

   private:

      //! Name of the limb.
      const std::string name_;
   };
} // namespace robotlib

#endif // _ROBOTLIB_LIMB_BASE_HPP_