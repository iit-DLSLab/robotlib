/*!
 * @file limb_base.hpp
 *
 * @brief LimbBase class definition and functions prototypes.
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_LIMB_BASE_HPP_
#define _ROBOTLIB_LIMB_BASE_HPP_

#include <memory>
#include "frame.hpp"
#include "utils.hpp"

namespace robotlib
{
   class Link;
   class Joint;

   /*!
	 * @brief LimbBase class.
	 * @details
	 * This class represents a generic robot Limb (Leg or Arm). It is an interface used to abstract a limb object, independently from the limb morphology.
	 */
   class LimbBase
   {
   public:
      /*!
      * @brief Constructor.
      * @param[in] name name of the limb.
      */
      LimbBase(const std::string &name);

      /*!
      * @brief Destructor.
      */
      virtual ~LimbBase();

      /*!
		 * @brief Get the name of the limb.
		 * @return name of the limb.
		 */
      virtual std::string getName() const;

      /*!
       * @brief Get the number of the joints of the Limb.
       * @return number of the limb's joints.
       */
      virtual int getNJoints() const = 0;

      /*!
       * @brief Get the number of the links of the limb.
       * @return number of the limb's links.
       */
      virtual int getNLinks() const = 0;

      /*!
       * @brief Get the Joint object associated to the joint name.
       * @param[in] name name of the Joint object that is returned
       * @return shared pointer pointing to the joint.
       */
      virtual std::shared_ptr<Joint> getJoint(const std::string &name) const = 0;

      /*!
       * @brief Get the Link object associated to the name.
       * @param[in] name name of the Link object that is returned.
       * @return shared pointer pointing to the link.
       */
      virtual std::shared_ptr<Link> getLink(const std::string &name) const = 0;

      /*!
       * @brief Get the limb end-effector.
       * @return shared pointer pointing to the limb end-effector. 
       */
      virtual std::shared_ptr<Link> getEndEffector() const = 0;

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      virtual std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getJoints() const = 0;

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      virtual std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> getLinks() const = 0;

      /*!
       * @brief Get the name of the joint's parent of the limb.
       * @param[in] joint Joint object for which you get the name of the parent.
       * @return name of the joint's parent. 
       */
      virtual std::string jointToParentName(const std::shared_ptr<Joint> joint) const = 0;

      /*!
       * @brief Get the name of the joint's child of the limb.
       * @param[in] joint Joint object for which you get the name of the child.
       * @return name of the joint's child.
       */
      virtual std::string jointToChildName(const std::shared_ptr<Joint> joint) const = 0;

      /*!
       * @brief Get the name of link's parent of the limb.
       * @param[in] link Link object for which you get the name of the parent.
       * @return name of the link's parent.
       */
      virtual std::string linkToParentName(const std::shared_ptr<Link> link) const = 0;

      /*!
       * @brief Get the name of the link's child of the limb.
       * @param[in] link Link object for which you get the name of the child.
       * @return name of the link's child.
       */
      virtual std::string linkToChildName(const std::shared_ptr<Link> link) const = 0; 

   private:
      //! Name of the limb.
      const std::string name_;
   };
} // namespace robotlib

#endif // _ROBOTLIB_LIMB_BASE_HPP_