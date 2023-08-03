/**
 * @file limb_base.hpp
 *
 * @brief LimbBase class definition and functions prototypes
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
#include "utils/container_base.hpp"

namespace robotlib
{
   class Link;
   class Joint;

   /**
	 * @brief LimbBase class
	 * @details
	 * This class represent a generic robot Limb (Leg/Arm) and contains the virtual methods implemented in the Limb class
	 */
   class LimbBase
   {
   public:
      LimbBase(const std::string &name);

      virtual ~LimbBase();

      /**
		 * @brief Get the name of the LimbBase
		 * @return const std::string
		 */
      const std::string getName() const;
      /**
       * @brief Get the number of links of the Limb
       * @return const int
       */
      virtual int getNJoints() const = 0;
      /**
       * @brief Get the joints of the Limb
       * @return const int
       */
      virtual int getNLinks() const = 0;
      /**
       * @brief Get the Joint object associated to the name
       * @param name name of the Joint object that is returned
       * @return const std::shared_ptr<Joint> 
       */
      virtual const std::shared_ptr<Joint> getJoint(const std::string &name) const = 0;
      /**
       * @brief Get the Link object associated to the name
       * @param name name of the Link object that is returned
       * @return const std::shared_ptr<Link> 
       */
      virtual const std::shared_ptr<Link> getLink(const std::string &name) const = 0;
      /**
       * @brief Get the Link object correspondent to the Limb end-effector
       * @return const std::shared_ptr<Link> 
       */
      virtual const std::shared_ptr<Link> getEndEffector() const = 0; //overridden by Limb class
      /**
       * @brief Get the joints of the Limb
       * @return const std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>>
       */
      virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getJoints() const = 0;
      /**
       * @brief Get the links of the Limb
       * @return const std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> 
       */
      virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> getLinks() const = 0;
      /**
       * @brief Get the name of the Limb joint parent
       * @param joint Joint object for which you get the name of the parent
       * @return const std::string 
       */
      virtual const std::string jointToParentName(const std::shared_ptr<Joint> joint) const = 0; /// overridden by Glue
      /**
       * @brief Get the name of the Limb joint child
       * @param joint Joint object for which you get the name of the child
       * @return const std::string 
       */
      virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) const = 0; /// overridden by Glue
      /**
       * @brief Get the name of the Limb link parent
       * @param joint Link object for which you get the name of the parent
       * @return const std::string 
       */
      virtual const std::string linkToParentName(const std::shared_ptr<Link> joint) const = 0; /// overridden by Glue
      /**
       * @brief Get the name of the Limb link child
       * @param joint Link object for which you get the name of the child
       * @return const std::string 
       */
      virtual const std::string linkToChildName(const std::shared_ptr<Link> joint) const = 0; /// overridden by Glue

   private:
      const std::string name_;
   };
} // namespace robotlib

#endif // _ROBOTLIB_LIMB_BASE_HPP_