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

#include <memory>
#include "frame.hpp"
#include "joint.hpp"
#include "link.hpp"
#include "limb.hpp"
#include <vector>

namespace robotlib
{
   /*!
	 * @brief Limb class.
	 * @details
	 * This class represents a robot limb (leg/arm/other) with a specific number of joints and links.
    */
   
   class Limb
   {
      //! Robot is a friend class to let it use the private methods of the Joint class.
		friend class Robot;

   public:
      /*!
      * @brief Constructor.
      * @param[in] name name of the limb.
      * @param[in] joints array of shared pointers pointing to limb's joints.
      * @param[in] links array of shared pointers pointing to limb's links.
      */
      Limb(const std::string& name,
           const std::vector<LinkPtr>& links,
           const std::vector<JointPtr>& joints,
           const std::string type = "generic");

      Limb();

      Limb( const std::string& name,
            const std::vector<std::string>& link_names,
            const std::vector<std::string>& joint_names,
            const std::string type = "generic");

      /*!
      * @brief Destructor.
      */
      virtual ~Limb() = default;

      /*!
       * @brief Get the number of the joints of the Limb.
       * @return number of the limb's joints.
       */
      unsigned int getNJoints() const;

      /*!
       * @brief Get the number of the links of the limb.
       * @return number of the limb's links.
       */
      unsigned int getNLinks() const;

      /*!
       * @brief Get the Joint object associated to the joint name.
       * @param[in] name name of the Joint object that is returned
       * @return shared pointer pointing to the joint.
      */
      const JointPtr getJoint(const std::string &name) const;

      /*!
       * @brief Get the Link object associated to the name.
       * @param[in] name name of the Link object that is returned.
       * @return shared pointer pointing to the link.
       */
      const LinkPtr getLink(const std::string &name) const;

      /*!
       * @brief Get the limb end-effector.
       * @return shared pointer pointing to the limb end-effector. 
       */
      const LinkPtr getEndEffector() const;   

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      const std::vector<JointPtr> getJoints() const;

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      const std::vector<LinkPtr> getLinks() const;

   // protected:

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      std::vector<JointPtr> getJoints();

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      std::vector<LinkPtr> getLinks();

      /*!
       * @brief Type of the the limb.
       * @return return a string with the type of the arm e.g, leg, arm, etc.
       */
      virtual std::string type() const;

      /*!
		 * @brief Get the name of the limb.
		 * @return name of the limb.
		 */
      const std::string& getName() const;

      bool operator==(const Limb& rhs) const;

      // Limb &operator=(const Limb &rhs);

      void setType(const std::string& type);

      //! Check if the limb is attached to a robot.
      bool isAttached() const;

      int id;
   private:
      //! Name of the limb.
      std::string name_;

      //! Shared pointer pointing to the list of the limb's joints.
      std::vector<JointPtr> joints_;

      //! Shared pointer pointing to the list of the limb's links.
      std::vector<LinkPtr> links_;

      //! Type of the limb (e.g., leg, arm, etc.).
      std::string type_;
   };
   typedef std::shared_ptr<Limb> LimbPtr;
} // namespace robotlib

#endif // _ROBOTLIB_LIMB_HPP_