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
#include "joint.hpp"
#include "link.hpp"
#include "utils/container.hpp"

namespace robotlib
{
   /*!
	 * @brief Limb class.
	 * @details
	 * This class represents a robot limb (leg/arm) with a specific number of joints and links and extends the abstract class LimbBase.
	 * @tparam NJOINTS number of joints of the limb.
    * @tparam NLINKS number of links of the limb.
    */
   template <unsigned int NJOINTS, unsigned int NLINKS>
   class Limb : public LimbBase
   {
   public:
      /*!
      * @brief Constructor.
      * @param[in] name name of the limb.
      * @param[in] joints array of shared pointers pointing to limb's joints.
      * @param[in] links array of shared pointers pointing to limb's links.
      */
      Limb(const std::string &name,
           const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
           const std::array<std::shared_ptr<Link>, NLINKS> &links);

      /*!
      * @brief Destructor.
      */
      virtual ~Limb();

      /*!
       * @brief Get the number of the joints of the Limb.
       * @return number of the limb's joints.
       */
      virtual int getNJoints() const override;

      /*!
       * @brief Get the number of the links of the limb.
       * @return number of the limb's links.
       */
      virtual int getNLinks() const override;

      /*!
       * @brief Get the Joint object associated to the joint name.
       * @param[in] name name of the Joint object that is returned
       * @return shared pointer pointing to the joint.
      /**
      virtual std::shared_ptr<Joint> getJoint(const std::string &name) const override;

      /*!
       * @brief Get the Link object associated to the name.
       * @param[in] name name of the Link object that is returned.
       * @return shared pointer pointing to the link.
       */
      virtual std::shared_ptr<Link> getLink(const std::string &name) const override;

      /*!
       * @brief Get the limb end-effector.
       * @return shared pointer pointing to the limb end-effector. 
       */
      virtual std::shared_ptr<Link> getEndEffector() const override;   

      /*!
       * @brief Get the joints of the Limb.
       * @return joints of the limb.
       */
      virtual std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getJoints() const override { return joints_; };

      /*!
       * @brief Get the links of the Limb.
       * @return links of the limb.
       */
      virtual std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> getLinks() const override { return links_; };

   protected:
      //! Shared pointer pointing to the list of the limb's joints.
      const std::shared_ptr<const Container<std::shared_ptr<Joint>, NJOINTS>> joints_;

      //! Shared pointer pointing to the list of the limb's links.
      const std::shared_ptr<const Container<std::shared_ptr<Link>, NLINKS>> links_;
   };
} // namespace robotlib

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_