/*!
 * @file link.hpp
 *
 * @brief Link class definition and functions prototypes.
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

#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"
#include "joint.hpp"
#include "utils/container_base.hpp"

#include <memory>

namespace robotlib
{
	class Joint;
	
	/*!
	 * @brief Link class.
	 * @details
	 * This class represents a generic robot link and extends the abstract class Frame.
	 */
	class Link : public Frame
	{
		template <unsigned int NLMBS, unsigned int NLNKS, unsigned int NJONTS>
		friend class Robot;
		friend class Joint;
		
	public:

		/*!
         * @brief Constructor.
         * @param[in] name name of the link.
         */
		Link(const std::string &name);

		/*!
         * @brief Constructor.
         * @param[in] name name of the link.
		 * @param[in] parent parent joint.
         */
		Link(const std::string &name, Joint& parent);


		/*!
         * @brief Destructor.
         */
		~Link() = default;
		
		//! Robot is a friend class to let it use the private methods of the Link class.
		template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
		friend class Robot;

		/*!
		 * @brief Get the parent object of the link, that is a Joint object.
		 * @return parent joint of the link.
		 */
		std::shared_ptr<const Joint> getParent() const;

		/*!
		 * @brief Get the children of the link, that are Joint objects.
		 * @return joints that are children of the link.
		 */
		const std::vector<std::shared_ptr<Joint>>& getChildren() const;

	protected:

		/*!
		 * @brief Set the parent of the link, that is a Joint object.
		 * @param[in] parent the parent of the link to be set.
		 */
		void setParent(Joint& parent);
	

	private:

		/*!
		 * @brief Set the child of the link, that is a Joint object.
		 * @details This method is private and can only be used by the friend class Joint
		 * This restriction garantee the coerence in the definition of the kinematic chain
		 * @param[in] child the child of the link to be set.
		 */
		void addChild(Joint& child);

		//! Children of the link.
		std::vector<std::shared_ptr<Joint>> children_;

		//! Parent of the link.
		std::shared_ptr<Joint> parent_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_LINK_HPP_