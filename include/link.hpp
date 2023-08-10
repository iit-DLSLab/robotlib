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
#include "utils/container_base.hpp"
#include <memory>

#include <iostream>

namespace robotlib
{
	class Joint;
	template <unsigned int NJOINTS, unsigned int NLINKS>
	class Limb;

	/*!
	 * @brief Link class.
	 * @details
	 * This class represents a generic robot link and extends the abstract class Frame.
	 */
	class Link : public Frame
	{
	public:
		/*!
         * @brief Constructor.
         * @param[in] name name of the link.
         */
		Link(const std::string &name);

		/*!
         * @brief Destructor.
         */
		virtual ~Link();
		
		//! Robot is a friend class to let it use the private methods of the Link class.
		template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
		friend class Robot;

        /*!
		 * @brief Get the name of the link.
		 * @return name of the link.
		 */
		virtual std::string getName() const override;

		/*!
		 * @brief Get the parent object of the link, that is a Joint object.
		 * @return parent joint of the link.
		 */
		virtual std::shared_ptr<Joint> getParent() const;

		/*!
		 * @brief Get the child object of the link, that is a Joint object.
		 * @return child joint of the link.
		 */
		virtual std::shared_ptr<Joint> getChild() const;

		/*!
		 * @brief Get the children of the link, that are Joint objects.
		 * @return joints that are children of the link.
		 */
		virtual std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getChildren() const;

	protected:
		/*!
		 * @brief Set the children of the link, that are Joint objects.
		 * @param[in] children the Link children to be set.
		 */
		virtual void setChildren(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children);

		/*!
		 * @brief Set the parent of the link, that is a Joint object.
		 * @param[in] parent the parent of the link to be set.
		 */
		virtual void setParent(const std::shared_ptr<Joint> parent);

		/*!
		 * @brief Set the child of the link, that is a Joint object.
		 * @param[in] child the child of the link to be set.
		 */
		virtual void setChild(const std::shared_ptr<Joint> child);

		//! Children of the link.
		std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children_;

		//! Parent of the link.
		std::shared_ptr<Joint> parent_;

		//! Child of the link.
		std::shared_ptr<Joint> child_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_LINK_HPP_