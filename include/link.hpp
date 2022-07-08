/**
 * @file link.hpp
 *
 * @brief Link class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"
#include "utils.hpp"
#include <memory>

#include <iostream>

namespace robotlib
{
	class Joint;
	template <unsigned int NJOINTS, unsigned int NLINKS>
	class Limb;

	/**
	 * @brief Link class
	 * @details
	 * This class represents a generic robot link and extends the abstract class Frame
	 */
	class Link : public Frame
	{
	public:
		Link(const std::string &name);

		virtual ~Link();

		template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
		friend class Robot;

		/**
		 * @brief Get the name of the Link
		 * @return const std::string
		 */
		virtual const std::string getName() const override;
		/**
		 * @brief Get the Link parent object, that is a Joint object
		 * @return const std::shared_ptr<Joint>
		 */
		const std::shared_ptr<Joint> getParent() const;
		/**
		 * @brief Get the Link child object, that is a Joint object
		 * @return const std::shared_ptr<Joint>
		 */
		const std::shared_ptr<Joint> getChild() const;

	protected:
		/**
		 * @brief Set the Link children, that is a ContainerBase of Joint
		 * @param children the Link children to be set
		 * @return void
		 */
		void setChildren(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children);
		/**
		 * @brief Set the Link parent object, that is a Joint object
		 * @param parent the Link parent to be set
		 * @return void
		 */
		void setParent(const std::shared_ptr<Joint> parent);
		/**
		 * @brief Set the Link child object, that is a Joint object
		 * @param child the Link child to be set
		 * @return void
		 */
		void setChild(const std::shared_ptr<Joint> child);

		std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children_;
		std::shared_ptr<Joint> parent_;
		std::shared_ptr<Joint> child_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_LINK_HPP_