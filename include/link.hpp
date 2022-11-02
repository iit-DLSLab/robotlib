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
		virtual std::string getName() const override;
		/**
		 * @brief Get the Link parent object, that is a Joint object
		 * @return const std::shared_ptr<Joint>
		 */
		std::shared_ptr<Joint> getParent() const;
		/**
		 * @brief Get the Link child object, that is a Joint object
		 * @return const std::shared_ptr<Joint>
		 */
		std::shared_ptr<Joint> getChild() const;
		/**
		 * @brief Get the Link children object, that is a ContainerBase of Joint objects
		 * @return const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>>
		 */
		  /// TODO:
          /// - getChildren could invoke getChild when it contains a single element
          /// - getChildren should not require a user to write a for loop (it should be done automatically from Robotlib and hidden)
          ///   and should return an easier data structure (e.g. a std::array)
          /// - both getChild and getChildren can return a nullptr. However, only getChildren can give a segfault because to access
          ///   its elements you need the annoying iteration over the ContainerBase structure
		std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getChildren() const;

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