/*!
 * @file joint.hpp
 *
 * @brief Joint class definition and functions prototypes.
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

#ifndef _ROBOTLIB_JOINT_HPP_
#define _ROBOTLIB_JOINT_HPP_

#include "frame.hpp"
#include <memory>

namespace robotlib
{
	class Link;
	template <unsigned int NJOINTS, unsigned int NLINKS>
	class Limb;

	/*!
     * @brief Joint class.
     * @details
     * This class represents a generic robot joint and extends the abstract class Frame.
     */
	class Joint : public Frame
	{
	public:
		/*!
         * @brief Constructor.
         * @param[in] name name of the joint.
         */
		Joint(const std::string &name);

		/*!
         * @brief Destructor.
         */
		virtual ~Joint();

		//! Robot is a friend class to let it use the private methods of the Joint class.
		template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
		friend class Robot;

		/*!
		 * @brief Get the name of the Joint.
		 * @return name of the joint.
		 */
		virtual std::string getName() const override;

		/*!
		 * @brief Get the Joint parent object, that is a Link object.
		 * @return shared pointer pointing to a Link object.
		 */
		virtual std::shared_ptr<Link> getParent() const;

		/*!
		 * @brief Get the Joint child object, that is a Link object.
		 * @return shared pointer pointing to a Link object.
		 */
		virtual std::shared_ptr<Link> getChild() const;

		/*!
		 * @brief Get the Joint minimum angle.
		 * @return minimum joint angle.
		 */
		virtual double getMinAngle() const;

		/*!
		 * @brief Get the Joint maximum angle.
		 * @return maximum joint angle.
		 */
		virtual double getMaxAngle() const;

		/*!
		 * @brief Get the Joint limit velocity.
		 * @return joint velocity limit.
		 */
		virtual double getMaxVelocity() const;

		/*!
		 * @brief Get the Joint limit effort.
		 * @return joint limit effort.
		 */
		virtual double getMaxEffort() const;

	private:
		/*!
		 * @brief Set the Joint parent object, that is a Link object.
		 * @param[in] parent the Link parent object to be set.
		 */
		void setParent(const std::shared_ptr<Link> parent);

		/*!
		 * @brief Set the Joint child object, that is a Link object.
		 * @param[in] child the Link child object to be set.
		 */
		void setChild(const std::shared_ptr<Link> child);

		/*!
		 * @brief Set the Joint minimum angle.
		 * @param[in] q_min minimum joint angle.
		 */
		void setMinAngle(const double q_min);

		/*!
		 * @brief Set the Joint maximum angle.
		 * @param[in] q_max maximum joint angle.
		 */
		void setMaxAngle(const double q_max);

		/*!
		 * @brief Set the Joint velocity limit.
		 * @param[in] qd_max joint velocity limit.
		 */
		void setMaxVelocity(const double qd_max);

		/*!
		 * @brief Set the Joint torque limit.
		 * @param[in] tau_max joint torque limit.
		 */
		void setMaxEffort(const double tau_max);

		//! Minimum joint angle.
		double q_min_;
		//! Maximum joint angle.
		double q_max_;
		//! Joint velocity limit.
		double qd_max_;
		//! Joint torque limit.
		double tau_max_;

	protected:
		//! Parent of the joint.
		std::shared_ptr<Link> parent_;
		//! Child of the joint.
		std::shared_ptr<Link> child_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_JOINT_HPP_