/**
 * @file joint.hpp
 *
 * @brief Joint class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
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

	/**
     * @brief Joint class
     * @details
     * This class represents a generic robot joint and extends the abstract class Frame
     */
	class Joint : public Frame
	{
	public:
		Joint(const std::string &name);

		virtual ~Joint();

		template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
		friend class Robot;

		/**
		 * @brief Get the name of the Joint
		 * @return const std::string
		 */
		virtual std::string getName() const override;
		/**
		 * @brief Get the Joint parent object, that is a Link object
		 * @return const std::shared_ptr<Link>
		 */
		std::shared_ptr<Link> getParent() const;
		/**
		 * @brief Get the Joint child object, that is a Link object
		 * @return const std::shared_ptr<Link>
		 */
		std::shared_ptr<Link> getChild() const;

		/**
		 * @brief Get the Joint minimum angle
		 * @return const double
		 */
		double getMinAngle() const;
		/**
		 * @brief Get the Joint maximum angle
		 * @return const double
		 */
		double getMaxAngle() const;
		/**
		 * @brief Get the Joint limit velocity
		 * @return const double
		 */
		double getMaxVelocity() const;
		/**
		 * @brief Get the Joint limit effort
		 * @return const double
		 */
		double getMaxEffort() const;


	private:
		/**
		 * @brief Set the Joint parent object, that is a Link object
		 * @param parent the Joint parent object to be set
		 * @return void
		 */
		void setParent(const std::shared_ptr<Link> parent);
		/**
		 * @brief Set the Joint child object, that is a Link object
		 * @param parent the Joint child object to be set
		 * @return void
		 */
		void setChild(const std::shared_ptr<Link> child);		
		/**
		 * @brief Set the Joint minimum angle
		 * @return const double
		 */
		void setMinAngle(const double q_min);
		/**
		 * @brief Set the Joint maximum angle
		 * @return const double
		 */
		void setMaxAngle(const double q_max);
		/**
		 * @brief Set the Joint limit velocity
		 * @return const double
		 */
		void setMaxVelocity(const double qd_max);
		/**
		 * @brief Set the Joint limit effort
		 * @return const double
		 */
		void setMaxEffort(const double tau_max);

		/**
		 * @brief Joint limits
		 */
		double q_min_;
		double q_max_;
		double qd_max_;
		double tau_max_;

	protected:
		std::shared_ptr<Link> parent_;
		std::shared_ptr<Link> child_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_JOINT_HPP_