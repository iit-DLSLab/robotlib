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
#include "link.hpp"


#include <memory>

namespace robotlib
{
	class Link;

	/*!
     * @brief Joint class.
     * @details
     * This class represents a generic robot joint and extends the abstract class Frame.
     */
	class Joint : public Frame
	{
		friend class Link;

	public:
		/*!
         * @brief Constructor.
		 * @param[in] name name of the joint.
         * @param[in] parent parent link.
         */
		Joint(const std::string& name);//, Link* parent = NULL);

		Joint();

		/*!
         * @brief Destructor.
         */
		virtual ~Joint() = default;

		//! Robot is a friend class to let it use the private methods of the Joint class.
		friend class Robot;

		/*!
		 * @brief Get the Joint minimum angle.
		 * @return minimum joint angle.
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

		/*!
         * @brief Set the limits to the joint in input
         * @details
         * @param[in] q_min minimum joint angle.
		 * @param[in] q_max maximum joint angle.
		 * @param[in] qd_max joint velocity limit.
		 * @param[in] tau_max joint torque limit.
         */
		virtual void setJointLimits(const double& q_min, const double& q_max, const double& qd_max, const double& tau_max);

		// operator==
		bool operator==(const Joint& rhs) const;

        Joint &operator=(const Joint &rhs);
		
		// order in the limb
		int sub_id;
		// order at robot level
		int id;
		// limb id
		int limb_id;
	protected:


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
      
	  	//! Check if the joint is attached to a robot.
      	bool isAttached() const;

	private:

		/*!
		 * @brief Set the Joint child object, that is a Link object.
		 * @details This method is private and can only be used by the friend class Link 
		 * This restriction garantee the coerence in the definition of the kinematic chain
		 * @param[in] child the Link child object to be set.
		 */
		void setChild(Link* child);

		//! Minimum joint angle.
		double q_min_;
		//! Maximum joint angle.
		double q_max_;
		//! Joint velocity limit.
		double qd_max_;
		//! Joint torque limit.
		double tau_max_;
	};
	typedef std::shared_ptr<Joint> JointPtr;
} // namespace robotlib

#endif // _ROBOTLIB_JOINT_HPP_