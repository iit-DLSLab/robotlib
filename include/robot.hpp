/*!
 * @file robot.hpp
 *
 * @brief Robot class definition and functions prototypes.
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

#ifndef _ROBOTLIB_ROBOT_HPP_
#define _ROBOTLIB_ROBOT_HPP_

#include "robot_base.hpp"

namespace robotlib
{
	/*!
     * @brief Robot class.
     * @details
     * This class represents a robot with a specific number of joints, links, legs and arms. It inherits from the RobotBase class and it overrides all the functions whose implementation requires the knowledge of the legs, arms, links and joints objects.
     * @tparam NJOINTS number of joints of the robot.
     * @tparam NLINKS number of links of the robot.
     * @tparam NLEGS number of legs of the robot.
     * @tparam NARMS number of arms of the robot.
     */
	template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
	class Robot : public RobotBase
	{
	public:
	    /*!
         * @brief Constructor.
         * @param[in] name name of the robot.
         * @param[in] trunk shared pointer pointing to the trunk object.
         * @param[in] legs shared pointer pointing to the robot's legs.
         * @param[in] arms shared pointer pointing to the robot's arms.
         */
		Robot(const std::string &name,
			  const std::shared_ptr<Trunk> trunk,
			  const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NLEGS>> legs,
			  const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NARMS>> arms);

		/*!
         * @brief Destructor.
         */
		virtual ~Robot();

		/*!
         * @brief Get robot's legs.
         * @return robot's legs as a shared pointer to a ContainerBase object.
         */
        virtual std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const override;

        /*!
         * @brief Get robot's arms.
         * @return robot's arms as a shared pointer to a ContainerBase object.
         */
        virtual std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const override;

        /*!
         * @brief Get robot's link from link's name.
         * @param[in] name name of the link.
         * @return a shared pointer pointing to the link.
         */
        virtual std::shared_ptr<Link> getLink(const std::string &name) override;

        /*!
         * @brief Get robot's joint from joint's name.
         * @param[in] name name of the joint.
         * @return a shared pointer pointing to the joint.
         */
        virtual std::shared_ptr<Joint> getJoint(const std::string &name) override;

        /*!
         * @brief Get robot's leg from leg's name.
         * @param[in] name name of the leg.
         * @return a shared pointer pointing to the leg.
         */
        virtual std::shared_ptr<LimbBase> getLeg(const std::string &name) override;

        /*!
         * @brief Get robot's arm from arm's name.
         * @param[in] name name of the arm.
         * @return a shared pointer pointing to the arm.
         */
        virtual std::shared_ptr<LimbBase> getArm(const std::string &name) override;

        /*!
         * @brief Get number of robot's legs.
         * @return number of robot's legs.
         */
        virtual int getNLEGS() override;

        /*!
         * @brief Get number of robot's arms.
         * @return number of robot's arms.
         */
        virtual int getNARMS() override;

        /*!
         * @brief Get number of robot's joints.
         * @return number of robot's joints.
         */
        virtual int getNJOINTS() override;

        /*!
         * @brief Get number of robot's links.
         * @return number of robot's links.
         */
        virtual int getNLINKS() override;

		/*!
         * @brief Get lower angle limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the lower limits of the joints' angles. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] q_min a joint state object to be filled with the lower limits of the joints' angles.
         */
        virtual void getMinJointAngle(JointState &q_min) override;

        /*!
         * @brief Get upper angle limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the upper limits of the joints' angles. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] q_max a joint state object to be filled with the upper limits of the joints' angles.
         */
        virtual void getMaxJointAngle(JointState &q_max) override;

        /*!
         * @brief Get maximum velocity limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the maximum velocity limits of the joints. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] qd_max a joint state object to be filled with the maximum velocity limits of the joints.
         */
        virtual void getMaxJointVelocity(JointState &qd_max) override;

        /*!
         * @brief Get maximum torque limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the maximum torque limits of the joints. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] tau_max a joint state object to be filled with the maximum torque limits of the joints.
         */
        virtual void getMaxJointEffort(JointState &tau_max) override;

		/*!
         * @brief Get the CoM of the trunk.
         * @return trunk's CoM.
         */
        virtual Eigen::Matrix<double, 3, 1> getTrunkCOM() const override;

	protected:
		/*!
         * @brief Set the child of a joint, that is a Link object.
         * @details
         * @param[in] joint joint to which associate the child.
         * @param[in] child child of the joint.
         */
		virtual void setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child);

		/*!
         * @brief Set the child of a link, that is a Joint object.
         * @details
         * @param[in] link link to which associate the child.
         * @param[in] child child of the link.
         */
		virtual void setChildOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> child);

		/*!
         * @brief Set the children of the trunk, that are Joint objects.
         * @details
         * @param[in] children children of the trunk.
         */
		virtual void setChildrenOfTrunk(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children);

		/*!
         * @brief Set the parent of a joint, that is a Link object.
         * @details
         * @param[in] joint joint to which associate the parent.
         * @param[in] parent parent of the joint.
         */
		virtual void setParentOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> parent);

		/*!
         * @brief Set the parent of a link, that is a Joint object.
         * @details
         * @param[in] link link to which associate the parent.
         * @param[in] parent parent of the link.
         */
		virtual void setParentOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> parent);

		/*!
         * @brief Set the limits to the joint in input
         * @details
         * @param[in] joint joint to which associate the limits.
         * @param[in] q_min minimum joint angle.
		 * @param[in] q_max maximum joint angle.
		 * @param[in] qd_max joint velocity limit.
		 * @param[in] tau_max joint torque limit.
         */
		virtual void setJointLimits(const std::shared_ptr<Joint> joint, 
							const double q_min, 
							const double q_max, 
							const double qd_max, 
							const double tau_max);

		//! Trunk of the robot
		const std::shared_ptr<Trunk> trunk_;

		//! Legs of the robot
		const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NLEGS>> legs_;

		//! Arms of the robot
		const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NARMS>> arms_;
	};
} // namespace robotlib

#include "robot.tpp"

#endif // _ROBOTLIB_ROBOT_HPP_