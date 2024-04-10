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
#include "utils/container.hpp"

namespace robotlib
{
	/*!
     * @brief Robot class.
     * @details
     * This class represents a robot with a specific number of joints, links, legs and arms. It inherits from the RobotBase class and it overrides all the functions whose implementation requires the knowledge of the legs, arms, links and joints objects.
     * @tparam NJOINTS number of joints of the robot.
     * @tparam NLINKS number of links of the robot.
     * @tparam NLIMBS number of limbs of the robot.
     */
	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	class Robot : public RobotBase
	{
	public:
	    /*!
         * @brief Constructor.
         * @param[in] name name of the robot.
         * @param[in] trunk shared pointer pointing to the trunk object.
         * @param[in] limbs robot's limbs.
         */
		Robot(const std::string& name,
			  const std::shared_ptr<Trunk>& trunk,
			  Container<LimbBase, NLIMBS>& limbs);

		/*!
         * @brief Destructor.
         */
		virtual ~Robot() = default;

        /*!
         * @brief Get number of robot's legs.
         * @return number of robot's legs.
         */
        virtual unsigned int getNLEGS() const override;

        /*!
         * @brief Get number of robot's arms.
         * @return number of robot's arms.
         */
        virtual unsigned int getNARMS() const override;

        /*!
         * @brief Get number of robot's limbs.
         * @return number of robot's limbs.
         */
        virtual unsigned int getNLIMBS() const override;

        /*!
         * @brief Get number of robot's joints.
         * @return number of robot's joints.
         */
        virtual unsigned int getNJOINTS() const override;

        /*!
         * @brief Get number of robot's links.
         * @return number of robot's links.
         */
        virtual unsigned int getNLINKS() const override;

        /*!
         * @brief Get robot's trunk.
         * @return reference to trunk link.
         */
        virtual const std::shared_ptr<Trunk>& getTrunk() const override;

        /*!
         * @brief Get robot's joint from joint's name.
         * @param[in] name name of the joint.
         * @return reference to link or throw except.
         */
        const Joint& getJoint(const std::string &name) const override;

        /*!
         * @brief Get robot's joints.
         * @return robot's joints as a ContainerBase object.
         */
        const ContainerBase<Joint> getJoints() const override;

        /*!
         * @brief Get robot's link from link's name.
         * @param[in] name name of the link.
         * @return reference to link or throw except.
         */
        const Link& getLink(const std::string &name) const override;

        /*!
         * @brief Get a list of all links of the robot.
         * @return a list of links of the robot.
         */
        const ContainerBase<Link> getLinks() const override;

        /*!
         * @brief Get robot's limb from limb's name.
         * @param[in] name name of the limb.
         * @return reference to limb or throw except.
         */
        const LimbBase& getLimb(const std::string &name) const override;

		/*!
         * @brief Get robot's limbs.
         * @return robot's limbs as a ContainerBase object.
         */
        const ContainerBase<LimbBase> getLimbs() const override;

        /*!
         * @brief Get robot's legs.
         * @return robot's legs as a vector object.
         */
        const std::vector<std::shared_ptr<const LimbBase>> getLegs() const override;

        /*!
         * @brief Get robot's arms.
         * @return robot's arms as a vector object.
         */
        const std::vector<std::shared_ptr<const LimbBase>> getArms() const override;

		/*!
         * @brief Get lower angle limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the lower limits of the joints' angles. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] q_min a joint state object to be filled with the lower limits of the joints' angles.
         */
        virtual void getMinJointAngle(JointState& q_min) override;

        /*!
         * @brief Get upper angle limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the upper limits of the joints' angles. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] q_max a joint state object to be filled with the upper limits of the joints' angles.
         */
        virtual void getMaxJointAngle(JointState& q_max) override;

        /*!
         * @brief Get maximum velocity limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the maximum velocity limits of the joints. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] qd_max a joint state object to be filled with the maximum velocity limits of the joints.
         */
        virtual void getMaxJointVelocity(JointState& qd_max) override;

        /*!
         * @brief Get maximum torque limit of each joint.
         * @details
         * A reference to a JointState instance is passed as input and it is set with the maximum torque limits of the joints. This avoids returning a new JointState object, leading to dynamic memory allocation.
         * @param[out] tau_max a joint state object to be filled with the maximum torque limits of the joints.
         */
        virtual void getMaxJointEffort(JointState& tau_max) override;

		/*!
         * @brief Get the CoM of the trunk.
         * @return trunk's CoM.
         */
        virtual Eigen::Vector3d getTrunkCOM() const override;

    protected:

		//! Trunk of the robot
		const std::shared_ptr<Trunk> trunk_;

    private:
		//! Limbs of the robot
		Container<LimbBase, NLIMBS> limbs_;

        //! List of pointes to the joints of the robot
		Container<Joint, NJOINTS> joints_;

        //! Joints of the robot
		Container<Link, NLINKS> links_;

	};
} // namespace robotlib

#include "robot.tpp"

#endif // _ROBOTLIB_ROBOT_HPP_