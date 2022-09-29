#ifndef _ROBOTLIB_ROBOT_HPP_
#define _ROBOTLIB_ROBOT_HPP_

#include "robot_base.hpp"

namespace robotlib
{
	/// NJOINTS: Robot joints. NLINKS: Robot links.
	template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
	class Robot : public RobotBase
	{
	public:
		Robot(const std::string &name,
			  const std::shared_ptr<Trunk> trunk,
			  const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NLEGS>> legs,
			  const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NARMS>> arms);
		virtual ~Robot();

		// virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) override;	/// NB: TODO

		virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const override;
		virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const override;

		virtual const int getNLEGS() override;
		virtual const int getNARMS() override;
		virtual const int getNJOINTS() override;
		virtual const int getNLINKS() override;

		virtual const std::shared_ptr<Link> getLink(const std::string &name) override;
		virtual const std::shared_ptr<Joint> getJoint(const std::string &name) override;
		virtual const std::shared_ptr<LimbBase> getLeg(const std::string &name) override;

        virtual void getMinJointAngle(JointState &q_min) override;
        virtual void getMaxJointAngle(JointState &q_max) override;
        virtual void getMaxJointVelocity(JointState &qd_max) override;
        virtual void getMaxJointEffort(JointState &tau_max) override;

		virtual const Eigen::Matrix<double, 3, 1>& getTrunkCOM() const override;

	protected:
		const std::shared_ptr<Trunk> trunk_; //! Trunk of the robot

		const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NLEGS>> legs_; //! Legs of the robot
		const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NARMS>> arms_; //! Arms of the robot

		void setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child);
		void setChildOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> child);
		void setChildrenOfTrunk(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children);

		void setParentOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> parent);
		void setParentOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> parent);


		void setJointLimits(const std::shared_ptr<Joint> joint, 
							const double q_min, 
							const double q_max, 
							const double qd_max, 
							const double tau_max);
	};
} // namespace robotlib

#include "robot.tpp"

#endif // _ROBOTLIB_ROBOT_HPP_