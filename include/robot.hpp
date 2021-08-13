#ifndef _ROBOTLIB_ROBOT_HPP_
#define _ROBOTLIB_ROBOT_HPP_

#include "robot_base.hpp"

namespace dls
{
	namespace robotlib
	{
		/// NJOINTS: Robot joints. NLINKS: Robot links.
		template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
		class Robot : public RobotBase
		{
		public:
			Robot(const std::string &name,
				  const std::shared_ptr<Trunk> trunk,
				  const std::shared_ptr<Container<LimbBase, NLEGS>> &legs,
				  const std::shared_ptr<Container<LimbBase, NARMS>> &arms);
			virtual ~Robot();

			virtual const std::shared_ptr<LimbBase> getLeg(const int id) override;
			virtual const std::shared_ptr<LimbBase> getArm(const int id) override;

            // virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) override;	/// NB: TODO

			virtual const std::shared_ptr<ContainerBase<LimbBase>> getLegs() override;
			virtual const std::shared_ptr<ContainerBase<LimbBase>> getArms() override;

			virtual const int getNLEGS() override;
			virtual const int getNARMS() override;
			virtual const int getNJOINTS() override;
			virtual const int getNLINKS() override;

			virtual const std::shared_ptr<Link> getLink(const std::string &name) override;
			virtual const std::shared_ptr<Joint> getJoint(const std::string &name) override;

		protected:
			const std::shared_ptr<Trunk> trunk_; //! Trunk of the robot

			const std::shared_ptr<Container<LimbBase, NLEGS>> legs_; //! Legs of the robot
			const std::shared_ptr<Container<LimbBase, NARMS>> arms_; //! Arms of the robot

			void setChildrenOfTrunk(const std::shared_ptr<ContainerBase<Joint>> children);
			void setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child);
			void setChildOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> child);

			void setParentOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> parent);
			void setParentOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> parent);
		};
	} // namespace robotlib
} // namespace dls

#include "robot.tpp"

#endif // _ROBOTLIB_ROBOT_HPP_
