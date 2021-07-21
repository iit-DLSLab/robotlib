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
				  const std::array<std::shared_ptr<LimbBase>, NLEGS> &legs,
				  const std::array<std::shared_ptr<LimbBase>, NARMS> &arms);
			virtual ~Robot();

			const std::array<std::shared_ptr<LimbBase>, NLEGS> getLegs(); /// Not currently used
			const std::array<std::shared_ptr<LimbBase>, NARMS> getArms(); /// Not currently used

			const std::shared_ptr<LimbBase> &getLeg(const int id) override;
			const std::shared_ptr<LimbBase> &getArm(const int id) override;

			virtual const int getNLEGS() override;
			virtual const int getNARMS() override;
			virtual const int getNJOINTS() override;
			virtual const int getNLINKS() override;

		protected:
			const std::array<std::shared_ptr<LimbBase>, NLEGS> legs_; //! Legs of the robot
			const std::array<std::shared_ptr<LimbBase>, NARMS> arms_; //! Arms of the robot
		};
	} // namespace robotlib
} // namespace dls

#include "robot.tpp"

#endif // _ROBOTLIB_ROBOT_HPP_
