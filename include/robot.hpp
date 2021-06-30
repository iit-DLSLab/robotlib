#ifndef _ROBOTLIB_ROBOT_HPP_
#define _ROBOTLIB_ROBOT_HPP_

#include "robot_base.hpp"

namespace dls
{
	namespace robotlib
	{
		/// NJOINTS: Robot joints. NLINKS: Robot links.
		template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS = 0>
		class Robot : public RobotBase
		{
		public:
			Robot(const std::string &name,
				  const std::array<std::shared_ptr<LimbBase>, NLEGS> &legs,
				  const std::shared_ptr<ForwardKinematicsBase> &fk);
			virtual ~Robot();

			virtual Iterator<const std::shared_ptr<LimbBase>> begin() override;
			virtual Iterator<const std::shared_ptr<LimbBase>> end() override;

			const std::array<std::shared_ptr<LimbBase>, NLEGS> getLegs();
			const std::shared_ptr<LimbBase> getLeg(const int id) override;

			virtual const int getNLEGS() override;
			virtual const int getNJOINTS() override;
			virtual const int getNLINKS() override;

			virtual std::shared_ptr<ForwardKinematicsBase> getForwardKinematics();

		protected:
			const std::array<std::shared_ptr<LimbBase>, NLEGS> legs_; //! Legs of the robot
																	  //const Trunk trunk_;													//! Trunk of the robot
		private:
			std::shared_ptr<ForwardKinematicsBase> fk_;
		};
	} // namespace robotlib
} // namespace dls

#include "../src/robot.tpp"

#endif // _ROBOTLIB_ROBOT_HPP_
