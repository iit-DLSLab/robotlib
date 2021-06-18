#ifndef _ROBOTLIB_ROBOT_HPP_
#define _ROBOTLIB_ROBOT_HPP_

#include "robot_base.hpp"
#include <fstream>

namespace dls
{
	namespace robotlib
	{
		template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS = 0>
		class Robot : public RobotBase
		{
		public:
			// Constructor
			Robot(const std::string &name, const std::array<std::shared_ptr<LimbBase>, NLEGS> &legs);
			virtual ~Robot();

			virtual Iterator<const std::shared_ptr<LimbBase>> begin() override;
			virtual Iterator<const std::shared_ptr<LimbBase>> end() override;

			// Get functions
			const std::array<std::shared_ptr<LimbBase>, NLEGS> getLegs();
			const std::shared_ptr<LimbBase> getLeg(const int id) override;

			virtual const int getNLEGS() override;
			virtual const int getNJOINTS() override;
			virtual const int getNLINKS() override;

		protected:
			const std::array<std::shared_ptr<LimbBase>, NLEGS> legs_; //! Legs of the robot
																	  //const Trunk trunk_;													//! Trunk of the robot
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_ROBOT_HPP_
