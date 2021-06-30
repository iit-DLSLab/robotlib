#ifndef _ROBOTLIB_POSE_HPP_
#define _ROBOTLIB_POSE_HPP_

#include <Eigen/Dense>

namespace dls
{
	namespace robotlib
	{
		class Pose
		{
		public:
			Pose(const Eigen::Vector3d &p, const Eigen::Vector3d &ori);

			~Pose();

			const Eigen::Vector3d &getPos();
			const Eigen::Vector3d &getOri();

		private:
			const Eigen::Vector3d p_;	//! 3D position
			const Eigen::Vector3d ori_; //! 3D orientation
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_POSE_HPP_
